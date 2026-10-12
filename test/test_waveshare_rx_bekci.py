#!/usr/bin/env python3
# Copyright 2026 gunes
# Licensed under the Apache-2.0 License.
#
# waveshare_socketcan_bridge RX sessizlik bekcisi testleri (robotaksi_ws#173).
#
# 18.08 sonrasi Here4 olumleri: seri hat hata vermiyor, TX (RTCM) akiyor ama
# Here4'ten hic kare gelmiyor. Eski kod oturumu yalniz seri hatada / TX
# cokunce bitirdigi icin dis dongudeki yeniden acma hic calismiyordu.
#
# Testler gercek saate bagli DEGIL: kopru modulunun `time`'i sahte bir saatle
# degistiriliyor, select() zaman asimi bu saati ilerletiyor. Boylece "3 s
# sessizlik" gibi esikler milisaniyede ve deterministik test ediliyor.

import importlib.util
import os
import struct
import threading
import time as gercek_time

import pytest

KOPRU = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "scripts",
    "waveshare_socketcan_bridge.py",
)
FMT = "=IB3x8s"
EFF = 0x80000000
RTR = 0x40000000


@pytest.fixture
def kopru():
    # Her test temiz modul: monkeypatch'ler testler arasi sizmasin.
    spec = importlib.util.spec_from_file_location("ws_bridge_bekci", KOPRU)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# --------------------------------------------------------------------------- #
#  Sahte saat, seri port, CAN soketi
# --------------------------------------------------------------------------- #


class SahteSaat:
    """kopru.time yerine gecer. monotonic() yalniz ana thread'de ilerler."""

    def __init__(self):
        self.t = 1000.0
        self.uykular = []

    def monotonic(self):
        return self.t

    def time(self):
        return self.t

    def sleep(self, s):
        self.uykular.append(s)
        if threading.current_thread() is threading.main_thread():
            self.t += s


class Bekle:
    """Bu kadar sure seri porta hic bayt gelmez (select zaman asimina duser)."""

    def __init__(self, s):
        self.kalan = s


class Veri:
    """select hazir doner, read() bu baytlari verir (b"" = hatasiz bos okuma)."""

    def __init__(self, b, dt=0.0):
        self.b = bytes(b)
        self.dt = dt


class Hata:
    """select hazir doner, read() bu istisnayi atar."""

    def __init__(self, e, dt=0.0):
        self.e = e
        self.dt = dt


class SahteSeri:
    """Senaryoyu adim adim oynatan seri port. Senaryo bitince select()
    KeyboardInterrupt atar -> oturum_calistir None doner (test biter)."""

    def __init__(self, saat, adimlar, ilk_select=None):
        self.saat = saat
        self.adimlar = list(adimlar)
        self.in_waiting = 1
        self.yazilan = []
        self.kapandi = False
        self.ilk_select = ilk_select

    # select yardimcisi: hazir mi? Bekle adimlarini zaman asimiyla tuketir.
    def hazir_mi(self, timeout):
        if self.ilk_select is not None:
            kanca, self.ilk_select = self.ilk_select, None
            kanca()
        if not self.adimlar:
            raise KeyboardInterrupt()
        adim = self.adimlar[0]
        if isinstance(adim, Bekle):
            if adim.kalan > timeout:
                adim.kalan -= timeout
                self.saat.t += timeout
                return False
            self.saat.t += adim.kalan
            self.adimlar.pop(0)
            return self.hazir_mi(timeout - adim.kalan) if self.adimlar else False
        return True

    def read(self, _n):
        adim = self.adimlar.pop(0)
        self.saat.t += adim.dt
        if isinstance(adim, Hata):
            raise adim.e
        return adim.b

    def write(self, d):
        self.yazilan.append(bytes(d))

    def close(self):
        self.kapandi = True

    def fileno(self):
        return 10


class SahteCan:
    """SocketCAN soketi: TX thread'ine `giden` kareleri verir, gelenleri toplar."""

    def __init__(self, giden=(), send_hatasi=None, recv_hatasi=None):
        self.giden = list(giden)
        self.gelen = []
        self.send_hatasi = send_hatasi
        self.recv_hatasi = recv_hatasi
        self.recv_patladi = threading.Event()
        self.kapandi = False

    def hazir_mi(self):
        return bool(self.giden) or self.recv_hatasi is not None

    def recvfrom(self, _n):
        if self.recv_hatasi is not None:
            self.recv_patladi.set()
            raise self.recv_hatasi
        if not self.giden:
            raise BlockingIOError()
        return self.giden.pop(0), None

    def send(self, f):
        if self.send_hatasi is not None:
            raise self.send_hatasi
        self.gelen.append(f)

    def bind(self, _a):
        pass

    def setblocking(self, _b):
        pass

    def close(self):
        self.kapandi = True

    def fileno(self):
        return 11


def sahte_select(r, _w, _x, timeout):
    """Seri port ve CAN soketi icin select. TX thread'i sahte saati
    ilerletmez; yalniz gercek zamanda kisa uyur (bos donmesin diye)."""
    hazir = []
    for o in r:
        if isinstance(o, SahteSeri):
            if o.hazir_mi(timeout):
                hazir.append(o)
        elif isinstance(o, SahteCan):
            if o.hazir_mi():
                hazir.append(o)
            else:
                gercek_time.sleep(0.001)
    return hazir, [], []


@pytest.fixture
def ortam(kopru, monkeypatch):
    saat = SahteSaat()
    monkeypatch.setattr(kopru, "time", saat)
    monkeypatch.setattr(kopru.select, "select", sahte_select)
    return saat


def oturum(kopru, ser, can=None, esik=3.0, istat=None, tx_sleep=0.0):
    return kopru.oturum_calistir(
        ser, can if can is not None else SahteCan(), tx_sleep, bytearray(),
        FMT, EFF, RTR, rx_sessiz_s=esik, istat=istat,
    )


def ext_kare(can_id=0x1E0A0B0C, data=b"\x01\x02\x03\x04\x05\x06\x07\x08"):
    """Waveshare degisken protokolunde extended veri karesi."""
    tip = 0xC0 | 0x20 | (len(data) & 0x0F)
    return bytes([0xAA, tip]) + can_id.to_bytes(4, "little") + data + b"\x55"


def std_kare(can_id=0x123, data=b"\xAA\xBB"):
    tip = 0xC0 | (len(data) & 0x0F)
    return bytes([0xAA, tip]) + can_id.to_bytes(2, "little") + data + b"\x55"


# --------------------------------------------------------------------------- #
#  oturum_calistir: bekci ne zaman tetiklenir / tetiklenmez
# --------------------------------------------------------------------------- #


def test_tam_sessizlik_esikten_hemen_sonra_bitiriyor(kopru, ortam):
    """Hic bayt gelmiyor (select hep zaman asimi) → 3 s'yi gecince 'RX sessiz'."""
    t0 = ortam.t
    ser = SahteSeri(ortam, [Bekle(1000.0)])
    istat = {}
    sebep = oturum(kopru, ser, istat=istat)
    assert sebep is not None and sebep.startswith("RX sessiz"), sebep
    gecen = ortam.t - t0
    assert 3.0 < gecen <= 3.0 + 0.05 + 1e-9, f"tetik ani {gecen:.3f} s"
    assert istat["rx_kare"] == 0


def test_esik_altindaki_bosluklar_tetiklemiyor(kopru, ortam):
    """Here4'un kendiliginden toparlanan donmalari (2.9 s'ye kadar) yeniden
    baglanma tetiklememeli — toplam sure esigi defalarca assa bile."""
    adimlar = []
    for _ in range(10):
        adimlar += [Bekle(2.9), Veri(ext_kare())]
    t0 = ortam.t
    istat = {}
    sebep = oturum(kopru, SahteSeri(ortam, adimlar), istat=istat)
    assert sebep is None, sebep
    assert ortam.t - t0 == pytest.approx(29.0)
    assert istat["rx_kare"] == 10


def test_akis_kesilince_son_kareden_itibaren_sayiyor(kopru, ortam):
    """Sayac son GECERLI kareden itibaren isler, oturum basindan degil."""
    adimlar = [Veri(ext_kare(), dt=0.01) for _ in range(5)]
    adimlar += [Bekle(2.0), Veri(ext_kare()), Bekle(1000.0)]
    istat = {}
    sebep = oturum(kopru, SahteSeri(ortam, adimlar), istat=istat)
    assert sebep.startswith("RX sessiz")
    assert istat["rx_kare"] == 6
    assert "RX 6" in sebep
    # son kare t0+0.05+2.0'da geldi; tetik ondan 3 s sonra (ilk 0.05'lik adimda)
    assert ortam.t - 1000.0 == pytest.approx(0.05 + 2.0 + 3.05, abs=1e-6)


def test_hatasiz_bos_okumalar_canlilik_sayilmiyor(kopru, ortam):
    """select hazir ama read() b"" donuyor (sahte uyanma) → yine sessizlik."""
    adimlar = [Veri(b"", dt=0.01) for _ in range(1000)]
    sebep = oturum(kopru, SahteSeri(ortam, adimlar))
    assert sebep is not None and sebep.startswith("RX sessiz"), sebep


@pytest.mark.parametrize(
    "cop",
    [
        b"\x00\x13\x37\x42",                     # 0xAA hic yok
        b"\xAA",                                 # yalniz baslik
        ext_kare()[:-1],                         # yarim kare, hic tamamlanmiyor
        ext_kare()[:-1] + b"\x00",               # bitis kodu bozuk
    ],
    ids=["baslik_yok", "tek_baslik", "yarim_kare", "bozuk_bitis"],
)
def test_gecersiz_bayt_canlilik_sayilmiyor(kopru, ortam, cop):
    """Adaptor bozuk bayt yagdirsa da bu 'Here4 canli' demek degil."""
    adimlar = [Veri(cop, dt=0.01) for _ in range(1000)]
    istat = {}
    sebep = oturum(kopru, SahteSeri(ortam, adimlar), istat=istat)
    assert sebep is not None and sebep.startswith("RX sessiz"), sebep
    assert istat["rx_kare"] == 0


def test_iki_okumaya_bolunmus_kare_tamamlaninca_sayiliyor(kopru, ortam):
    """Yarim kare sayaci sifirlamaz; tamamlandigi an sifirlar."""
    k = ext_kare()
    adimlar = [Bekle(1.0), Veri(k[:5]), Bekle(1.5), Veri(k[5:]), Bekle(2.9)]
    istat = {}
    sebep = oturum(kopru, SahteSeri(ortam, adimlar), istat=istat)
    assert sebep is None, "kare 2.5. saniyede tamamlandi, sonraki 2.9 s esik alti"
    assert istat["rx_kare"] == 1


def test_yarim_kare_sayaci_sifirlamiyor(kopru, ortam):
    k = ext_kare()
    adimlar = [Bekle(2.5), Veri(k[:5]), Bekle(1000.0)]
    istat = {}
    sebep = oturum(kopru, SahteSeri(ortam, adimlar), istat=istat)
    assert sebep.startswith("RX sessiz")
    assert istat["rx_kare"] == 0
    # 3 s oturum basindan sayildi, yarim kare (2.5 s) sayaci sifirlamadi
    assert ortam.t - 1000.0 == pytest.approx(3.05, abs=1e-6)


def test_tek_okumada_cok_kare_hepsi_sayiliyor_ve_iletiliyor(kopru, ortam):
    can = SahteCan()
    veri = ext_kare(0x10) + std_kare(0x7FF) + ext_kare(0x1FFFFFFF)
    istat = {}
    sebep = oturum(kopru, SahteSeri(ortam, [Veri(veri)]), can=can, istat=istat)
    assert sebep is None
    assert istat["rx_kare"] == 3
    assert len(can.gelen) == 3


def test_kareler_socketcan_e_dogru_iletiliyor(kopru, ortam):
    """Sayac eklemek ayristirmayi bozmamali: ID, bayraklar ve veri birebir."""
    can = SahteCan()
    veri = b"\x13" + ext_kare(0x1E0A0B0C, b"\x01\x02\x03") + std_kare(0x123, b"\xAA\xBB")
    oturum(kopru, SahteSeri(ortam, [Veri(veri)]), can=can)
    assert len(can.gelen) == 2
    cid, dlc, data = struct.unpack(FMT, can.gelen[0])
    assert cid == 0x1E0A0B0C | EFF and dlc == 3 and data[:3] == b"\x01\x02\x03"
    cid, dlc, data = struct.unpack(FMT, can.gelen[1])
    assert cid == 0x123 and dlc == 2 and data[:2] == b"\xAA\xBB"


def test_socketcan_dolu_olsa_da_kare_canlilik_sayiliyor(kopru, ortam):
    """vcan tamponu dolu (send hata) → Here4 yine de konusuyor; yeniden
    baglanmak (adaptoru sifirlamak) yanlis olur."""
    can = SahteCan(send_hatasi=OSError("No buffer space available"))
    adimlar = []
    for _ in range(5):
        adimlar += [Bekle(2.0), Veri(ext_kare())]
    istat = {}
    sebep = oturum(kopru, SahteSeri(ortam, adimlar), can=can, istat=istat)
    assert sebep is None, sebep
    assert istat["rx_kare"] == 5


def test_araya_seri_hata_giren_sessizlik_de_yakalaniyor(kopru, ortam):
    """Tek tuk okuma hatasi (tolerans altinda) + bos okuma: eski kod burada
    sonsuza dek takilirdi. Bekci 3 s'de bitirmeli."""
    adimlar = []
    for _ in range(200):
        adimlar += [Hata(OSError("readiness but no data")), Veri(b"", dt=0.01)]
    sebep = oturum(kopru, SahteSeri(ortam, adimlar))
    assert sebep is not None and sebep.startswith("RX sessiz"), sebep


def test_ardisik_seri_hata_bekciden_once_seri_hatasi_donuyor(kopru, ortam):
    """Gercek USB kopmasi (5 ardisik hata, 0.25 s) bekciden once yakalanir ve
    sebep dogru raporlanir."""
    adimlar = [Hata(OSError("Input/output error")) for _ in range(5)]
    sebep = oturum(kopru, SahteSeri(ortam, adimlar))
    assert sebep.startswith("Seri okuma hatasi"), sebep
    assert "Input/output error" in sebep


@pytest.mark.parametrize("esik", [0.0, -1.0])
def test_esik_sifir_veya_negatif_bekciyi_kapatiyor(kopru, ortam, esik):
    sebep = oturum(kopru, SahteSeri(ortam, [Bekle(600.0)]), esik=esik)
    assert sebep is None, "bekci kapaliyken 10 dk sessizlik oturumu bitirmemeli"


def test_esik_varsayilani_oturum_fonksiyonunda_kapali(kopru, ortam):
    """Eski cagiranlar (rx_sessiz_s vermeyen) davranis degisikligi gormemeli."""
    sebep = kopru.oturum_calistir(
        SahteSeri(ortam, [Bekle(600.0)]), SahteCan(), 0.0, bytearray(), FMT, EFF, RTR
    )
    assert sebep is None


def test_istat_her_oturumda_sifirlaniyor(kopru, ortam):
    istat = {"rx_kare": 99, "tx_kare": 42}
    oturum(kopru, SahteSeri(ortam, [Bekle(1000.0)]), istat=istat)
    assert istat == {"rx_kare": 0, "tx_kare": 0}


def test_sebep_satiri_tx_sayisini_yaziyor(kopru, ortam):
    """Saha teshisi: 'TX akiyor ama RX sessiz' ayrimi log satirindan okunmali."""
    giden = [struct.pack(FMT, 0x10 | EFF, 2, b"\x01\x02".ljust(8, b"\0"))] * 3
    can = SahteCan(giden=giden)

    def tx_bitsin():
        son = gercek_time.monotonic() + 2.0
        while can.giden and gercek_time.monotonic() < son:
            gercek_time.sleep(0.001)
        gercek_time.sleep(0.01)  # son write+sayac artisi

    ser = SahteSeri(ortam, [Bekle(1000.0)], ilk_select=tx_bitsin)
    istat = {}
    sebep = oturum(kopru, ser, can=can, istat=istat)
    assert sebep.startswith("RX sessiz")
    assert istat["tx_kare"] == 3
    assert "TX 3 kare" in sebep and "RX 0" in sebep
    assert len(ser.yazilan) == 3
    assert ser.yazilan[0][:2] == bytes([0xAA, 0xC0 | 0x20 | 2])


def test_tx_cokerse_sebep_tx_hatasi(kopru, ortam):
    """TX thread'i coktugunde sebep RX sessiz degil TX hatasi olmali."""
    can = SahteCan(recv_hatasi=OSError("Network is down"))

    def tx_coksun():
        assert can.recv_patladi.wait(2.0)
        gercek_time.sleep(0.01)

    ser = SahteSeri(ortam, [Bekle(1000.0)], ilk_select=tx_coksun)
    sebep = oturum(kopru, ser, can=can)
    assert sebep.startswith("TX thread hatasi"), sebep
    assert "Network is down" in sebep


def test_kullanici_durdurmasi_none(kopru, ortam):
    sebep = oturum(kopru, SahteSeri(ortam, []))
    assert sebep is None


# --------------------------------------------------------------------------- #
#  main(): dis dongu, esik katlama, arguman
# --------------------------------------------------------------------------- #


class _MainOrtam:
    def __init__(self, kopru, monkeypatch, senaryo, argv=("kopru", "/dev/ttyX", "can0")):
        self.esikler = []
        self.seriler = []
        self.yapilandirma = []
        self.can = SahteCan()
        senaryo = list(senaryo)

        monkeypatch.setattr(kopru.socket, "socket", lambda *a: self.can)
        monkeypatch.setattr(kopru, "time", SahteSaat())

        def seri_ac(port):
            s = SahteSeri(None, [])
            self.seriler.append(s)
            return s

        monkeypatch.setattr(kopru, "seri_ac", seri_ac)
        monkeypatch.setattr(kopru, "configure_adapter",
                            lambda ser, can_baud: self.yapilandirma.append(can_baud))

        def sahte_oturum(*_a, rx_sessiz_s, istat):
            self.esikler.append(rx_sessiz_s)
            kare, sebep = senaryo.pop(0)
            istat["rx_kare"] = kare
            return sebep

        monkeypatch.setattr(kopru, "oturum_calistir", sahte_oturum)
        monkeypatch.setattr(kopru.sys, "argv", list(argv))


SESSIZ = "RX sessiz: 3.1 s gecerli kare yok"


def test_main_ust_uste_sessizlikte_esik_katlaniyor_ve_tavanda_kaliyor(kopru, monkeypatch):
    m = _MainOrtam(kopru, monkeypatch, [(0, SESSIZ)] * 7 + [(0, None)])
    kopru.main()
    assert m.esikler == [3.0, 6.0, 12.0, 24.0, 30.0, 30.0, 30.0, 30.0]
    assert kopru.RX_SESSIZ_VARSAYILAN_S == 3.0 and kopru.RX_SESSIZ_MAX_S == 30.0


def test_main_kare_gelen_oturumdan_sonra_esik_basa_donuyor(kopru, monkeypatch):
    m = _MainOrtam(kopru, monkeypatch,
                   [(0, SESSIZ), (0, SESSIZ), (500, SESSIZ), (0, SESSIZ), (0, None)])
    kopru.main()
    assert m.esikler == [3.0, 6.0, 12.0, 3.0, 6.0]


def test_main_kare_gelen_seri_hatasi_da_esigi_sifirliyor(kopru, monkeypatch):
    """Esik katlanmisken Here4 geri gelip sonra USB koparsa, sonraki oturum
    yine hizli (3 s) bekcile baslamali."""
    m = _MainOrtam(kopru, monkeypatch,
                   [(0, SESSIZ), (0, SESSIZ), (80, "Seri okuma hatasi (5 ardisik): EIO"),
                    (0, None)])
    kopru.main()
    assert m.esikler == [3.0, 6.0, 12.0, 3.0]


def test_main_karesiz_baska_hata_esigi_degistirmiyor(kopru, monkeypatch):
    """Seri/TX hatasi Here4 hakkinda bilgi vermez: esik ne katlanir ne sifirlanir."""
    m = _MainOrtam(kopru, monkeypatch,
                   [(0, SESSIZ), (0, "Seri okuma hatasi (5 ardisik): EIO"),
                    (0, "TX thread hatasi: x"), (0, SESSIZ), (0, None)])
    kopru.main()
    assert m.esikler == [3.0, 6.0, 6.0, 6.0, 12.0]


def test_main_her_oturumda_port_kapatilip_yeniden_aciliyor(kopru, monkeypatch, capsys):
    m = _MainOrtam(kopru, monkeypatch, [(0, SESSIZ), (0, SESSIZ), (0, None)])
    kopru.main()
    assert len(m.seriler) == 3
    assert all(s.kapandi for s in m.seriler), "her oturum sonunda port kapanmali"
    assert m.yapilandirma == [0x01, 0x01, 0x01], "her acilista adaptor yapilandirilmali"
    assert m.can.kapandi
    err = capsys.readouterr().err
    assert "[KOPMA #1] RX sessiz" in err and "[KOPMA #2] RX sessiz" in err
    assert "[KOPMA #3]" not in err, "kullanici durdurmasi kopma sayilmamali"


@pytest.mark.parametrize(
    "arg, beklenen",
    [
        ("0", [0.0, 0.0, 0.0]),
        ("-1", [-1.0, -1.0, -1.0]),
        ("1.5", [1.5, 3.0, 6.0]),
        ("20", [20.0, 30.0, 30.0]),
        ("60", [60.0, 60.0, 60.0]),  # tavan kullanici esiginin altina inmez
    ],
)
def test_main_besinci_arguman_esigi_belirliyor(kopru, monkeypatch, arg, beklenen):
    """0/negatif = kapali: oturum bekciyi calistirmaz, katlama da anlamsiz
    deger uretmez (0*2=0, negatif sabit kalir)."""
    m = _MainOrtam(kopru, monkeypatch, [(0, SESSIZ), (0, SESSIZ), (0, None)],
                   argv=("kopru", "/dev/ttyX", "can0", "0x01", "1.0", arg))
    kopru.main()
    if float(arg) <= 0:
        # kapaliyken "RX sessiz" gelmez ama gelse bile esik pozitife cikmamali
        assert all(e <= 0 for e in m.esikler)
    assert m.esikler == beklenen


def test_main_gercek_oturumla_sessizlikten_kurtariyor(kopru, monkeypatch, capsys):
    """Uctan uca: 1. oturum sessiz → bekci bitirir → port yeniden acilir →
    2. oturumda Here4 konusur, kareler can0'a gider."""
    saat = SahteSaat()
    can = SahteCan()
    monkeypatch.setattr(kopru, "time", saat)
    monkeypatch.setattr(kopru.select, "select", sahte_select)
    monkeypatch.setattr(kopru.socket, "socket", lambda *a: can)
    yapilandirma = []
    monkeypatch.setattr(kopru, "configure_adapter",
                        lambda ser, can_baud: yapilandirma.append(saat.t))
    seriler = [
        SahteSeri(saat, [Bekle(1000.0)]),                       # Here4 sustu
        SahteSeri(saat, [Veri(ext_kare()), Bekle(1.0), Veri(ext_kare())]),
    ]
    acilan = []
    monkeypatch.setattr(kopru, "seri_ac", lambda p: acilan.append(p) or seriler[len(acilan) - 1])
    monkeypatch.setattr(kopru.sys, "argv", ["kopru", "/dev/ttyX", "can0"])

    kopru.main()

    assert acilan == ["/dev/ttyX", "/dev/ttyX"]
    assert len(yapilandirma) == 2
    assert len(can.gelen) == 2, "ikinci oturumda iki kare iletilmeli"
    assert seriler[0].kapandi and seriler[1].kapandi
    err = capsys.readouterr().err
    assert "[KOPMA #1] RX sessiz" in err and "RX 0" in err
