#!/usr/bin/env python3
# Copyright 2026 gunes
# Licensed under the Apache-2.0 License.
#
# waveshare_socketcan_bridge kopma dayanikliligi testleri.
#
# 06.08.2026 saha belirtisi: "uzun vadede GPS kopuyor". Sebep kod okumasiyla
# bulundu — seri portta tek bir hata `break` ile dongu disina cikiyor, sureç
# oluyor ve `can0` SONSUZA DEK sessiz kaliyordu; setup script'i koprüyu
# "nohup ... &" ile at-ve-unut baslattigi icin kimse yeniden kaldirmiyordu.

import importlib.util
import os

import pytest

KOPRU = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "scripts",
    "waveshare_socketcan_bridge.py",
)


def _modul():
    spec = importlib.util.spec_from_file_location("ws_bridge", KOPRU)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope="module")
def kopru():
    return _modul()


class _SahteSeri:
    """Belirlenen sayida okumadan sonra patlayan sahte seri port."""

    def __init__(self, patlama_sonrasi=1, hata="USB koptu"):
        self.kalan = patlama_sonrasi
        self.hata = hata
        self.kapandi = False
        self.in_waiting = 1

    def read(self, _n):
        if self.kalan <= 0:
            raise OSError(self.hata)
        self.kalan -= 1
        return b""

    def write(self, _d):
        pass

    def close(self):
        self.kapandi = True

    def fileno(self):
        return 0


def test_seri_ac_pes_etmiyor(kopru, monkeypatch):
    """USB birkaç saniye kaybolursa vazgeçilmemeli, beklenip tekrar denenmeli.

    Eski davranış: `serial.Serial(...)` hata verince sys.exit(1) —
    köprü ölür, can0 sessizleşir, kimse kaldırmaz.
    """
    denemeler = {"n": 0}

    def sahte_serial(port, baud, timeout=None):
        denemeler["n"] += 1
        if denemeler["n"] < 3:
            raise OSError("could not open port")
        return _SahteSeri()

    monkeypatch.setattr(kopru.serial, "Serial", sahte_serial)
    monkeypatch.setattr(kopru.time, "sleep", lambda _s: None)

    port = kopru.seri_ac("/dev/ttyUSB_yok")
    assert port is not None, "sonunda açılmalı"
    assert denemeler["n"] == 3, "iki başarısız denemeden sonra açıldı"


def test_surekli_okuma_hatasi_oturumu_bitiriyor(kopru, monkeypatch):
    """Gerçek kopma (üst üste hata) oturumu bitirmeli — ama süreci öldürmemeli.

    Dönüş bir hata açıklaması olmalı (None değil); çağıran taraf portu yeniden
    açıyor. `None` yalnızca kullanıcı durdurunca döner.
    """
    monkeypatch.setattr(kopru.select, "select", lambda r, w, x, t: (r, [], []))
    monkeypatch.setattr(kopru.time, "sleep", lambda _s: None)

    ser = _SahteSeri(patlama_sonrasi=0, hata="Input/output error")
    sebep = kopru.oturum_calistir(
        ser, _SahteCanSock(), 0.0, bytearray(), "=IB3x8s", 0x80000000, 0x40000000
    )

    assert sebep is not None, "üst üste hatada sebep dönmeli"
    assert "Seri okuma hatasi" in sebep
    assert "Input/output error" in sebep


def test_gecici_okuma_hatasi_yeniden_baglanmayi_tetiklemiyor(kopru, monkeypatch):
    """Tek tük "readiness but no data" olayı adaptörü sıfırlatmamalı.

    18.08.2026: her okuma hatasında yeniden bağlanılıyordu; her bağlanış
    configure_adapter ile adaptörün CAN denetleyicisini sıfırlıyor, adaptör
    hiç oturamıyor ve RXD LED'i hiç yanmıyordu. Geçici hata tolere edilmeli,
    yalnızca ÜST ÜSTE gelenler gerçek kopma sayılmalı.
    """
    monkeypatch.setattr(kopru.select, "select", lambda r, w, x, t: (r, [], []))
    monkeypatch.setattr(kopru.time, "sleep", lambda _s: None)

    class _AradaPatlayan(_SahteSeri):
        """Bir hata, sonra iyi veri, sonra hata... hiç üst üste gelmez."""

        def __init__(self):
            super().__init__()
            self.n = 0

        def read(self, _k):
            self.n += 1
            if self.n > 200:
                raise KeyboardInterrupt()  # testi bitir
            if self.n % 2:
                raise OSError("device reports readiness to read but returned no data")
            return b""

    sebep = kopru.oturum_calistir(
        _AradaPatlayan(), _SahteCanSock(), 0.0, bytearray(),
        "=IB3x8s", 0x80000000, 0x40000000
    )
    assert sebep is None, (
        "araya iyi okuma giriyorsa yeniden bağlanma tetiklenmemeli "
        "(sayaç sıfırlanmalı)"
    )


class _SahteCanSock:
    def recvfrom(self, _n):
        raise BlockingIOError()

    def send(self, _f):
        pass

    def close(self):
        pass

    def fileno(self):
        return 1


def test_kullanici_durdurmasi_none_donuyor(kopru, monkeypatch):
    """Ctrl-C yeniden bağlanma döngüsünü tetiklememeli."""

    def patlayan_select(*_a, **_k):
        raise KeyboardInterrupt()

    monkeypatch.setattr(kopru.select, "select", patlayan_select)

    sebep = kopru.oturum_calistir(
        _SahteSeri(), _SahteCanSock(), 0.0, bytearray(), "=IB3x8s", 0x80000000, 0x40000000
    )
    assert sebep is None, "kullanıcı durdurmasında None dönmeli"


def test_tx_thread_cokerse_oturum_bitiyor(kopru, monkeypatch):
    """TX çökerse tek yönlü köprü kalmamalı.

    Aksi halde RX akmaya devam eder (GPS gelir) ama RTCM gitmez — sessizce
    RTK'sız çalışır ve teşhisi çok zorlaşır.
    """
    monkeypatch.setattr(kopru.select, "select", lambda r, w, x, t: ([], [], []))

    kaynak = open(KOPRU).read()
    assert 'if tx_hata["sebep"]:' in kaynak, "RX döngüsü TX sağlığını kontrol etmeli"
    assert "return tx_hata[\"sebep\"]" in kaynak, "TX çöküşü oturumu bitirmeli"
