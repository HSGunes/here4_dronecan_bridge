#!/usr/bin/env python3
# Copyright 2026 gunes
# Licensed under the Apache-2.0 License.
#
# here4_bridge_node barometre seyreltmesi (robotaksi_ws#375): Here4 basinc ve
# sicakligi ~75'er Hz gonderiyor, canli abonesi yok. Yayin baro_rate_hz'e
# seyrelir; yayinlanan mesajlarin icerigi eski davranisla (baro_rate_hz=0,
# her mesaj) birebir ayni kalmali.

from types import SimpleNamespace

import pytest
import rclpy
from std_msgs.msg import Header

from here4_dronecan_bridge import here4_bridge_node as modul
from here4_dronecan_bridge.here4_bridge_node import Here4BridgeNode

GELIS_HZ = 75.0
SURE_S = 10.0


class _Yayinci:
    def __init__(self):
        self.mesajlar = []

    def publish(self, msg):
        self.mesajlar.append(msg)


class _Saat:
    """time.monotonic yerine; deger elle ilerletilir."""

    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def saat(monkeypatch):
    s = _Saat()
    monkeypatch.setattr(modul.time, "monotonic", s)
    return s


def _kopru(baro_hz):
    # __init__ gercek DroneCAN thread'i baslatir; burada yalniz isleyiciler lazim.
    k = Here4BridgeNode.__new__(Here4BridgeNode)
    k._baro_aralik_s = 1.0 / baro_hz if baro_hz > 0.0 else 0.0
    k._baro_son = {"basinc": None, "sicaklik": None}
    k._pub_baro = _Yayinci()
    k._pub_temp = _Yayinci()
    k._make_header = lambda frame_id: Header(frame_id=frame_id)
    return k


def _basinc(i):
    return SimpleNamespace(
        message=SimpleNamespace(
            static_pressure=101325.0 + i, static_pressure_variance=0.5 + i
        )
    )


def _sicaklik(i):
    return SimpleNamespace(
        message=SimpleNamespace(
            static_temperature=300.0 + 0.01 * i, static_temperature_variance=0.1 + i
        )
    )


def _oynat(kopru, saat):
    """75 Hz'lik basinc + sicaklik akisini oynatir; her iki yayinin mesajlarini dondurur."""
    n = int(GELIS_HZ * SURE_S)
    for i in range(n):
        saat.t = 1000.0 + i / GELIS_HZ
        kopru._handle_static_pressure(_basinc(i))
        kopru._handle_static_temperature(_sicaklik(i))
    return kopru._pub_baro.mesajlar, kopru._pub_temp.mesajlar


def _ozet_basinc(m):
    return (m.header.frame_id, m.fluid_pressure, m.variance)


def _ozet_sicaklik(m):
    return (m.header.frame_id, m.temperature, m.variance)


def test_sifir_hz_eski_davranis_her_mesaj(saat):
    basinc, sicaklik = _oynat(_kopru(0.0), saat)
    n = int(GELIS_HZ * SURE_S)
    assert len(basinc) == n
    assert len(sicaklik) == n


def test_5hz_seyrelir_ve_icerik_oracle_ile_ayni(saat):
    oracle_b, oracle_s = _oynat(_kopru(0.0), saat)
    basinc, sicaklik = _oynat(_kopru(5.0), saat)

    # 10 s'de ~50 yayin; esik yuvarlamasi en fazla bir mesaj oynatir.
    beklenen = int(5.0 * SURE_S)
    assert beklenen - 1 <= len(basinc) <= beklenen + 1
    assert beklenen - 1 <= len(sicaklik) <= beklenen + 1

    # Yayinlanan her mesaj eski davranisin ayni indisli mesajiyla birebir
    # ayni: yalniz seyrelir, icerik degismez. Ilk mesaj hemen yayinlanir.
    indis_b = [round(m.fluid_pressure - 101325.0) for m in basinc]
    indis_s = [round((m.temperature - 300.0) * 100.0) for m in sicaklik]
    assert indis_b[0] == 0
    assert indis_s[0] == 0
    assert [_ozet_basinc(m) for m in basinc] == [_ozet_basinc(oracle_b[i]) for i in indis_b]
    assert [_ozet_sicaklik(m) for m in sicaklik] == [
        _ozet_sicaklik(oracle_s[i]) for i in indis_s
    ]
    # Araliklar 0,2 s'ye (75 Hz'de 15 gelis) yakin, kayma birikmiyor.
    for indisler in (indis_b, indis_s):
        farklar = [b - a for a, b in zip(indisler, indisler[1:])]
        assert all(14 <= f <= 16 for f in farklar), farklar
        assert abs((indisler[-1] - indisler[0]) / (len(indisler) - 1) - 15.0) < 0.1


def test_basinc_ve_sicaklik_birbirinden_bagimsiz(saat):
    k = _kopru(5.0)
    saat.t = 1000.0
    k._handle_static_pressure(_basinc(0))
    # Sicaklik ilk kez geliyor: basincin damgasi onu engellememeli.
    saat.t = 1000.05
    k._handle_static_temperature(_sicaklik(0))
    assert len(k._pub_baro.mesajlar) == 1
    assert len(k._pub_temp.mesajlar) == 1


def test_bosluk_sonrasi_ilk_mesaj_yayinlanir(saat):
    k = _kopru(5.0)
    saat.t = 1000.0
    k._handle_static_pressure(_basinc(0))
    saat.t = 1000.1
    k._handle_static_pressure(_basinc(1))  # aralik dolmadi -> atlanir
    saat.t = 1003.0
    k._handle_static_pressure(_basinc(2))  # uzun bosluk -> yayinlanir
    assert [m.fluid_pressure for m in k._pub_baro.mesajlar] == [101325.0, 101327.0]
