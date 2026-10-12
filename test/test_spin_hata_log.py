#!/usr/bin/env python3
# Copyright 2026 gunes
# Licensed under the Apache-2.0 License.
#
# here4_bridge_node._spin_dongusu: DroneCAN transport istisnalari node'u
# oldurmemeli AMA sessizce yutulmamali (robotaksi_ws#173). Eskiden
# `except Exception: pass` vardi; Here4 susmalarinin nedeni hicbir loga
# dusmuyordu.

import pytest
import rclpy

from here4_dronecan_bridge.here4_bridge_node import Here4BridgeNode


class _Logger:
    def __init__(self):
        self.uyarilar = []

    def warning(self, msg, **kw):
        self.uyarilar.append((msg, kw))

    def info(self, *_a, **_k):
        pass

    error = info
    debug = info


class _DroneCan:
    """spin() senaryoyu oynatir: istisna ornegi atilir, None normal doner.
    Senaryo bitince kopru durdurulur (_running = False)."""

    def __init__(self, kopru, senaryo):
        self.kopru = kopru
        self.senaryo = list(senaryo)
        self.cagri = 0

    def spin(self, timeout):
        self.cagri += 1
        if not self.senaryo:
            self.kopru._running = False
            return
        adim = self.senaryo.pop(0)
        if adim is not None:
            raise adim


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def _kopru(senaryo):
    # __init__ gercek DroneCAN thread'i baslatir; burada yalniz dongu lazim.
    k = Here4BridgeNode.__new__(Here4BridgeNode)
    k._running = True
    k._ntrip_client = None
    k._last_position_save = float("inf")  # konum kaydi tetiklenmesin
    k.pump = 0
    k.led = 0
    k._pump_rtcm = lambda: setattr(k, "pump", k.pump + 1)
    k._broadcast_led_command = lambda: setattr(k, "led", k.led + 1)
    k.log = _Logger()
    k.get_logger = lambda: k.log
    k._dronecan_node = _DroneCan(k, senaryo)
    return k


def test_istisna_loglaniyor_ve_dongu_suruyor():
    hatalar = [ValueError("toggle bit mismatch"), OSError("Network is down"), KeyError("x")]
    k = _kopru(hatalar)
    k._spin_dongusu()

    assert k._dronecan_node.cagri == 4, "her istisnadan sonra spin surmeli"
    assert k._spin_hata_sayisi == 3
    assert len(k.log.uyarilar) == 3, "throttle rclpy'de; burada her cagri gorulur"
    mesajlar = [m for m, _ in k.log.uyarilar]
    assert "ValueError: toggle bit mismatch" in mesajlar[0] and "toplam 1" in mesajlar[0]
    assert "OSError: Network is down" in mesajlar[1] and "toplam 2" in mesajlar[1]
    assert "KeyError" in mesajlar[2] and "toplam 3" in mesajlar[2]
    assert all(kw == {"throttle_duration_sec": 5.0} for _, kw in k.log.uyarilar)


def test_hatasiz_spinde_uyari_yok_ve_rtcm_pompalaniyor():
    k = _kopru([None, None, None])
    k._spin_dongusu()
    assert k.log.uyarilar == []
    assert k._spin_hata_sayisi == 0
    assert k.pump == 4, "RTCM pompasi her turda calismali (3 + durduran tur)"
    assert k.led == 1, "LED saniyede bir (sahte saat yok, ayni saniye)"


def test_araya_giren_hata_sonraki_turlari_bozmuyor():
    k = _kopru([None, RuntimeError("frame drop"), None])
    k._spin_dongusu()
    assert k._spin_hata_sayisi == 1
    assert k.pump == 3, "hatali turda pompa atlanir (4 tur - 1), sonrakilerde surer"


def test_sayac_her_dongu_basinda_sifirlaniyor():
    k = _kopru([ValueError("a")])
    k._spin_dongusu()
    assert k._spin_hata_sayisi == 1
    k._running = True
    k._dronecan_node = _DroneCan(k, [])
    k._spin_dongusu()
    assert k._spin_hata_sayisi == 0


def test_gercek_rclpy_logger_throttle_parametresini_kabul_ediyor():
    """Humble rclpy'de throttle_duration_sec gecerli; ayni cagri yerinden
    ikinci uyari 5 s icinde bastirilmali (rcutils_logger CallerId ile)."""
    logger = rclpy.logging.get_logger("spin_hata_test")
    sonuc = [logger.warning(f"deneme {i}", throttle_duration_sec=5.0) for i in range(2)]
    assert sonuc == [True, False]
