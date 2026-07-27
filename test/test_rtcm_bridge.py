#!/usr/bin/env python3
# Copyright 2026 gunes
# Licensed under the Apache-2.0 License.
#
# NTRIP -> RTCMStream köprüsü testleri. CAN bus GEREKMEZ: node can0'ı
# açamayınca DroneCAN thread'i hata verip çıkar, kuyruk/parametre mantığı
# etkilenmez.

import queue

import dronecan
import pytest
import rclpy
from dronecan.transport import Transfer
from sensor_msgs.msg import NavSatStatus

from here4_dronecan_bridge.here4_bridge_node import (
    RTCM_FRAGMENT_SIZE,
    Here4BridgeNode,
)
from here4_dronecan_bridge.ntrip_client import crc24q


def make_rtcm_frame(msg_type, payload_len):
    payload = bytes([msg_type >> 4, (msg_type & 0x0F) << 4]) + bytes(
        (payload_len - 2) * [0x5A]
    )
    body = bytes([0xD3, (len(payload) >> 8) & 0x03, len(payload) & 0xFF]) + payload
    return body + crc24q(body).to_bytes(3, "big")


@pytest.fixture(scope="module")
def node():
    rclpy.init()
    bridge = Here4BridgeNode()
    yield bridge
    bridge.destroy_node()
    rclpy.shutdown()


@pytest.fixture(autouse=True)
def temiz_kuyruk(node):
    while True:
        try:
            node._rtcm_queue.get_nowait()
        except queue.Empty:
            break
    node._rtcm_dropped_fragments = 0
    node._last_fix_position = None


def kuyrugu_bosalt(node):
    fragments = []
    while True:
        try:
            fragments.append(node._rtcm_queue.get_nowait())
        except queue.Empty:
            return fragments


# --- Parçalama ------------------------------------------------------------ #


@pytest.mark.parametrize("payload_len", [2, 60, 126, 200, 400, 1023])
def test_cerceve_parcalari_kayipsiz_ve_sinirda(node, payload_len):
    frame = make_rtcm_frame(1077, payload_len)
    node._on_rtcm_frame(frame)
    fragments = kuyrugu_bosalt(node)

    assert b"".join(fragments) == frame, "parçalar birleşince orijinali vermeli"
    assert all(len(f) <= RTCM_FRAGMENT_SIZE for f in fragments)
    assert len(fragments) == -(-len(frame) // RTCM_FRAGMENT_SIZE)


def test_kuyruk_tasinca_en_eski_atilir(node):
    """Bayat RTCM işe yaramaz: kuyruk sınırda kalmalı, sayaç artmalı."""
    big = make_rtcm_frame(1077, 1023)
    for _ in range(100):
        node._on_rtcm_frame(big)

    assert node._rtcm_queue.qsize() <= node._rtcm_queue.maxsize
    assert node._rtcm_dropped_fragments > 0


# --- DroneCAN tel formatı -------------------------------------------------- #


@pytest.mark.parametrize("size", [1, 64, 127, 128])
def test_rtcmstream_can_frame_turu(size):
    """Parça -> RTCMStream -> CAN frame -> geri çöz: bayt bayt aynı olmalı."""
    msg = dronecan.uavcan.equipment.gnss.RTCMStream()
    msg.protocol_id = msg.PROTOCOL_ID_RTCM3
    payload = bytes(range(256))[:size]
    msg.data = payload

    frames = Transfer(payload=msg, source_node_id=10, transfer_id=3).to_frames()
    decoded = Transfer()
    decoded.from_frames(frames)

    assert decoded.payload._type.full_name == "uavcan.equipment.gnss.RTCMStream"
    assert decoded.payload.protocol_id == 3
    assert bytes(bytearray(decoded.payload.data)) == payload


def test_rtcmstream_129_bayt_kabul_etmez():
    """RTCM_FRAGMENT_SIZE'ın 128 olmasının sebebi: DSDL sınırı uint8[<=128]."""
    msg = dronecan.uavcan.equipment.gnss.RTCMStream()
    with pytest.raises(Exception):
        msg.data = b"\x00" * (RTCM_FRAGMENT_SIZE + 1)


# --- Fix2 mode/sub_mode -> NavSatStatus ------------------------------------ #


class _FakeEvent:
    def __init__(self, message):
        self.message = message


def make_fix2(status, mode, sub_mode):
    fix = dronecan.uavcan.equipment.gnss.Fix2()
    fix.status = status
    fix.mode = mode
    fix.sub_mode = sub_mode
    fix.latitude_deg_1e8 = int(37.053 * 1e8)
    fix.longitude_deg_1e8 = int(35.3213 * 1e8)
    fix.height_msl_mm = 25000
    return fix


@pytest.mark.parametrize(
    "status,mode,sub_mode,beklenen",
    [
        (0, 0, 0, NavSatStatus.STATUS_NO_FIX),
        (3, 0, 0, NavSatStatus.STATUS_FIX),  # SINGLE
        (3, 1, 0, NavSatStatus.STATUS_GBAS_FIX),  # DGPS, yer tabanlı
        (3, 1, 1, NavSatStatus.STATUS_SBAS_FIX),  # DGPS, SBAS kaynaklı
        (3, 2, 0, NavSatStatus.STATUS_GBAS_FIX),  # RTK FLOAT
        (3, 2, 1, NavSatStatus.STATUS_GBAS_FIX),  # RTK FIXED
        (3, 3, 0, NavSatStatus.STATUS_GBAS_FIX),  # PPP
    ],
)
def test_fix_durum_eslemesi(node, status, mode, sub_mode, beklenen):
    """RTK bilgisi Fix2.status'ta değil mode/sub_mode'da — eşleme doğru mu."""
    yayinlanan = []
    orijinal = node._pub_gps.publish
    node._pub_gps.publish = yayinlanan.append
    try:
        node._handle_gnss_fix2(_FakeEvent(make_fix2(status, mode, sub_mode)))
    finally:
        node._pub_gps.publish = orijinal

    assert yayinlanan[0].status.status == beklenen


@pytest.mark.parametrize(
    "mode,sub_mode,beklenen_uere",
    [
        (2, 1, 0.02),  # RTK FIXED
        (2, 0, 0.30),  # RTK FLOAT
        (1, 0, 1.00),  # DGPS
        (0, 0, 0.50),  # SINGLE -> uere varsayılanı
    ],
)
def test_uere_cozum_kalitesine_gore_secilir(node, mode, sub_mode, beklenen_uere):
    """Sabit uere ile RTK kilidi düşünce EKF yanlış GPS'e güvenirdi."""
    node._last_fix_mode = mode
    node._last_fix_sub_mode = sub_mode
    assert node._select_uere() == pytest.approx(beklenen_uere)


# --- GGA konum kaynağı ----------------------------------------------------- #


def test_gga_konumu_sadece_gecerli_fixte_guncellenir(node):
    node._handle_gnss_fix2(_FakeEvent(make_fix2(0, 0, 0)))
    assert node._last_fix_position is None

    node._handle_gnss_fix2(_FakeEvent(make_fix2(3, 2, 1)))
    assert node._last_fix_position is not None
    lat, lon, _alt = node._last_fix_position
    assert lat == pytest.approx(37.053, abs=1e-6)
    assert lon == pytest.approx(35.3213, abs=1e-6)


def test_fallback_konum_davranisi(node):
    assert node._get_gga_position() is None, "fallback 0/0 iken konum yok"

    node.set_parameters(
        [
            rclpy.parameter.Parameter("ntrip_fallback_lat", value=37.0),
            rclpy.parameter.Parameter("ntrip_fallback_lon", value=35.3),
        ]
    )
    assert node._get_gga_position() == (37.0, 35.3, 100.0)

    node._last_fix_position = (1.0, 2.0, 3.0)
    assert node._get_gga_position() == (1.0, 2.0, 3.0), "gerçek fix fallback'i ezer"
