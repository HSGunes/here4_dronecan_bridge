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
    # Testler birbirini kirletmesin: fallback parametreleri ve önbellek sıfırla
    node.set_parameters(
        [
            rclpy.parameter.Parameter("ntrip_fallback_lat", value=0.0),
            rclpy.parameter.Parameter("ntrip_fallback_lon", value=0.0),
        ]
    )
    node._position_cache_path = ""
    node._fallback_position_cached = None
    node._refresh_ntrip_param_cache()


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
    fix.height_ellipsoid_mm = 36000  # N = 36.0 - 25.0 = 11.0 m
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
    lat, lon, alt, geoid_sep, gga_quality = node._last_fix_position
    assert lat == pytest.approx(37.053, abs=1e-6)
    assert lon == pytest.approx(35.3213, abs=1e-6)
    # Jeoit ayrımı N = elipsoit - MSL. GGA alan 11'e bu gider; sabit 0
    # göndermek konumumuzu düşey olarak N kadar yanlış bildirir.
    assert alt == pytest.approx(25.0, abs=1e-3), "GGA yüksekliği MSL olmalı"
    assert geoid_sep == pytest.approx(11.0, abs=1e-3), (
        "N, Fix2'nin iki yükseklik alanından hesaplanmalı"
    )
    assert gga_quality == 4, "RTK FIXED -> NMEA quality 4"


def test_fallback_konum_davranisi(node):
    assert node._get_gga_position() is None, "fallback 0/0 iken konum yok"

    node.set_parameters(
        [
            rclpy.parameter.Parameter("ntrip_fallback_lat", value=37.0),
            rclpy.parameter.Parameter("ntrip_fallback_lon", value=35.3),
        ]
    )
    # B55: NTRIP thread'i rclpy'ye dokunmaz; parametreler spin thread'inde
    # onbellege alinir. Degisikligin gecerli olmasi icin tazeleme sart.
    node._refresh_ntrip_param_cache()
    assert node._get_gga_position() == (37.0, 35.3, 100.0, 0.0, 1)

    node._last_fix_position = (1.0, 2.0, 3.0, 4.0, 5)
    assert node._get_gga_position() == (
        1.0,
        2.0,
        3.0,
        4.0,
        5,
    ), "gerçek fix fallback'i ezer"


# --- RTCM parça bütçesi ---------------------------------------------------- #
# 28.07.2026: Waveshare köprüsünde TX ayrı thread'e alındı, yani frame arası
# bekleme artık seri OKUMAYI durdurmuyor. Eski "RX körlüğü / tty taşması"
# gerekçesi GEÇERSİZ (ölçüm: IMU boşluğu 686 ms -> 20 ms).
#
# Bütçenin bugünkü iki işlevi:
#   1) ALT sınır — bir RTCM epoch'u tek spin turunda boşalabilmeli. 05.08.2026
#      CAN ölçümü: epoch = 155 CAN frame ~= 8-12 parça, gözlenen yayılma 338 ms.
#      Bütçe epoch'tan küçükse epoch birden çok tura yayılır ve her düzeltme
#      gereksiz yere gecikir (saha: gecikme düştükçe FIXED süresi 360->100->60 s).
#   2) ÜST sınır — durak sonrası kuyruk birikirse tek patlama adaptörün TX
#      hattını bir epoch süresinden (1 s) uzun meşgul etmemeli.


def test_parca_butcesi_gelen_hizi_marjla_karsilayabiliyor(node):
    """Alt sınır: pompa, NTRIP'ten gelen parça hızını marjla karşılayabilmeli.

    Epoch'u tek turda boşaltmak CAZİP AMA YANLIŞ bir hedef: bütçe=12 denendi,
    CAN'de yayılma 338 -> 128 ms indi ama 8 dakikada hiç FIXED gelmedi
    (bütçe=4'te ölçülen 1.4 dk'ya karşı). Üst sınırı gecikme değil, Here4'ün
    iç GPS UART tamponu belirliyor — ArduPilot inject_data() yer yoksa veriyi
    sessizce düşürüyor. O yüzden alt sınır sadece "yetişebiliyor mu" olmalı.
    """
    budget = node.get_parameter("rtcm_max_fragments_per_cycle").value
    SPIN_CYCLES_PER_S = 5.0  # ölçülen pompa turu hızı
    DEMAND_FRAGMENTS_PER_S = 9.3  # ölçülen NTRIP parça hızı
    MARGIN = 1.5

    capacity = budget * SPIN_CYCLES_PER_S
    assert capacity >= DEMAND_FRAGMENTS_PER_S * MARGIN, (
        f"bütçe={budget} -> kapasite {capacity:.0f} parça/s, "
        f"ihtiyaç {DEMAND_FRAGMENTS_PER_S} -> marj yetersiz, kuyruk birikir"
    )


def test_parca_butcesi_tx_hattini_bir_epochtan_uzun_mesgul_etmiyor(node):
    """Durak sonrası kuyruk birikirse tek patlama TX'i kilitlememeli."""
    budget = node.get_parameter("rtcm_max_fragments_per_cycle").value
    CAN_FRAMES_PER_FRAGMENT = 19  # ölçüldü: 128 B RTCMStream -> 19 CAN frame
    WAVESHARE_TX_SLEEP_S = 0.001  # waveshare_socketcan_bridge.py tx_sleep default
    EPOCH_PERIOD_S = 1.0  # RTCM epoch'ları 1 Hz

    burst_s = budget * CAN_FRAMES_PER_FRAGMENT * WAVESHARE_TX_SLEEP_S
    assert burst_s <= EPOCH_PERIOD_S, (
        f"bütçe={budget} -> tek patlama TX'i {burst_s * 1000:.0f} ms meşgul eder; "
        "bir epoch periyodundan (1000 ms) uzun olmamalı"
    )


@pytest.mark.parametrize(
    "msg_type,gonderilmeli",
    [
        (1075, True),  # GPS MSM5 — asıl iş
        (1085, True),  # GLONASS MSM5
        (1095, True),  # Galileo MSM5
        (1125, True),  # BeiDou MSM5
        (1005, True),  # referans istasyonu
        (1230, True),  # GLONASS kod-faz bias
        (4094, False),  # Trimble özel — akışın %7.2'si
        (1030, False),  # ağ artığı
        (1031, False),
        (1032, False),
        (1303, False),
        (1304, False),
    ],
)
def test_f9p_cozemedigi_mesajlar_cana_konmuyor(node, msg_type, gonderilmeli):
    node._on_rtcm_frame(make_rtcm_frame(msg_type, 60))
    fragments = kuyrugu_bosalt(node)

    if gonderilmeli:
        assert fragments, f"{msg_type} F9P giriş listesinde, gönderilmeliydi"
    else:
        assert not fragments, f"{msg_type} F9P tarafından atılıyor, CAN'e konmamalı"
        assert node._rtcm_filtered_frames > 0


def test_filtre_kapatilabiliyor(node):
    """Farklı bir alıcı kullanılırsa filtre devre dışı bırakılabilmeli."""
    node.set_parameters(
        [rclpy.parameter.Parameter("rtcm_filter_unsupported", value=False)]
    )
    node._refresh_ntrip_param_cache()
    try:
        node._on_rtcm_frame(make_rtcm_frame(4094, 60))
        assert kuyrugu_bosalt(node), "filtre kapalıyken her mesaj geçmeli"
    finally:
        node.set_parameters(
            [rclpy.parameter.Parameter("rtcm_filter_unsupported", value=True)]
        )
        node._refresh_ntrip_param_cache()


@pytest.mark.parametrize(
    "mode,sub_mode,beklenen_quality",
    [
        (2, 1, 4),  # RTK FIXED
        (2, 0, 5),  # RTK FLOAT
        (1, 0, 2),  # DGPS
        (0, 0, 1),  # SINGLE
    ],
)
def test_gga_kalitesi_cozum_kalitesini_yansitiyor(node, mode, sub_mode, beklenen_quality):
    """GGA alan 6, caster oturum log'larında görünür; hep 1 göndermek bizi
    "standalone" gösteriyordu (TKGM destek görüşmesinde kafa karıştırır)."""
    node._handle_gnss_fix2(_FakeEvent(make_fix2(3, mode, sub_mode)))
    assert node._last_fix_position[4] == beklenen_quality


# --- Konum önbelleği (soğuk açılış hızlandırma) ---------------------------- #
# Soğuk açılışta zincir SERİ bekliyor: alıcı fix alsın -> GGA -> VRS üretsin
# -> RTCM aksın. Son bilinen konumu diskten okuyup GGA'yı hemen göndermek bu
# beklemeyi kesiyor. Gerçek fix gelince hemen ezilir.


def test_konum_onbellegi_diske_yazilip_okunuyor(node, tmp_path):
    yol = tmp_path / "son_konum.json"
    node._position_cache_path = str(yol)

    node._last_fix_position = (37.0473, 35.3645, 104.6, 11.0, 4)
    node._save_cached_position()
    assert yol.exists(), "kayıt dosyası oluşmalı"

    okunan = node._load_cached_position()
    assert okunan is not None
    assert okunan[0] == pytest.approx(37.0473)
    assert okunan[1] == pytest.approx(35.3645)
    assert okunan[2] == pytest.approx(104.6)
    assert okunan[4] == 1, "kalite bilinmiyor -> tek nokta bildirilmeli"


def test_konum_onbellegi_bozuk_dosyada_cokmuyor(node, tmp_path):
    yol = tmp_path / "bozuk.json"
    yol.write_text("{bu gecerli json degil")
    node._position_cache_path = str(yol)
    assert node._load_cached_position() is None


def test_konum_onbellegi_olmayan_dosyada_cokmuyor(node, tmp_path):
    node._position_cache_path = str(tmp_path / "hic_yok.json")
    assert node._load_cached_position() is None


def test_konum_onbellegi_kapatilabiliyor(node, tmp_path):
    node._position_cache_path = ""
    node._last_fix_position = (37.0, 35.0, 100.0, 0.0, 4)
    node._save_cached_position()  # sessizce hiçbir şey yapmamalı
    assert node._load_cached_position() is None
    assert not list(tmp_path.iterdir()), "kapalıyken dosya yazılmamalı"


def test_acik_parametre_onbellegi_eziyor(node, tmp_path):
    """ntrip_fallback_lat/lon verilmişse diskteki konum kullanılmaz."""
    yol = tmp_path / "son_konum.json"
    yol.write_text('{"lat": 1.0, "lon": 2.0, "alt": 3.0}')
    node._position_cache_path = str(yol)
    node._fallback_position_cached = None
    node.set_parameters(
        [
            rclpy.parameter.Parameter("ntrip_fallback_lat", value=41.0),
            rclpy.parameter.Parameter("ntrip_fallback_lon", value=29.0),
        ]
    )
    node._refresh_ntrip_param_cache()
    assert node._get_gga_position() == (41.0, 29.0, 100.0, 0.0, 1)


def test_gercek_fix_onbellegi_eziyor(node, tmp_path):
    yol = tmp_path / "son_konum.json"
    yol.write_text('{"lat": 1.0, "lon": 2.0, "alt": 3.0}')
    node._position_cache_path = str(yol)
    node._fallback_position_cached = None
    node._refresh_ntrip_param_cache()
    assert node._get_gga_position()[:2] == (1.0, 2.0), "önce önbellek kullanılır"

    node._last_fix_position = (37.5, 35.5, 100.0, 0.0, 4)
    assert node._get_gga_position()[:2] == (37.5, 35.5), "gerçek fix ezer"
