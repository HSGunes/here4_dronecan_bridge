#!/usr/bin/env python3
# Copyright 2026 gunes
# Licensed under the Apache-2.0 License.
#
# Here 4 DroneCAN Bridge Node
# Reads GPS, IMU, and Magnetometer data from a Here 4 sensor via DroneCAN (SocketCAN)
# and publishes them as standard ROS 2 messages.
# Acts as a DroneCAN Node ID allocator so Here 4 can join without a Pixhawk.

import json
import os
import queue
import threading
import logging

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import (
    NavSatFix,
    NavSatStatus,
    Imu,
    MagneticField,
    FluidPressure,
    Temperature,
)
from std_msgs.msg import Header

import dronecan
from dronecan.app import node_monitor as _node_monitor
from dronecan.app import dynamic_node_id as _dynamic_node_id

from .ntrip_client import (
    DEFAULT_HOST as NTRIP_DEFAULT_HOST,
    DEFAULT_MOUNTPOINT as NTRIP_DEFAULT_MOUNTPOINT,
    DEFAULT_PORT as NTRIP_DEFAULT_PORT,
    NtripClient,
)

# uavcan.equipment.gnss.RTCMStream.data alanı uint8[<=128]; kütüphane 129'da
# AttributeError fırlatıyor, bu yüzden parçalamayı biz yapıyoruz.
RTCM_FRAGMENT_SIZE = 128

# NEO-F9P'nin gezici olarak çözebildiği RTCM 3.3 giriş mesajları
# (u-blox NEO-F9P Entegrasyon Kılavuzu, "List of supported RTCM input messages").
# Bu listede OLMAYAN her mesajı alıcı sessizce çöpe atıyor; CAN'e koymanın
# tek etkisi Waveshare adaptörünün 2 ms/frame TX bütçesini yemek oluyor.
# 27.07.2026 ölçümü: TUSAGA akışının %10.2'si bu kategoride (%7.2'si tek
# başına Trimble'a özel 4094).
F9P_RTCM_INPUT_TYPES = frozenset(
    {
        1005,
        1006,
        1007,
        1033,  # referans istasyonu ve anten tanımı
        1074,
        1075,
        1077,  # GPS MSM4/5/7
        1084,
        1085,
        1087,  # GLONASS MSM4/5/7
        1094,
        1095,
        1097,  # Galileo MSM4/5/7
        1124,
        1125,
        1127,  # BeiDou MSM4/5/7
        1230,  # GLONASS kod-faz bias'ı
    }
)

# Suppress noisy dronecan/uavcan library logs to WARNING level
logging.getLogger("uavcan").setLevel(logging.WARNING)
logging.getLogger("dronecan").setLevel(logging.WARNING)

# Monkey-patch: NodeMonitor._update_from_info crashes when GetNodeInfo
# response lacks a 'status' field (common with Here 4 firmware).
_orig_update_from_info = _node_monitor.NodeMonitor.Entry._update_from_info


def _patched_update_from_info(self, e):
    try:
        _orig_update_from_info(self, e)
    except AttributeError:
        pass  # Gracefully skip missing fields


_node_monitor.NodeMonitor.Entry._update_from_info = _patched_update_from_info


class Here4BridgeNode(Node):
    """ROS 2 node that bridges Here 4 DroneCAN sensor data to standard ROS 2 topics."""

    def __init__(self):
        super().__init__("here4_bridge_node")

        # --- Declare Parameters ---
        self.declare_parameter("can_interface", "can0")
        self.declare_parameter("node_id", 10)
        self.declare_parameter(
            "uere", 0.5
        )  # User Equivalent Range Error for covariance
        # Frame id'ler lokalizasyon zinciriyle uyumlu olmalı:
        # navsat_params_real.yaml gps_frame_id=gps_link bekler ve
        # localization.launch.py base_link->gps_link static TF yayınlar.
        self.declare_parameter("gps_frame_id", "gps_link")
        self.declare_parameter("imu_frame_id", "imu_link")
        self.declare_parameter("mag_frame_id", "imu_link")
        # Hard/soft-iron kalibrasyonu (µT cinsinden, FLU body frame — yani
        # zaten Gauss->µT ve FRD->FLU çevrimi sonrası uygulanır). Değerler
        # scripts/mag_calibrator.py çıktısıyla birebir uyumlu; default 0/1 =
        # no-op. mag_calibrator.py /imu/mag dinler; bu köprü use_standard_topics
        # default'uyla /imu/mag'e yayınladığında doğrudan çalışır.
        self.declare_parameter("mag_offset_x", 0.0)
        self.declare_parameter("mag_offset_y", 0.0)
        self.declare_parameter("mag_offset_z", 0.0)
        self.declare_parameter("mag_scale_x", 1.0)
        self.declare_parameter("mag_scale_y", 1.0)
        self.declare_parameter("mag_scale_z", 1.0)

        # --- NTRIP / RTK düzeltmesi ---
        # Zincir: caster -> NtripClient thread -> kuyruk -> spin thread ->
        # uavcan.equipment.gnss.RTCMStream -> AP_Periph handle_RTCMStream()
        # -> gps.handle_gps_rtcm_fragment() -> NEO-F9P
        self.declare_parameter("ntrip_enabled", False)
        self.declare_parameter("ntrip_host", NTRIP_DEFAULT_HOST)
        self.declare_parameter("ntrip_port", NTRIP_DEFAULT_PORT)
        self.declare_parameter("ntrip_mountpoint", NTRIP_DEFAULT_MOUNTPOINT)
        # Kimlik bilgisi: parametre BOŞ bırakılırsa TUSAGA_USER/TUSAGA_PASS
        # ortam değişkenlerinden okunur. Şifreyi launch dosyasına YAZMA —
        # bu paket herkese açık bir git deposu.
        self.declare_parameter("ntrip_user", "")
        self.declare_parameter("ntrip_password", "")
        self.declare_parameter("ntrip_gga_period", 5.0)
        # Akis bu kadar saniye susarsa oturumu koparip yeniden baglan (0=kapali).
        # 05.08.2026 olcumu: 24 s'lik durak boyunca AYNI caster'a yapilan 5
        # kimliksiz yoklamanin 5'i de 0.19 s'de basarili oldu -> caster ayakta,
        # susan yalniz bizim oturumumuz. Beklemek yerine kopariyoruz.
        self.declare_parameter("ntrip_stall_reconnect_s", 8.0)
        # Konum gelmeden VRS akıtmaz. Fix yokken (kapalı alan testi) GGA
        # gönderebilmek için: 0.0 = kapalı, gerçek fix gelince otomatik bırakılır.
        self.declare_parameter("ntrip_fallback_lat", 0.0)
        self.declare_parameter("ntrip_fallback_lon", 0.0)
        # Soguk aciliste zincir SERI bekliyor: alici kendi fix'ini alsin ->
        # GGA gonderelim -> VRS uretmeye baslasin -> RTCM aksin. Son bilinen
        # konumu diske yazip acilista fallback olarak kullanirsak VRS daha
        # alici uyduları ararken akmaya baslar. Gercek fix gelince hemen ezer,
        # yani bayat konumun tek maliyeti birkac saniyelik yanlis VRS.
        # Bos string = kalicilastirmayi kapat.
        self.declare_parameter(
            "ntrip_position_cache", "~/.ros/here4_last_position.json"
        )
        # Bir spin turunda gönderilecek azami parça.
        # 28.07.2026: köprüde TX artık AYRI THREAD'de (waveshare bridge), yani
        # 2 ms'lik frame arası bekleme seri OKUMAYI ARTIK DURDURMUYOR — eski
        # "RX körlüğü" gerekçesi geçersiz (ölçüm: IMU boşluğu 686 ms -> 20 ms).
        # Kalan kısıt gecikme: parça kuyrukta beklerse düzeltme bayatlar ve RTK
        # FIXED'e geçiş uzar. Saha ölçümleri:
        #   budget=2: kapasite ~10 parça/s, marj %7,   yaş 0.71 s, FIXED ~6 dk
        #   budget=4: marj %115,                        yaş 0.57 s, FIXED ~1.40 dk
        # 05.08.2026: bütçeyi 12'ye çıkarmayı denedik — epoch yayılması CAN'de
        # 338 -> 128 ms indi AMA 8 dakikada hiç FIXED gelmedi; 4'e geri dönüldü.
        # DÜZELTME (06.08): o gün suçu "GPS UART 38400 baud, txspace taşıyor"
        # teorisine atmıştık — YANLIŞ. AP_GPS.cpp tespit sırasında u-blox'a
        # UBLOX_SET_BINARY_230400 blob'u gönderir; SERIAL3_BAUD=38 yalnız
        # başlangıç değeridir, GPS_AUTO_CONFIG=1 + GPS_DRV_OPTIONS=0 ile alıcı
        # ve port 230400'de (~23 KB/s) çalışır. inject_data()'nın txspace
        # düşürmesi bu hızda pratikte tetiklenmez. bütçe=12 testi, sonradan
        # çoklu-yol sorunlu olduğu kanıtlanan noktada koşmuştu (uydu 22-26,
        # HDOP 3.5'e kadar) — sonucu ortam kirletti, bütçe suçlu olmayabilir.
        # 4, ölçülmüş ve iyi çalışan değer olarak kalıyor; 12'yi ancak açık
        # alanda, FIXED alınan bir noktada A/B ile yeniden denemek anlamlı.
        self.declare_parameter("rtcm_max_fragments_per_cycle", 4)
        # F9P'nin çözemediği RTCM mesajlarını CAN'e hiç koyma. Farklı bir
        # alıcı kullanılırsa false yapılabilir.
        self.declare_parameter("rtcm_filter_unsupported", True)

        # Düzeltme kalitesine göre UERE. Sabit uere:=0.02 ile çalışırken RTK
        # kilidi düşerse (4G kopması, köprü altı) kovaryans 2 cm'de kalır ve
        # EKF metrelerce hatalı GPS'e körü körüne güvenir — bu yüzden dinamik.
        self.declare_parameter("uere_rtk_fixed", 0.02)
        self.declare_parameter("uere_rtk_float", 0.30)
        self.declare_parameter("uere_dgps", 1.00)

        self._can_interface = (
            self.get_parameter("can_interface").get_parameter_value().string_value
        )
        self._node_id = (
            self.get_parameter("node_id").get_parameter_value().integer_value
        )
        self._uere = self.get_parameter("uere").get_parameter_value().double_value
        self._gps_frame_id = (
            self.get_parameter("gps_frame_id").get_parameter_value().string_value
        )
        self._imu_frame_id = (
            self.get_parameter("imu_frame_id").get_parameter_value().string_value
        )
        self._mag_frame_id = (
            self.get_parameter("mag_frame_id").get_parameter_value().string_value
        )
        self._mag_offset_x = (
            self.get_parameter("mag_offset_x").get_parameter_value().double_value
        )
        self._mag_offset_y = (
            self.get_parameter("mag_offset_y").get_parameter_value().double_value
        )
        self._mag_offset_z = (
            self.get_parameter("mag_offset_z").get_parameter_value().double_value
        )
        self._mag_scale_x = (
            self.get_parameter("mag_scale_x").get_parameter_value().double_value
        )
        self._mag_scale_y = (
            self.get_parameter("mag_scale_y").get_parameter_value().double_value
        )
        self._mag_scale_z = (
            self.get_parameter("mag_scale_z").get_parameter_value().double_value
        )

        # --- ROS 2 Publishers ---
        self._pub_gps = self.create_publisher(NavSatFix, "/here4/gps/fix", 10)
        self._pub_imu = self.create_publisher(Imu, "/here4/imu/data", 10)
        self._pub_mag = self.create_publisher(MagneticField, "/here4/mag", 10)
        self._pub_baro = self.create_publisher(
            FluidPressure, "/here4/baro/pressure", 10
        )
        self._pub_temp = self.create_publisher(
            Temperature, "/here4/baro/temperature", 10
        )

        # --- DroneCAN Node Setup ---
        # All DroneCAN objects are created in the spin thread to avoid
        # SQLite cross-thread errors from the ID allocator.
        self._dronecan_node = None
        self._dronecan_thread = None
        self._running = False
        self._node_monitor = None
        self._id_allocator = None
        self._last_gps_fix_status = 0  # 0 = No fix, 3 = 3D fix
        self._last_fix_mode = 0  # Fix2.MODE_SINGLE
        self._last_fix_sub_mode = 0
        self._hdop = 0.0
        self._vdop = 0.0

        # --- RTCM boru hattı ---
        # NTRIP thread'i buraya yazar, spin thread'i okur. Kuyruk sınırlı:
        # bayat RTCM işe yaramaz, birikmesindense atılması iyidir.
        self._rtcm_queue = queue.Queue(maxsize=512)
        self._rtcm_sent_fragments = 0
        self._rtcm_dropped_fragments = 0
        self._rtcm_broadcast_errors = 0
        self._rtcm_filtered_frames = 0
        # NTRIP thread'i ROS API'sine HIC dokunmamali (B55): rclpy parametre
        # erisimi executor disindan yapiliyor ve kapanis yarislarinda istisna
        # atabiliyor; o istisna NTRIP thread'ini olduruyordu. Bu iki deger
        # spin thread'inde tazelenip buradan okunur.
        self._filter_unsupported_cached = True
        self._fallback_position_cached = None
        _cache = self.get_parameter("ntrip_position_cache").value
        self._position_cache_path = os.path.expanduser(_cache) if _cache else ""
        self._last_position_save = 0.0
        # GGA için son konum. Tek parça tuple olarak atanır; GIL altında
        # atomik okunur/yazılır, bu yüzden kilide gerek yok.
        self._last_fix_position = None
        self._ntrip_client = None

        # Start everything in the DroneCAN thread
        self._running = True
        self._dronecan_thread = threading.Thread(
            target=self._dronecan_thread_main, daemon=True
        )
        self._dronecan_thread.start()

    def _dronecan_thread_main(self):
        """DroneCAN thread: initialize node, allocator, handlers, then spin."""
        # --- Create DroneCAN node ---
        try:
            self._dronecan_node = dronecan.make_node(
                self._can_interface,
                node_id=self._node_id,
                node_info=dronecan.uavcan.protocol.GetNodeInfo.Response(
                    name="here4_ros2_bridge",
                    software_version=dronecan.uavcan.protocol.SoftwareVersion(
                        major=0, minor=1
                    ),
                    hardware_version=dronecan.uavcan.protocol.HardwareVersion(
                        major=0, minor=0
                    ),
                ),
            )
            self.get_logger().info(
                f"DroneCAN node initialized on {self._can_interface} with node_id={self._node_id}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to initialize DroneCAN node: {e}")
            self.get_logger().error(
                "Make sure the CAN interface is up: "
                "sudo ip link set can0 up type can bitrate 1000000"
            )
            return

        # --- Dynamic Node ID Allocation Server ---
        # Created in the same thread that calls node.spin() to avoid SQLite errors.
        try:
            self._node_monitor = _node_monitor.NodeMonitor(self._dronecan_node)
            self._id_allocator = _dynamic_node_id.CentralizedServer(
                self._dronecan_node,
                self._node_monitor,
            )
            self.get_logger().info(
                "Dynamic Node ID Allocation server started (range 1-125)."
            )
        except Exception as e:
            self.get_logger().warn(
                f"Dynamic Node ID Allocation server failed to start: {e}. "
                "Devices without a static Node ID will not be able to join the bus."
            )

        # --- Subscribe to DroneCAN message types ---
        self._dronecan_node.add_handler(
            dronecan.uavcan.equipment.gnss.Fix2, self._handle_gnss_fix2
        )
        self._dronecan_node.add_handler(
            dronecan.uavcan.equipment.ahrs.RawIMU, self._handle_raw_imu
        )
        self._dronecan_node.add_handler(
            dronecan.uavcan.equipment.ahrs.MagneticFieldStrength,
            self._handle_magnetic_field,
        )
        self._dronecan_node.add_handler(
            dronecan.uavcan.equipment.gnss.Auxiliary,
            self._handle_gnss_auxiliary,
        )
        self._dronecan_node.add_handler(
            dronecan.uavcan.equipment.air_data.StaticPressure,
            self._handle_static_pressure,
        )
        self._dronecan_node.add_handler(
            dronecan.uavcan.equipment.air_data.StaticTemperature,
            self._handle_static_temperature,
        )

        # Log NodeStatus from other nodes (detects when Here 4 joins the bus)
        self._dronecan_node.add_handler(
            dronecan.uavcan.protocol.NodeStatus,
            self._handle_node_status,
        )

        self.get_logger().info("DroneCAN listener thread started.")

        # --- NTRIP istemcisi ---
        # CAN node'u hazır olduktan SONRA başlatılır: erken başlarsa gelen
        # düzeltmeler kuyrukta bayatlar (RTCM'in ömrü saniyeler mertebesinde).
        self._start_ntrip_client()

        # --- Spin loop ---
        import time

        last_led_time = 0.0
        last_rtcm_report = 0.0

        while self._running and rclpy.ok():
            try:
                self._dronecan_node.spin(timeout=0.1)

                # Broadcast LED color every 1 second
                now = time.time()
                if now - last_led_time > 1.0:
                    last_led_time = now
                    self._broadcast_led_command()

                # RTCM parçalarını CAN'e ver (NTRIP thread'i broadcast edemez)
                self._pump_rtcm()

                if self._ntrip_client is not None and now - last_rtcm_report > 10.0:
                    last_rtcm_report = now
                    self._refresh_ntrip_param_cache()
                    self._log_rtcm_status()

                if now - self._last_position_save > 60.0:
                    self._last_position_save = now
                    self._save_cached_position()

            except Exception:
                # Ignore transport errors (like toggle bit errors due to CAN frame drops)
                # Just log as a debug/warn and continue spinning so the node doesn't die.
                pass
        self.get_logger().info("DroneCAN listener thread stopped.")

    # ------------------------------------------------------------------ #
    #  NTRIP -> RTCMStream boru hattı
    # ------------------------------------------------------------------ #

    def _start_ntrip_client(self):
        """ntrip_enabled ise NTRIP istemcisini başlatır."""
        if not self.get_parameter("ntrip_enabled").value:
            self.get_logger().info(
                "NTRIP kapalı (ntrip_enabled:=false) — GPS tek nokta çözümde kalacak."
            )
            return

        user = self.get_parameter("ntrip_user").value or os.environ.get(
            "TUSAGA_USER", ""
        )
        password = self.get_parameter("ntrip_password").value or os.environ.get(
            "TUSAGA_PASS", ""
        )
        if not user or not password:
            self.get_logger().error(
                "NTRIP açık ama kimlik bilgisi yok. ntrip_user/ntrip_password "
                "parametrelerini ver ya da TUSAGA_USER/TUSAGA_PASS ortam "
                "değişkenlerini ayarla. RTK devre dışı."
            )
            return

        self._ntrip_client = NtripClient(
            host=self.get_parameter("ntrip_host").value,
            port=self.get_parameter("ntrip_port").value,
            mountpoint=self.get_parameter("ntrip_mountpoint").value,
            user=user,
            password=password,
            on_rtcm=self._on_rtcm_frame,
            get_position=self._get_gga_position,
            gga_period_s=self.get_parameter("ntrip_gga_period").value,
            stall_reconnect_s=self.get_parameter("ntrip_stall_reconnect_s").value,
            logger=self.get_logger(),
        )
        self._refresh_ntrip_param_cache()
        self._ntrip_client.start()

    def _refresh_ntrip_param_cache(self):
        """NTRIP thread'inin kullandigi parametreleri spin thread'inde tazeler.

        Boylece NTRIP thread'i rclpy'ye hic dokunmaz (B55).
        """
        self._filter_unsupported_cached = bool(
            self.get_parameter("rtcm_filter_unsupported").value
        )
        lat = self.get_parameter("ntrip_fallback_lat").value
        lon = self.get_parameter("ntrip_fallback_lon").value
        if lat != 0.0 or lon != 0.0:
            # Açıkça verilen parametre her zaman önceliklidir.
            self._fallback_position_cached = (lat, lon, 100.0, 0.0, 1)
        elif self._fallback_position_cached is None:
            # Diskteki son bilinen konum (yalnız bir kez okunur).
            self._fallback_position_cached = self._load_cached_position()

    def _load_cached_position(self):
        """Diske yazılmış son bilinen konumu okur (yoksa None).

        Soğuk açılışta GGA'yı beklemeden göndermek için; gerçek fix gelir
        gelmez `_last_fix_position` bunu ezer.
        """
        if not self._position_cache_path:
            return None
        try:
            with open(self._position_cache_path) as handle:
                data = json.load(handle)
            pos = (
                float(data["lat"]),
                float(data["lon"]),
                float(data.get("alt", 100.0)),
                0.0,
                1,  # GGA kalitesi: bilmiyoruz, tek nokta bildir
            )
        except (OSError, ValueError, KeyError, TypeError):
            return None
        self.get_logger().info(
            f"Son bilinen konum yüklendi ({pos[0]:.5f}, {pos[1]:.5f}) — "
            "VRS ilk fix'i beklemeden akmaya başlayabilir."
        )
        return pos

    def _save_cached_position(self):
        """Son fix'i diske yazar (atomik: geçici dosya + rename).

        Araç aniden kapanabildiği için doğrudan yazmak dosyayı bozabilir.
        """
        pos = self._last_fix_position
        if not self._position_cache_path or pos is None:
            return
        tmp = self._position_cache_path + ".tmp"
        try:
            klasor = os.path.dirname(self._position_cache_path)
            if klasor:
                os.makedirs(klasor, exist_ok=True)
            with open(tmp, "w") as handle:
                json.dump({"lat": pos[0], "lon": pos[1], "alt": pos[2]}, handle)
            os.replace(tmp, self._position_cache_path)
        except OSError:
            pass  # Kalıcılaştırma en iyi çabadır; başarısızlığı akışı etkilemez

    def _get_gga_position(self):
        """NTRIP thread'i için GGA konumu; fix yoksa fallback, o da yoksa None."""
        if self._last_fix_position is not None:
            return self._last_fix_position
        return self._fallback_position_cached

    def _on_rtcm_frame(self, frame):
        """NTRIP thread'inden çağrılır — sadece kuyruğa yaz, CAN'e dokunma.

        dronecan node'u thread-safe değil; bu dosyadaki tüm DroneCAN nesneleri
        bilerek spin thread'inde yaratılıyor (bkz. _dronecan_thread_main).
        """
        # Alıcının çözemeyeceği mesajı CAN'e koymanın tek etkisi adaptörün
        # TX bütçesini yemek olur (bkz. F9P_RTCM_INPUT_TYPES).
        if len(frame) >= 5 and self._filter_unsupported_cached:
            msg_type = (frame[3] << 4) | (frame[4] >> 4)
            if msg_type not in F9P_RTCM_INPUT_TYPES:
                self._rtcm_filtered_frames += 1
                return

        for offset in range(0, len(frame), RTCM_FRAGMENT_SIZE):
            fragment = frame[offset : offset + RTCM_FRAGMENT_SIZE]
            try:
                self._rtcm_queue.put_nowait(fragment)
            except queue.Full:
                # Bayat düzeltme işe yaramaz: en eskiyi at, yenisine yer aç.
                try:
                    self._rtcm_queue.get_nowait()
                    self._rtcm_queue.put_nowait(fragment)
                except (queue.Empty, queue.Full):
                    pass
                self._rtcm_dropped_fragments += 1

    def _pump_rtcm(self):
        """Kuyruktaki RTCM parçalarını RTCMStream olarak CAN'e yayınlar."""
        if self._dronecan_node is None:
            return
        budget = self.get_parameter("rtcm_max_fragments_per_cycle").value
        for _ in range(budget):
            try:
                fragment = self._rtcm_queue.get_nowait()
            except queue.Empty:
                return
            try:
                msg = dronecan.uavcan.equipment.gnss.RTCMStream()
                msg.protocol_id = msg.PROTOCOL_ID_RTCM3
                msg.data = fragment
                self._dronecan_node.broadcast(msg)
                self._rtcm_sent_fragments += 1
            except Exception:
                # Spin döngüsündeki genel except bunu yutardı; sayaçla görünür kıl.
                self._rtcm_broadcast_errors += 1

    def _log_rtcm_status(self):
        """RTCM boru hattının durumunu periyodik olarak raporlar."""
        client = self._ntrip_client
        mode_names = {0: "SINGLE", 1: "DGPS", 2: "RTK", 3: "PPP"}
        sub_names = {0: "FLOAT", 1: "FIXED"}
        quality = mode_names.get(self._last_fix_mode, f"?{self._last_fix_mode}")
        if self._last_fix_mode == 2:
            quality += "/" + sub_names.get(self._last_fix_sub_mode, "?")

        self.get_logger().info(
            f"RTCM: bağlı={client.connected} "
            f"cerceve={client.total_frames} bayt={client.total_bytes} "
            f"gonderilen={self._rtcm_sent_fragments} "
            f"filtrelenen={self._rtcm_filtered_frames} "
            f"dusen={self._rtcm_dropped_fragments} "
            f"yayin_hatasi={self._rtcm_broadcast_errors} "
            f"crc_hata={client.crc_errors} yeniden_baglanma={client.reconnects} "
            f"| durak={client.stalls} olu={client.stalled_seconds:.0f}s "
            f"durak_baglanti={client.stall_reconnects} "
            f"beklenmedik={client.unexpected_errors} "
            f"callback_hata={client.callback_errors} "
            f"enuzun={client.longest_stall_s:.0f}s "
            f"| GPS: {quality} uere={self._select_uere():.2f}"
        )
        if (
            client.connected
            and self._rtcm_sent_fragments > 0
            and (self._last_fix_mode == 0)
        ):
            self.get_logger().warn(
                "RTCM akıyor ama GPS hâlâ SINGLE modda. Kontrol et: "
                "(1) GGA konumu doğru bölgede mi, "
                "(2) Here4 mountpoint formatını destekliyor mu (RTCM3 olmalı), "
                "(3) CAN'de RTCMStream düşüyor mu (candump can0 | grep -i 1062)."
            )

    # ------------------------------------------------------------------ #
    #  DroneCAN Callbacks -> ROS 2 Publishers
    # ------------------------------------------------------------------ #

    _discovered_nodes = set()

    def _broadcast_led_command(self):
        """Broadcasts LED colors to stop the rainbow mode. Green for 3D Fix, Blue otherwise."""
        if not self._dronecan_node:
            return

        cmd = dronecan.uavcan.equipment.indication.LightsCommand()
        light = dronecan.uavcan.equipment.indication.SingleLightCommand()
        # light_id = 255 means "all lights" or "default light" on the node
        light.light_id = 255

        color = dronecan.uavcan.equipment.indication.RGB565()
        # LED = düzeltme kalitesi göstergesi. Sahada araç dışarıdayken laptop
        # başında olunmuyor; RTK FIXED'e geçişi anteni görerek anlamak gerekiyor.
        # Sadece 3D fix'e bakmak yetmiyordu: mod SINGLE->DGPS->RTK ilerlerken
        # status hep 3 kaldığı için LED hiç değişmiyordu.
        # RGB565: red/blue 5 bit (0-31), green 6 bit (0-63).
        if self._last_gps_fix_status < 3:
            rgb = (0, 0, 31)  # mavi   — fix yok
        elif self._last_fix_mode == 2 and self._last_fix_sub_mode == 1:
            rgb = (0, 63, 0)  # yeşil  — RTK FIXED (cm)
        elif self._last_fix_mode == 2:
            rgb = (31, 0, 31)  # mor    — RTK FLOAT (dm)
        elif self._last_fix_mode in (1, 3):
            rgb = (0, 63, 31)  # turkuaz— DGPS / PPP (m)
        else:
            rgb = (31, 63, 0)  # sarı   — 3D fix ama düzeltme yok (SINGLE)

        color.red, color.green, color.blue = rgb

        light.color = color
        cmd.commands.append(light)

        try:
            self._dronecan_node.broadcast(cmd)
        except Exception:
            pass

    def _handle_gnss_auxiliary(self, event):
        """Extract DOP values for covariance calculation."""
        self._hdop = event.message.hdop
        self._vdop = event.message.vdop

    def _handle_node_status(self, event):
        """Log when a new DroneCAN node joins the bus."""
        src = event.transfer.source_node_id
        if src != self._node_id and src not in self._discovered_nodes:
            self._discovered_nodes.add(src)
            status = event.message
            self.get_logger().info(
                f"New DroneCAN node joined: ID={src}, "
                f"health={status.health}, mode={status.mode}"
            )

    def _make_header(self, frame_id: str) -> Header:
        """Create a ROS 2 Header with current timestamp and the given frame_id."""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        return header

    def _select_uere(self):
        """Kovaryans için UERE'yi son çözüm kalitesine göre seçer.

        Sabit `uere` ile çalışırken RTK kilidi düşerse kovaryans olduğu gibi
        kalır ve EKF metrelerce hatalı GPS'e körü körüne güvenir. Bu yüzden
        Fix2.mode/sub_mode'a bağlı.
        """
        if self._last_fix_mode == 2:  # RTK
            if self._last_fix_sub_mode == 1:  # FIXED
                return self.get_parameter("uere_rtk_fixed").value
            return self.get_parameter("uere_rtk_float").value
        if self._last_fix_mode == 1:  # DGPS
            return self.get_parameter("uere_dgps").value
        return self.get_parameter("uere").value

    def _handle_gnss_fix2(self, event):
        """Convert uavcan.equipment.gnss.Fix2 -> sensor_msgs/NavSatFix."""
        msg = NavSatFix()
        msg.header = self._make_header(self._gps_frame_id)

        fix2 = event.message

        # --- Position conversion ---
        msg.latitude = fix2.latitude_deg_1e8 / 1e8
        msg.longitude = fix2.longitude_deg_1e8 / 1e8

        # Prefer MSL altitude; fall back to ellipsoid
        if hasattr(fix2, "height_msl_mm") and fix2.height_msl_mm != 0:
            msg.altitude = fix2.height_msl_mm / 1e3
        elif hasattr(fix2, "height_ellipsoid_mm"):
            msg.altitude = fix2.height_ellipsoid_mm / 1e3
        else:
            msg.altitude = 0.0

        # --- Fix status mapping ---
        status = NavSatStatus()
        dronecan_status = getattr(fix2, "status", 0)
        self._last_gps_fix_status = dronecan_status

        # RTK bilgisi `status` alanında DEĞİL: o alan 2 bitlik ve en fazla
        # STATUS_3D_FIX=3 olabiliyor. Düzeltme kalitesi mode/sub_mode'da:
        #   mode: 0=SINGLE 1=DGPS 2=RTK 3=PPP
        #   sub_mode (RTK icin): 0=FLOAT 1=FIXED
        fix_mode = getattr(fix2, "mode", 0)
        fix_sub_mode = getattr(fix2, "sub_mode", 0)
        self._last_fix_mode = fix_mode
        self._last_fix_sub_mode = fix_sub_mode

        if dronecan_status < 2:
            status.status = NavSatStatus.STATUS_NO_FIX
        elif fix_mode == 2 or fix_mode == 3:  # RTK veya PPP
            status.status = NavSatStatus.STATUS_GBAS_FIX
        elif fix_mode == 1:  # DGPS
            # sub_mode 1 = SBAS kaynaklı (uydu tabanlı), diğerleri yer tabanlı
            status.status = (
                NavSatStatus.STATUS_SBAS_FIX
                if fix_sub_mode == 1
                else NavSatStatus.STATUS_GBAS_FIX
            )
        else:
            status.status = NavSatStatus.STATUS_FIX

        status.service = NavSatStatus.SERVICE_GPS
        msg.status = status

        # NTRIP thread'inin GGA'sı için son konumu sakla (tek atama = atomik).
        # Jeoit ayrımı N = elipsoit - MSL; GGA'da alan 11 bunu ister ve caster
        # elipsoit yüksekliğimizi MSL + N olarak hesaplar. Sabit 0 göndermek
        # konumumuzu düşey olarak N kadar yanlış bildirir.
        # DİKKAT (05.08.2026 ölçümü): Here4 AP_Periph firmware'i iki alanı da
        # AYNI değerle dolduruyor (ikisi de 103670 mm), yani N burada 0 çıkıyor
        # ve pratikte eski sabit davranışla aynı. Gerçek jeoit ayrımı gerekirse
        # bir jeoit modeli (EGM96/TG03) gerekir — alıcıdan türetilemiyor.
        if dronecan_status >= 2:
            geoid_sep = 0.0
            msl_mm = getattr(fix2, "height_msl_mm", 0)
            ell_mm = getattr(fix2, "height_ellipsoid_mm", 0)
            if msl_mm and ell_mm:
                geoid_sep = (ell_mm - msl_mm) / 1e3
            # NMEA GGA kalitesi: 1=SPS, 2=DGPS, 4=RTK FIXED, 5=RTK FLOAT.
            # Caster bunu düzeltme üretmek için kullanmaz ama oturum log'una
            # yazar; hep 1 göndermek bizi "standalone" gösteriyordu.
            if fix_mode == 2:
                gga_quality = 4 if fix_sub_mode == 1 else 5
            elif fix_mode == 1:
                gga_quality = 2
            else:
                gga_quality = 1
            self._last_fix_position = (
                msg.latitude,
                msg.longitude,
                msg.altitude,
                geoid_sep,
                gga_quality,
            )

        # --- Covariance calculation ---
        if self._hdop > 0.0 and self._vdop > 0.0:
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            uere = self._select_uere()
            var_h = (self._hdop * uere) ** 2
            var_v = (self._vdop * uere) ** 2
            msg.position_covariance = [
                var_h,
                0.0,
                0.0,
                0.0,
                var_h,
                0.0,
                0.0,
                0.0,
                var_v,
            ]
        else:
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self._pub_gps.publish(msg)

    def _handle_raw_imu(self, event):
        """Convert uavcan.equipment.ahrs.RawIMU -> sensor_msgs/Imu."""
        msg = Imu()
        msg.header = self._make_header(self._imu_frame_id)

        raw = event.message
        dt = float(getattr(raw, "integration_interval", 0.0))

        # --- Angular velocity (rad/s) ---
        # Convert FRD to FLU (Y = -Y, Z = -Z)
        if (
            dt > 0.0
            and hasattr(raw, "rate_gyro_integral")
            and len(raw.rate_gyro_integral) >= 3
        ):
            msg.angular_velocity.x = float(raw.rate_gyro_integral[0]) / dt
            msg.angular_velocity.y = -(float(raw.rate_gyro_integral[1]) / dt)
            msg.angular_velocity.z = -(float(raw.rate_gyro_integral[2]) / dt)
        elif hasattr(raw, "rate_gyro_latest") and len(raw.rate_gyro_latest) >= 3:
            msg.angular_velocity.x = float(raw.rate_gyro_latest[0])
            msg.angular_velocity.y = -float(raw.rate_gyro_latest[1])
            msg.angular_velocity.z = -float(raw.rate_gyro_latest[2])

        # --- Linear acceleration (m/s^2) ---
        # Convert FRD to FLU (Y = -Y, Z = -Z)
        if (
            dt > 0.0
            and hasattr(raw, "accelerometer_integral")
            and len(raw.accelerometer_integral) >= 3
        ):
            msg.linear_acceleration.x = float(raw.accelerometer_integral[0]) / dt
            msg.linear_acceleration.y = -(float(raw.accelerometer_integral[1]) / dt)
            msg.linear_acceleration.z = -(float(raw.accelerometer_integral[2]) / dt)
        elif (
            hasattr(raw, "accelerometer_latest") and len(raw.accelerometer_latest) >= 3
        ):
            msg.linear_acceleration.x = float(raw.accelerometer_latest[0])
            msg.linear_acceleration.y = -float(raw.accelerometer_latest[1])
            msg.linear_acceleration.z = -float(raw.accelerometer_latest[2])

        # --- Orientation: not available in RawIMU ---
        # Set quaternion to identity and mark covariance as unknown (-1 in first element)
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0
        msg.orientation_covariance[0] = -1.0  # Signals orientation data is invalid

        # --- Set Covariances ---
        # Provide base covariances so EKF does not explode (Zero covariance = infinite trust)
        msg.angular_velocity_covariance = [
            1e-5,
            0.0,
            0.0,
            0.0,
            1e-5,
            0.0,
            0.0,
            0.0,
            1e-5,
        ]
        msg.linear_acceleration_covariance = [
            1e-3,
            0.0,
            0.0,
            0.0,
            1e-3,
            0.0,
            0.0,
            0.0,
            1e-3,
        ]

        self._pub_imu.publish(msg)

    def _handle_magnetic_field(self, event):
        """Convert uavcan.equipment.ahrs.MagneticFieldStrength -> sensor_msgs/MagneticField."""
        msg = MagneticField()
        msg.header = self._make_header(self._mag_frame_id)

        mag = event.message

        # Convert from Gauss to µT (1 Gauss = 100 µT), FRD -> FLU (Y=-Y, Z=-Z)
        if hasattr(mag, "magnetic_field_ga") and len(mag.magnetic_field_ga) >= 3:
            raw_x_ut = float(mag.magnetic_field_ga[0]) * 100.0
            raw_y_ut = -(float(mag.magnetic_field_ga[1]) * 100.0)
            raw_z_ut = -(float(mag.magnetic_field_ga[2]) * 100.0)

            # Hard/soft-iron kalibrasyonu (µT, FLU body frame). Offset=0/scale=1
            # iken tam olarak eski davranışla aynı (no-op).
            cal_x_ut = (raw_x_ut - self._mag_offset_x) * self._mag_scale_x
            cal_y_ut = (raw_y_ut - self._mag_offset_y) * self._mag_scale_y
            cal_z_ut = (raw_z_ut - self._mag_offset_z) * self._mag_scale_z

            msg.magnetic_field.x = cal_x_ut * 1e-6  # µT -> Tesla
            msg.magnetic_field.y = cal_y_ut * 1e-6
            msg.magnetic_field.z = cal_z_ut * 1e-6

            # Set covariance for magnetometer (Typical values in Tesla^2)
            msg.magnetic_field_covariance = [
                1e-7,
                0.0,
                0.0,
                0.0,
                1e-7,
                0.0,
                0.0,
                0.0,
                1e-7,
            ]

        self._pub_mag.publish(msg)

    def _handle_static_pressure(self, event):
        """Convert uavcan.equipment.air_data.StaticPressure -> sensor_msgs/FluidPressure."""
        msg = FluidPressure()
        msg.header = self._make_header("here4_baro_link")
        msg.fluid_pressure = event.message.static_pressure
        msg.variance = event.message.static_pressure_variance
        self._pub_baro.publish(msg)

    def _handle_static_temperature(self, event):
        """Convert uavcan.equipment.air_data.StaticTemperature -> sensor_msgs/Temperature."""
        msg = Temperature()
        msg.header = self._make_header("here4_baro_link")
        # DroneCAN temperature is often in Kelvin, so it can be published as is
        # ROS 2 Temperature message doesn't strictly specify unit, but usually Celsius or Kelvin.
        # Let's leave it as raw from DroneCAN (Kelvin) to prevent precision loss.
        msg.temperature = event.message.static_temperature
        msg.variance = event.message.static_temperature_variance
        self._pub_temp.publish(msg)

    def destroy_node(self):
        """Clean up DroneCAN thread and allocator on shutdown."""
        self._running = False
        if self._ntrip_client is not None:
            self._ntrip_client.stop()
            self._ntrip_client = None
        if self._id_allocator is not None:
            try:
                self._id_allocator.close()
            except Exception:
                pass
        if self._dronecan_thread is not None and self._dronecan_thread.is_alive():
            self._dronecan_thread.join(timeout=2.0)
        self.get_logger().info("Here4BridgeNode shutting down.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Here4BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
