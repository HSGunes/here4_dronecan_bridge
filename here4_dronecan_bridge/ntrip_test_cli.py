#!/usr/bin/env python3
# Copyright 2026 gunes
# Licensed under the Apache-2.0 License.
#
# TUSAGA-Aktif NTRIP bağlantı testi (CAN/Here4 gerektirmez).
#
# Amaç: Here4'e RTCM enjekte etmeden ÖNCE aboneliğin gerçekten akıttığını
# kanıtlamak. Script sadece okur; caster'a giden tek veri periyodik NMEA GGA.
#
# Kullanım:
#   export TUSAGA_USER=... TUSAGA_PASS=...
#   ros2 run here4_dronecan_bridge ntrip_test --lat 37.0 --lon 35.32
#   ros2 run here4_dronecan_bridge ntrip_test --ros-fix   # konumu /here4/gps/fix'ten
#   ros2 run here4_dronecan_bridge ntrip_test --sourcetable  # yayın listesi, çık
#
# ROS ortamı olmadan da çalışır:
#   python3 here4_dronecan_bridge/ntrip_test_cli.py --sourcetable

import argparse
import os
import socket
import sys
import time

# Modül hem kurulu paketten hem de kaynak ağacından import edilebilsin.
try:
    from here4_dronecan_bridge.ntrip_client import (
        DEFAULT_HOST,
        DEFAULT_MOUNTPOINT,
        DEFAULT_PORT,
        ChunkedDecoder,
        Rtcm3Parser,
        build_gga,
        fetch_sourcetable,
        open_ntrip,
    )
except ImportError:
    # ROS ortamı source edilmemişse: paketi içeren dizini (repo kökü) path'e ekle
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
    from here4_dronecan_bridge.ntrip_client import (  # noqa: E402
        DEFAULT_HOST,
        DEFAULT_MOUNTPOINT,
        DEFAULT_PORT,
        ChunkedDecoder,
        Rtcm3Parser,
        build_gga,
        fetch_sourcetable,
        open_ntrip,
    )


def get_fix_from_ros(timeout_s: float = 10.0):
    """/here4/gps/fix'ten tek bir NavSatFix bekler. rclpy yoksa None döner."""
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import NavSatFix
    except ImportError:
        print(
            "[HATA] rclpy/sensor_msgs yok — ROS ortamını source et "
            "veya --lat/--lon ver.",
            file=sys.stderr,
        )
        return None

    rclpy.init()
    node = Node("ntrip_tusaga_test_fix_listener")
    result = {}

    def on_fix(msg):
        if msg.status.status >= 0:  # STATUS_FIX veya daha iyisi
            result["fix"] = (msg.latitude, msg.longitude, msg.altitude)

    node.create_subscription(NavSatFix, "/here4/gps/fix", on_fix, 10)
    deadline = time.monotonic() + timeout_s
    while rclpy.ok() and "fix" not in result and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()

    if "fix" not in result:
        print(
            f"[HATA] {timeout_s:.0f} s içinde /here4/gps/fix'ten geçerli konum gelmedi.",
            file=sys.stderr,
        )
        return None
    return result["fix"]


def run_stream(
    sock, lat, lon, alt, gga_period_s, duration_s, report_period_s, is_chunked=False
):
    parser = Rtcm3Parser()
    decoder = ChunkedDecoder() if is_chunked else None
    total_bytes = 0

    start = time.monotonic()
    last_gga = 0.0
    last_report = start
    last_report_bytes = 0

    sock.settimeout(1.0)
    print(f"[gga] {lat:.6f}, {lon:.6f}, {alt:.1f} m — her {gga_period_s:.0f} s")

    while duration_s <= 0 or time.monotonic() - start < duration_s:
        now = time.monotonic()

        if now - last_gga >= gga_period_s:
            sock.sendall(build_gga(lat, lon, alt))
            last_gga = now

        try:
            chunk = sock.recv(4096)
            if chunk == b"":
                print("[UYARI] Caster bağlantıyı kapattı (akış kesildi).")
                break
        except socket.timeout:
            chunk = b""

        if chunk:
            total_bytes += len(chunk)
            if decoder is not None:
                chunk = decoder.feed(chunk)
                if decoder.finished:
                    print("[UYARI] Caster akışı sonlandırdı (son chunk).")
                    break
            for _frame in parser.feed(chunk):
                pass

        if now - last_report >= report_period_s:
            elapsed = now - last_report
            rate = (total_bytes - last_report_bytes) / elapsed
            top = ", ".join(f"{t}:{c}" for t, c in parser.types.most_common(6))
            print(
                f"[{now - start:6.1f}s] {rate:7.0f} B/s | "
                f"çerçeve={parser.frames:5d} crc_hata={parser.crc_errors:3d} | {top}"
            )
            last_report = now
            last_report_bytes = total_bytes

    return parser, total_bytes, time.monotonic() - start


def main():
    parser_args = argparse.ArgumentParser(
        description="TUSAGA-Aktif NTRIP akış testi (salt okuma)"
    )
    parser_args.add_argument("--host", default=DEFAULT_HOST)
    parser_args.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser_args.add_argument("--mountpoint", default=DEFAULT_MOUNTPOINT)
    parser_args.add_argument("--lat", type=float, help="GGA enlem (derece)")
    parser_args.add_argument("--lon", type=float, help="GGA boylam (derece)")
    parser_args.add_argument(
        "--alt", type=float, default=100.0, help="GGA yükseklik (m)"
    )
    parser_args.add_argument(
        "--ros-fix",
        action="store_true",
        help="Konumu /here4/gps/fix topic'inden al (--lat/--lon yerine)",
    )
    parser_args.add_argument("--gga-period", type=float, default=5.0)
    parser_args.add_argument(
        "--duration", type=float, default=60.0, help="Saniye; 0 = süresiz"
    )
    parser_args.add_argument("--report-period", type=float, default=5.0)
    parser_args.add_argument(
        "--sourcetable", action="store_true", help="Yayın listesini dök ve çık"
    )
    args = parser_args.parse_args()

    if args.sourcetable:
        print(fetch_sourcetable(args.host, args.port))
        return 0

    user = os.environ.get("TUSAGA_USER")
    password = os.environ.get("TUSAGA_PASS")
    if not user or not password:
        print(
            "[HATA] TUSAGA_USER ve TUSAGA_PASS ortam değişkenleri gerekli.\n"
            "  export TUSAGA_USER='...'\n"
            "  export TUSAGA_PASS='...'",
            file=sys.stderr,
        )
        return 2

    if args.ros_fix:
        fix = get_fix_from_ros()
        if fix is None:
            return 2
        lat, lon, alt = fix
        print(f"[ros] /here4/gps/fix -> {lat:.7f}, {lon:.7f}, {alt:.1f} m")
    elif args.lat is not None and args.lon is not None:
        lat, lon, alt = args.lat, args.lon, args.alt
    else:
        print(
            "[HATA] Konum gerekli: --lat/--lon ver ya da --ros-fix kullan.\n"
            "  VRS sanal bazı bu konuma üretir; yanlış konum = işe yaramaz düzeltme.",
            file=sys.stderr,
        )
        return 2

    print(f"[bağlan] {args.host}:{args.port}/{args.mountpoint} kullanıcı={user}")
    try:
        sock, status_line, is_chunked = open_ntrip(
            args.host, args.port, args.mountpoint, user, password
        )
        print(f"[caster] {status_line}" + (" (chunked)" if is_chunked else ""))
    except (OSError, PermissionError, ConnectionError) as exc:
        print(f"[HATA] {exc}", file=sys.stderr)
        return 1

    try:
        rtcm, total_bytes, elapsed = run_stream(
            sock,
            lat,
            lon,
            alt,
            args.gga_period,
            args.duration,
            args.report_period,
            is_chunked,
        )
    except KeyboardInterrupt:
        print("\n[dur] kullanıcı iptali")
        return 0
    finally:
        sock.close()

    print("\n===== ÖZET =====")
    print(f"süre           : {elapsed:.1f} s")
    print(
        f"toplam bayt    : {total_bytes} ({total_bytes / max(elapsed, 1e-6):.0f} B/s)"
    )
    print(f"RTCM3 çerçeve  : {rtcm.frames}")
    print(f"CRC hatası     : {rtcm.crc_errors}")
    if rtcm.types:
        print("mesaj tipleri  :")
        for msg_type, count in sorted(rtcm.types.items()):
            print(f"  {msg_type:5d} x{count}")
    if rtcm.frames == 0:
        print(
            "\n[UYARI] Hiç geçerli RTCM3 çerçevesi çözülemedi. "
            "GGA konumu kapsama alanı dışında olabilir veya "
            "mountpoint formatı RTCM3 değildir (ör. VRSCMRP = CMR+)."
        )
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
