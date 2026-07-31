#!/usr/bin/env python3
# Copyright 2026 gunes
# Licensed under the Apache-2.0 License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch the Here 4 DroneCAN bridge node with configurable parameters.

    Varsayılan olarak topic'ler lokalizasyon zincirinin beklediği standart
    isimlere remap edilir (bkz. ISSUES.md B4):
      /here4/gps/fix  -> /gps/fix   (gps_filter_node girişi)
      /here4/imu/data -> /imu/data  (fix_imu_covariance girişi)
      /here4/mag      -> /imu/mag   (imu_filter_madgwick girişi)
    Ham /here4/* isimleri için: use_standard_topics:=false
    """

    can_interface_arg = DeclareLaunchArgument(
        "can_interface",
        default_value="can0",
        description="SocketCAN interface name (e.g. can0, vcan0)",
    )

    node_id_arg = DeclareLaunchArgument(
        "node_id",
        default_value="10",
        description="DroneCAN node ID for this bridge node",
    )

    uere_arg = DeclareLaunchArgument(
        "uere",
        default_value="0.5",
        description="User Equivalent Range Error (meters) for covariance",
    )

    use_standard_topics_arg = DeclareLaunchArgument(
        "use_standard_topics",
        default_value="true",
        description="Lokalizasyon zincirinin beklediği topic isimlerine remap et",
    )

    # --- NTRIP / RTK ---
    # Kimlik bilgisi BİLEREK launch argümanı değil: bu paket herkese açık bir
    # git deposu. TUSAGA_USER / TUSAGA_PASS ortam değişkenlerinden okunur.
    ntrip_enabled_arg = DeclareLaunchArgument(
        "ntrip_enabled",
        default_value="false",
        description="TUSAGA-Aktif RTK düzeltmesini aç (TUSAGA_USER/TUSAGA_PASS gerekir)",
    )

    ntrip_mountpoint_arg = DeclareLaunchArgument(
        "ntrip_mountpoint",
        default_value="VRSRTCM34",
        description="NTRIP mountpoint (VRSRTCM34 = GPS+GLO+GAL+BDS+QZS; yedek VRSRTCM31)",
    )

    ntrip_fallback_lat_arg = DeclareLaunchArgument(
        "ntrip_fallback_lat",
        default_value="0.0",
        description="GPS fix'i yokken GGA için kullanılacak enlem (0 = kapalı)",
    )

    ntrip_fallback_lon_arg = DeclareLaunchArgument(
        "ntrip_fallback_lon",
        default_value="0.0",
        description="GPS fix'i yokken GGA için kullanılacak boylam (0 = kapalı)",
    )

    rtcm_max_fragments_arg = DeclareLaunchArgument(
        "rtcm_max_fragments_per_cycle",
        default_value="4",
        description="Spin döngüsü başına CAN'e verilecek RTCM parça sayısı. "
        "Saha ölçümü: 2 -> 4 ile RTK FIXED süresi ~360 s'den ~100 s'ye indi "
        "(1 ms TX beklemesiyle ~60 s). 4'ün üstü test edilmedi.",
    )

    rtcm_filter_arg = DeclareLaunchArgument(
        "rtcm_filter_unsupported",
        default_value="true",
        description="F9P'nin çözemediği RTCM mesajlarını CAN'e hiç koyma. "
        "Farklı bir alıcı kullanılırsa false yapılabilir.",
    )

    def setup_node(context):
        from launch_ros.actions import Node

        use_standard = (
            context.launch_configurations.get("use_standard_topics", "true").lower()
            == "true"
        )

        remappings = []
        if use_standard:
            remappings = [
                ("/here4/gps/fix", "/gps/fix"),
                ("/here4/imu/data", "/imu/data"),
                ("/here4/mag", "/imu/mag"),
            ]

        return [
            Node(
                package="here4_dronecan_bridge",
                executable="here4_bridge_node",
                name="here4_bridge_node",
                output="screen",
                parameters=[
                    {
                        "can_interface": LaunchConfiguration("can_interface"),
                        "node_id": LaunchConfiguration("node_id"),
                        "uere": LaunchConfiguration("uere"),
                        "ntrip_enabled": LaunchConfiguration("ntrip_enabled"),
                        "ntrip_mountpoint": LaunchConfiguration("ntrip_mountpoint"),
                        "ntrip_fallback_lat": LaunchConfiguration("ntrip_fallback_lat"),
                        "ntrip_fallback_lon": LaunchConfiguration("ntrip_fallback_lon"),
                        "rtcm_max_fragments_per_cycle": LaunchConfiguration(
                            "rtcm_max_fragments_per_cycle"
                        ),
                        "rtcm_filter_unsupported": LaunchConfiguration(
                            "rtcm_filter_unsupported"
                        ),
                    }
                ],
                remappings=remappings,
            )
        ]

    return LaunchDescription(
        [
            can_interface_arg,
            node_id_arg,
            uere_arg,
            use_standard_topics_arg,
            ntrip_enabled_arg,
            ntrip_mountpoint_arg,
            ntrip_fallback_lat_arg,
            ntrip_fallback_lon_arg,
            rtcm_max_fragments_arg,
            rtcm_filter_arg,
            OpaqueFunction(function=setup_node),
        ]
    )
