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
            OpaqueFunction(function=setup_node),
        ]
    )
