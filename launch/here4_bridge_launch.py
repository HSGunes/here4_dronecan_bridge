#!/usr/bin/env python3
# Copyright 2026 gunes
# Licensed under the Apache-2.0 License.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch the Here 4 DroneCAN bridge node with configurable parameters."""

    can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='SocketCAN interface name (e.g. can0, vcan0)'
    )

    node_id_arg = DeclareLaunchArgument(
        'node_id',
        default_value='10',
        description='DroneCAN node ID for this bridge node'
    )

    uere_arg = DeclareLaunchArgument(
        'uere',
        default_value='0.5',
        description='User Equivalent Range Error (meters) for covariance'
    )

    here4_bridge_node = Node(
        package='here4_dronecan_bridge',
        executable='here4_bridge_node',
        name='here4_bridge_node',
        output='screen',
        parameters=[{
            'can_interface': LaunchConfiguration('can_interface'),
            'node_id': LaunchConfiguration('node_id'),
            'uere': LaunchConfiguration('uere'),
        }],
    )

    return LaunchDescription([
        can_interface_arg,
        node_id_arg,
        uere_arg,
        here4_bridge_node,
    ])
