#!/usr/bin/env python3
# Copyright 2026 gunes
# Licensed under the Apache-2.0 License.
#
# Here 4 DroneCAN Bridge Node
# Reads GPS, IMU, and Magnetometer data from a Here 4 sensor via DroneCAN (SocketCAN)
# and publishes them as standard ROS 2 messages.
# Acts as a DroneCAN Node ID allocator so Here 4 can join without a Pixhawk.

import threading
import logging

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu, MagneticField, FluidPressure, Temperature
from std_msgs.msg import Header

import dronecan
from dronecan.app import node_monitor as _node_monitor
from dronecan.app import dynamic_node_id as _dynamic_node_id

# Suppress noisy dronecan/uavcan library logs to WARNING level
logging.getLogger('uavcan').setLevel(logging.WARNING)
logging.getLogger('dronecan').setLevel(logging.WARNING)

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
        super().__init__('here4_bridge_node')

        # --- Declare Parameters ---
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('node_id', 10)
        self.declare_parameter('uere', 2.0)  # User Equivalent Range Error for covariance

        self._can_interface = self.get_parameter('can_interface').get_parameter_value().string_value
        self._node_id = self.get_parameter('node_id').get_parameter_value().integer_value
        self._uere = self.get_parameter('uere').get_parameter_value().double_value

        # --- ROS 2 Publishers ---
        self._pub_gps = self.create_publisher(NavSatFix, '/here4/gps/fix', 10)
        self._pub_imu = self.create_publisher(Imu, '/here4/imu/data', 10)
        self._pub_mag = self.create_publisher(MagneticField, '/here4/mag', 10)
        self._pub_baro = self.create_publisher(FluidPressure, '/here4/baro/pressure', 10)
        self._pub_temp = self.create_publisher(Temperature, '/here4/baro/temperature', 10)

        # --- DroneCAN Node Setup ---
        # All DroneCAN objects are created in the spin thread to avoid
        # SQLite cross-thread errors from the ID allocator.
        self._dronecan_node = None
        self._dronecan_thread = None
        self._running = False
        self._node_monitor = None
        self._id_allocator = None
        self._last_gps_fix_status = 0  # 0 = No fix, 3 = 3D fix
        self._hdop = 0.0
        self._vdop = 0.0

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
                    name='here4_ros2_bridge',
                    software_version=dronecan.uavcan.protocol.SoftwareVersion(
                        major=0, minor=1
                    ),
                    hardware_version=dronecan.uavcan.protocol.HardwareVersion(
                        major=0, minor=0
                    ),
                )
            )
            self.get_logger().info(
                f'DroneCAN node initialized on {self._can_interface} with node_id={self._node_id}'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to initialize DroneCAN node: {e}')
            self.get_logger().error(
                'Make sure the CAN interface is up: '
                'sudo ip link set can0 up type can bitrate 1000000'
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
                'Dynamic Node ID Allocation server started (range 1-125).'
            )
        except Exception as e:
            self.get_logger().warn(
                f'Dynamic Node ID Allocation server failed to start: {e}. '
                'Devices without a static Node ID will not be able to join the bus.'
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

        self.get_logger().info('DroneCAN listener thread started.')

        # --- Spin loop ---
        import time
        last_led_time = 0.0
        
        while self._running and rclpy.ok():
            try:
                self._dronecan_node.spin(timeout=0.1)
                
                # Broadcast LED color every 1 second
                now = time.time()
                if now - last_led_time > 1.0:
                    last_led_time = now
                    self._broadcast_led_command()
                    
            except Exception as e:
                # Ignore transport errors (like toggle bit errors due to CAN frame drops)
                # Just log as a debug/warn and continue spinning so the node doesn't die.
                pass
        self.get_logger().info('DroneCAN listener thread stopped.')

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
        # Status >= 3 means 3D Fix or better
        if self._last_gps_fix_status >= 3:
            # Solid Green (Max 63)
            color.red = 0
            color.green = 63
            color.blue = 0
        else:
            # Solid Blue (Max 31)
            color.red = 0
            color.green = 0
            color.blue = 31
            
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
                f'New DroneCAN node joined: ID={src}, '
                f'health={status.health}, mode={status.mode}'
            )

    def _make_header(self, frame_id: str) -> Header:
        """Create a ROS 2 Header with current timestamp and the given frame_id."""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        return header

    def _handle_gnss_fix2(self, event):
        """Convert uavcan.equipment.gnss.Fix2 -> sensor_msgs/NavSatFix."""
        msg = NavSatFix()
        msg.header = self._make_header('here4_gps_link')

        fix2 = event.message

        # --- Position conversion ---
        msg.latitude = fix2.latitude_deg_1e8 / 1e8
        msg.longitude = fix2.longitude_deg_1e8 / 1e8

        # Prefer MSL altitude; fall back to ellipsoid
        if hasattr(fix2, 'height_msl_mm') and fix2.height_msl_mm != 0:
            msg.altitude = fix2.height_msl_mm / 1e3
        elif hasattr(fix2, 'height_ellipsoid_mm'):
            msg.altitude = fix2.height_ellipsoid_mm / 1e3
        else:
            msg.altitude = 0.0

        # --- Fix status mapping ---
        status = NavSatStatus()
        dronecan_status = getattr(fix2, 'status', 0)
        self._last_gps_fix_status = dronecan_status
        
        if dronecan_status >= 2:
            status.status = NavSatStatus.STATUS_FIX
        else:
            status.status = NavSatStatus.STATUS_NO_FIX

        status.service = NavSatStatus.SERVICE_GPS
        msg.status = status
        
        # --- Covariance calculation ---
        if self._hdop > 0.0 and self._vdop > 0.0:
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            # Fetch UERE from ROS 2 parameter
            uere = self.get_parameter('uere').value
            var_h = (self._hdop * uere) ** 2
            var_v = (self._vdop * uere) ** 2
            msg.position_covariance = [
                var_h, 0.0, 0.0,
                0.0, var_h, 0.0,
                0.0, 0.0, var_v
            ]
        else:
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self._pub_gps.publish(msg)

    def _handle_raw_imu(self, event):
        """Convert uavcan.equipment.ahrs.RawIMU -> sensor_msgs/Imu."""
        msg = Imu()
        msg.header = self._make_header('here4_imu_link')

        raw = event.message
        dt = float(getattr(raw, 'integration_interval', 0.0))

        # --- Angular velocity (rad/s) ---
        # Convert FRD to FLU (Y = -Y, Z = -Z)
        if dt > 0.0 and hasattr(raw, 'rate_gyro_integral') and len(raw.rate_gyro_integral) >= 3:
            msg.angular_velocity.x = float(raw.rate_gyro_integral[0]) / dt
            msg.angular_velocity.y = -(float(raw.rate_gyro_integral[1]) / dt)
            msg.angular_velocity.z = -(float(raw.rate_gyro_integral[2]) / dt)
        elif hasattr(raw, 'rate_gyro_latest') and len(raw.rate_gyro_latest) >= 3:
            msg.angular_velocity.x = float(raw.rate_gyro_latest[0])
            msg.angular_velocity.y = -float(raw.rate_gyro_latest[1])
            msg.angular_velocity.z = -float(raw.rate_gyro_latest[2])

        # --- Linear acceleration (m/s^2) ---
        # Convert FRD to FLU (Y = -Y, Z = -Z)
        if dt > 0.0 and hasattr(raw, 'accelerometer_integral') and len(raw.accelerometer_integral) >= 3:
            msg.linear_acceleration.x = float(raw.accelerometer_integral[0]) / dt
            msg.linear_acceleration.y = -(float(raw.accelerometer_integral[1]) / dt)
            msg.linear_acceleration.z = -(float(raw.accelerometer_integral[2]) / dt)
        elif hasattr(raw, 'accelerometer_latest') and len(raw.accelerometer_latest) >= 3:
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
            1e-5, 0.0, 0.0,
            0.0, 1e-5, 0.0,
            0.0, 0.0, 1e-5
        ]
        msg.linear_acceleration_covariance = [
            1e-3, 0.0, 0.0,
            0.0, 1e-3, 0.0,
            0.0, 0.0, 1e-3
        ]

        self._pub_imu.publish(msg)

    def _handle_magnetic_field(self, event):
        """Convert uavcan.equipment.ahrs.MagneticFieldStrength -> sensor_msgs/MagneticField."""
        msg = MagneticField()
        msg.header = self._make_header('here4_mag_link')

        mag = event.message

        # Convert from Gauss to Tesla (1 Gauss = 1e-4 Tesla)
        # Convert FRD to FLU (Y = -Y, Z = -Z)
        if hasattr(mag, 'magnetic_field_ga') and len(mag.magnetic_field_ga) >= 3:
            msg.magnetic_field.x = float(mag.magnetic_field_ga[0]) * 1e-4
            msg.magnetic_field.y = -(float(mag.magnetic_field_ga[1]) * 1e-4)
            msg.magnetic_field.z = -(float(mag.magnetic_field_ga[2]) * 1e-4)
            
            # Set covariance for magnetometer (Typical values in Tesla^2)
            msg.magnetic_field_covariance = [
                1e-7, 0.0, 0.0,
                0.0, 1e-7, 0.0,
                0.0, 0.0, 1e-7
            ]

        self._pub_mag.publish(msg)

    def _handle_static_pressure(self, event):
        """Convert uavcan.equipment.air_data.StaticPressure -> sensor_msgs/FluidPressure."""
        msg = FluidPressure()
        msg.header = self._make_header('here4_baro_link')
        msg.fluid_pressure = event.message.static_pressure
        msg.variance = event.message.static_pressure_variance
        self._pub_baro.publish(msg)

    def _handle_static_temperature(self, event):
        """Convert uavcan.equipment.air_data.StaticTemperature -> sensor_msgs/Temperature."""
        msg = Temperature()
        msg.header = self._make_header('here4_baro_link')
        # DroneCAN temperature is often in Kelvin, so it can be published as is
        # ROS 2 Temperature message doesn't strictly specify unit, but usually Celsius or Kelvin.
        # Let's leave it as raw from DroneCAN (Kelvin) to prevent precision loss.
        msg.temperature = event.message.static_temperature
        msg.variance = event.message.static_temperature_variance
        self._pub_temp.publish(msg)

    def destroy_node(self):
        """Clean up DroneCAN thread and allocator on shutdown."""
        self._running = False
        if self._id_allocator is not None:
            try:
                self._id_allocator.close()
            except Exception:
                pass
        if self._dronecan_thread is not None and self._dronecan_thread.is_alive():
            self._dronecan_thread.join(timeout=2.0)
        self.get_logger().info('Here4BridgeNode shutting down.')
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


if __name__ == '__main__':
    main()
