# Here 4 DroneCAN ROS 2 Bridge

A production-ready ROS 2 (Humble) package designed to interface the **CubePilot Here 4** (AP_Periph) GPS/GNSS sensor directly with a ROS computer via DroneCAN, **without requiring a Pixhawk or ArduPilot/PX4 flight controller.**

This package was developed specifically to solve the common pitfalls, bootloader loops, and CAN buffer overflow issues that engineers face when using standalone Here 4 sensors in autonomous vehicles (Robotaxi, UGVs).

---

## 🌟 Core Features

- **Direct SocketCAN Bridge:** Connect your Here 4 directly to a Jetson/Raspberry Pi/PC using a USB-CAN adapter.
- **Dynamic Node ID Allocation:** Acts as a centralized allocator so the Here 4 can join the bus seamlessly.
- **High-Precision IMU Decoding:** Decodes AP_Periph's complex Coning/Sculling Gyroscope integrations (`rate_gyro_integral` / `integration_interval`) to provide incredibly crisp, noise-free `angular_velocity` data.
- **Dynamic Covariance Matrix:** Calculates real-time `position_covariance` based on GNSS HDOP/VDOP metrics and UERE parameters.
- **Autonomous LED Management:** Bypasses the default "Rainbow/Disco" bootloader animation and autonomously broadcasts `LightsCommand` based on GPS fix status (🟢 Green = 3D Fix, 🔵 Blue = No Fix).
- **Barometer Support:** Automatically extracts and publishes `FluidPressure` and `Temperature` if enabled on the sensor.

---

## 🛠️ Hardware Configuration (DroneCAN GUI Tool) - CRITICAL!

Out of the box, the Here 4 expects a Pixhawk to give it orders. If you connect it directly to a PC, it will get stuck in a "Rainbow LED" bootloader loop or refuse to send IMU data. **You MUST configure the sensor using the [DroneCAN GUI Tool](https://dronecan.github.io/GUI_Tool/Overview/) before using this ROS package.**

### Step 1: Fix the "Rainbow LED" Bootloader Loop
1. Open DroneCAN GUI Tool and connect to your CAN adapter (e.g., `slcan0` or `can0`).
2. Wait for the Here 4 to appear in the node list and **double-click** it.
3. Click the **Parameters** button.
4. Search for `CAN_NODE`. By default, this is `0` (Dynamic). 
5. **Fix:** Set `CAN_NODE` to `125` (or any static ID between 1-125).
6. Click **Store All**.
*Result: The sensor will now instantly boot into operational mode on power-up without waiting for a Pixhawk allocator.*

### Step 2: Enable the IMU (Gyro & Accelerometer)
By default, AP_Periph disables IMU broadcasting over CAN to save bandwidth.
1. In the Parameters menu, search for `IMU_SAMPLE_RATE`.
2. **Fix:** Change it from `0` to `100` (100 Hz is optimal for EKF).
3. Click **Store All** and reboot the sensor.

### Step 3: Enable the Barometer (Optional)
If you need high-precision altitude data (Z-axis):
1. Search for `BARO_ENABLE`.
2. **Fix:** Change it from `0` to `1`.
3. Click **Store All** and reboot.

---

## 🩹 The "Waveshare USB-CAN" Buffer Overflow Hack

If you are using a budget USB-CAN adapter (like the **Waveshare USB-CAN-A**) via `slcan`, you will likely encounter "Timeout" errors when trying to save parameters in the DroneCAN GUI Tool. 

**Why?** Setting a parameter with a long name (like `IMU_SAMPLE_RATE`) forces the PC to blast 4 CAN frames instantly. Budget adapters have tiny hardware buffers that overflow, dropping the frames.
**Our Fix:** We included a custom initialization script (`scripts/waveshare_socketcan_bridge.py`) that applies a micro-sleep (`time.sleep(0.002)`) between multi-frame DroneCAN transmissions, completely eliminating the timeouts!

---

## 🚀 Installation & Usage

### 1. Build the Package
```bash
cd ~/your_ws
colcon build --packages-select here4_dronecan_bridge --symlink-install
source install/setup.bash
```

### 2. Run the Node
```bash
ros2 launch here4_dronecan_bridge here4_bridge_launch.py
```

---

## ⚙️ ROS 2 Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `can_interface` | `can0` | The SocketCAN interface your adapter is bound to. |
| `node_id` | `10` | The DroneCAN Node ID for your ROS computer. |
| `uere` | `2.0` | User Equivalent Range Error (meters) for standalone ZED-F9P. Used to dynamically calculate the GNSS covariance matrix. *(Set to `0.02` if using RTK)*. |

## 📡 Published Topics
- `/here4/gps/fix` (`sensor_msgs/msg/NavSatFix`) - GNSS Data with dynamic Covariance
- `/here4/imu/data` (`sensor_msgs/msg/Imu`) - 100Hz Accelerometer & Integral-filtered Gyro
- `/here4/mag` (`sensor_msgs/msg/MagneticField`) - Magnetometer (Compass) Data
- `/here4/baro/pressure` (`sensor_msgs/msg/FluidPressure`) - Static Air Pressure (If `BARO_ENABLE=1`)
- `/here4/baro/temperature` (`sensor_msgs/msg/Temperature`) - Internal Sensor Temperature
