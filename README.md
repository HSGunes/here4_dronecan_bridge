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

## 🔌 Wiring & Schematic

This package is designed to operate over a CAN bus. We strongly recommend using the budget-friendly [Waveshare USB-CAN-A](https://www.waveshare.com/wiki/USB-CAN-A) adapter to connect the sensor directly to your ROS computer (PC, Raspberry Pi, Jetson).

### Physical Connections:
**Here 4 CAN Port** <----------------> **Waveshare USB-CAN-A**
- `CAN_H` (High)   <----------------> `CAN_H`
- `CAN_L` (Low)    <----------------> `CAN_L`
- `GND` (Ground)   <----------------> `GND`

*Note on Power:* The Waveshare adapter does **not** provide 5V power over the CAN lines by default. You must supply 5V to the Here 4 sensor separately (either via a Power Distribution Board, a 5V BEC, or splicing USB 5V into the Here 4's power pins).

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

### How to Use the Custom Waveshare Bridge
Instead of using the standard `slcand` daemon, run our setup script before launching the ROS node or the DroneCAN GUI Tool:

```bash
# 1. Make the setup script executable (only needed once)
chmod +x src/here4_dronecan_bridge/scripts/setup_waveshare_can.sh

# 2. Run the script with your adapter's USB port (usually /dev/ttyUSB0)
sudo src/here4_dronecan_bridge/scripts/setup_waveshare_can.sh /dev/ttyUSB0
```
*(This script will automatically bring up the `can0` interface and run our patched python bridge in the background).*

---

## 🚀 Installation & Usage

### 0. Prerequisites
You must install the official DroneCAN library on your ROS computer before building or running this node:
```bash
pip3 install dronecan
```

### 1. Build the Package
```bash
cd ~/your_ws
colcon build --packages-select here4_dronecan_bridge --symlink-install
source install/setup.bash
```

### 2. Run the Node
```bash
# Standard run (UERE = 2.0m for standalone GNSS)
ros2 launch here4_dronecan_bridge here4_bridge_launch.py

# With TUSAGA-Aktif RTK corrections (see the RTK section below)
export TUSAGA_USER='...' TUSAGA_PASS='...'
ros2 launch here4_dronecan_bridge here4_bridge_launch.py ntrip_enabled:=true
```

---

## 📶 RTK via NTRIP (TUSAGA-Aktif)

The Here 4 has no USB port, so corrections reach its ZED-F9P over the CAN bus:

```
NTRIP caster ──TCP──> NtripClient thread ──queue──> spin thread
   ↑ NMEA GGA uplink                                    │
                                                        ▼
                        uavcan.equipment.gnss.RTCMStream (1062, ≤128 B/msg)
                                                        │
                    AP_Periph handle_RTCMStream() ──> gps.handle_gps_rtcm_fragment()
                                                        │
                                                        ▼  ZED-F9P
                            Fix2.mode=RTK, sub_mode=FIXED ──> /here4/gps/fix
```

**TUSAGA-Aktif connection details** (verified against the live caster source table):

| | |
|---|---|
| Caster | `212.156.70.42:2101` |
| Mountpoint | `VRSRTCM34` — RTCM 3.4, GPS+GLO+GAL+BDS+QZS (fallback: `VRSRTCM31`, GPS+GLONASS only) |
| Auth | HTTP Basic — credentials come with your TKGM subscription |
| NMEA GGA | **Required.** Every stream advertises `nmea=1`; the VRS is generated at the position you report. No GGA, no corrections. |

Credentials are read from `TUSAGA_USER` / `TUSAGA_PASS` **environment variables** by
default — do not put a password in a launch file, this repo is public. The
`ntrip_user` / `ntrip_password` parameters override them if you must.

### Two gotchas that cost us a field session

**The NTRIP username is not your portal login.** `tusaga-aktif.gov.tr` signs you in
with your *e-mail address*; the caster wants the *per-receiver* username/password
listed under **"Tanımlı Alıcılar"** in that portal (ours looks like `K040100701`).
Using the portal e-mail gets a clean `401 Unauthorized`.

**The caster streams `Transfer-Encoding: chunked`** (`Server: Trimble Ntrip Caster
5.2`). A client that ignores this splices ASCII chunk-length headers (`1F\r\n`,
`400\r\n`, …) straight into the RTCM byte stream — and because chunk boundaries fall
at arbitrary offsets, they land *inside* RTCM frames. Measured on a real 23 251-byte
capture: **4.12 % of bytes outside any frame, ~5 % of frames destroyed.** `ChunkedDecoder`
handles this; after the fix the same 30 s window decodes 211 frames with **0 CRC errors**
(was 203 frames / 10 CRC errors).

### Verify your subscription before touching CAN

```bash
export TUSAGA_USER='...' TUSAGA_PASS='...'
ros2 run here4_dronecan_bridge ntrip_test --lat 37.05 --lon 35.32
# or, once the Here 4 already has a standalone fix:
ros2 run here4_dronecan_bridge ntrip_test --ros-fix
```

The script only reads — it needs neither CAN nor the Here 4. It reports byte rate
and decoded RTCM message types. `--sourcetable` dumps the caster's stream list
without sending any credentials.

### Checking that it worked

```bash
ros2 topic echo /here4/gps/fix --field status.status   # 2 = STATUS_GBAS_FIX = RTK
candump can0 | grep -c 1062                            # RTCMStream frames on the bus
```

The node logs a `RTCM: ...` line every 10 s with frame counts, dropped fragments
and the current fix quality.

---

## ⚙️ ROS 2 Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `can_interface` | `can0` | The SocketCAN interface your adapter is bound to. |
| `node_id` | `10` | The DroneCAN Node ID for your ROS computer. |
| `uere` | `2.0` | User Equivalent Range Error (meters) for a standalone (SINGLE) solution. Used to dynamically calculate the GNSS covariance matrix. |
| `uere_rtk_fixed` | `0.02` | UERE when `Fix2.mode=RTK, sub_mode=FIXED`. |
| `uere_rtk_float` | `0.30` | UERE when `Fix2.mode=RTK, sub_mode=FLOAT`. |
| `uere_dgps` | `1.00` | UERE when `Fix2.mode=DGPS`. |
| `ntrip_enabled` | `false` | Enable the NTRIP client. |
| `ntrip_host` / `ntrip_port` | `212.156.70.42` / `2101` | Caster address. |
| `ntrip_mountpoint` | `VRSRTCM34` | Correction stream. |
| `ntrip_user` / `ntrip_password` | `""` | Falls back to `TUSAGA_USER` / `TUSAGA_PASS`. |
| `ntrip_gga_period` | `5.0` | Seconds between GGA uplinks. |
| `ntrip_fallback_lat` / `_lon` | `0.0` | GGA position to use before the first fix (`0` = disabled). Useful for indoor testing; a real fix always wins. |
| `rtcm_max_fragments_per_cycle` | `8` | Caps RTCM bursts per spin cycle so they cannot starve the 100 Hz IMU stream on the CAN bus. |

**Why UERE is no longer a single fixed number:** with a hardcoded `uere:=0.02`, losing
RTK lock (LTE dropout, overpass) leaves the covariance at 2 cm while the position
error grows to metres — the EKF would keep trusting it blindly. The value now
follows `Fix2.mode`/`sub_mode`.

## 📡 Published Topics
- `/here4/gps/fix` (`sensor_msgs/msg/NavSatFix`) - GNSS Data with dynamic Covariance
- `/here4/imu/data` (`sensor_msgs/msg/Imu`) - 100Hz Accelerometer & Integral-filtered Gyro
- `/here4/mag` (`sensor_msgs/msg/MagneticField`) - Magnetometer (Compass) Data
- `/here4/baro/pressure` (`sensor_msgs/msg/FluidPressure`) - Static Air Pressure (If `BARO_ENABLE=1`)
- `/here4/baro/temperature` (`sensor_msgs/msg/Temperature`) - Internal Sensor Temperature
