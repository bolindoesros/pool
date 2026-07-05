# Loki AUV

ROS 2 (Jazzy, via [pixi](https://pixi.sh)) workspace for the Loki autonomous
underwater vehicle: cascaded PID control, EKF localization (IMU + DVL),
VESC thrusters, and ESP32-driven fins over micro-ROS.

## Setup

```bash
pixi install          # dependencies (see docs/README_PIXI.md)
pixi run build        # colcon build --symlink-install
scripts/setup_udev.sh # /dev/vesc_*, /dev/esp32_* symlinks (once per machine)
scripts/setup_dvl_eth.sh  # static IP for the DVL (after every boot)
```

## Run

```bash
pixi run start        # micro-ROS agent (ESP32, Docker)
pixi run main         # ros2 launch loki_bringup real.launch.py
pixi run arm          # arm the controller
pixi run stop         # stop the agent
```

`pixi run vesc-ps` / `vesc-kill` for VESC process hygiene; `scripts/rosbag.sh`
to record a deployment bag.

## Packages

| Package | Purpose |
|---|---|
| `loki_bringup` | launch files, config (PID, EKF, VESC), URDF |
| `loki_control` | cascaded PID controller (`/target/*` → `/cmd/*`) |
| `loki_actuators` | hw_bridge: `/cmd/*` → VESC duty + ESP32 servo packet |
| `loki_vesc` | VESC serial driver (thrusters) |
| `loki_icm` | ESP32 raw sensor → IMU/mag topics |
| `loki_cerulean` | Tracker 650 DVL driver (UDP → twist) |
| `loki_monitor` | Foxglove telemetry topics |
| `loki_msgs` / `auv_interfaces` | messages (split on purpose: type names are baked into ESP32 firmware) |

System dataflow and safety chain: [docs/dataflow.md](docs/dataflow.md)

The ESP32 firmware lives in the `loki_ws` workspace (`ESP32/SDK/firmware/esp32`), not here.
