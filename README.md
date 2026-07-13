# Loki AUV

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

## Packages

| Package | Purpose |
|---|---|
| `loki_bringup` | launch files, config (PID, EKF, VESC), URDF |
| `loki_control` | cascaded PID controller (`/target/*` → `/cmd/*`) |
| `loki_actuators` | hw_bridge: `/cmd/*` → VESC duty + ESP32 servo packet |
| `loki_vesc` | VESC serial driver (thrusters) |
| `loki_icm` | ESP32 raw sensor → IMU/mag topics |
| `loki_cerulean` | Tracker 650 DVL driver  |
| `loki_monitor` | Foxglove telemetry topics |
| `loki_msgs` / `auv_interfaces` | messages |

