# Loki AUV dataflow

Rates and gating verified against the code on 2026-07-05. If you change a
rate, watchdog, or topic, update this file — a stale diagram is worse than none.

```mermaid
flowchart TD
    subgraph ESP32["ESP32-S3 firmware (lives in loki_ws, not this repo)"]
        Sensors["ICM45686 IMU + RM3100 mag<br/>(MS5837 pressure read but unused downstream)"]
        RawPub["publish esp/raw_sensor 50 Hz<br/>(auv_interfaces/EspRawSensor)"]
        PcSub["subscribe /pc_to_esp_cmd<br/>(loki_msgs/EspPacket)"]
        Servos["4 servos, GPIO 39-42<br/>pwm[0..1]=elevator, pwm[2..3]=rudder"]
        Failsafe["2 s command timeout -> neutral"]

        Sensors --> RawPub
        PcSub --> Servos
        PcSub --> Failsafe
        Failsafe --> Servos
    end

    RawPub -->|"esp/raw_sensor"| Ahrs["ahrs_orientation (loki_icm)<br/>-> /imu/data_raw + /imu/mag, 50 Hz"]
    Ahrs -->|"/imu/data_raw"| Madgwick["imu_filter_madgwick<br/>-> /imu/data"]

    DVL["Tracker 650 DVL"] -->|"UDP :27000"| Recv["tracker650_receiver<br/>-> dvl/raw_data"]
    Recv --> Repub["tracker650_republisher<br/>parses $DVPDX; rejects logged with reason<br/>(ZERO_CONFIDENCE / MALFORMED / BAD_DT), throttled 5 s"]
    Repub -->|"/dvl/twist_stamped"| EKF

    Madgwick -->|"/imu/data"| EKF["ekf_filter (robot_localization), 50 Hz<br/>-> /odometry/filtered"]
    EKF --> Controller

    subgraph Controller["loki_controller (currently commented out of real.launch.py)"]
        Arm["/system/arm (SetBool service)<br/>arm state republished 1 Hz"]
        Outer["outer loop 25 Hz: depth PID -> desired pitch<br/>always publishes /monitor/target/*"]
        Inner["inner loop 100 Hz: speed/yaw/pitch PIDs<br/>-> /cmd/thruster,elevator,rudder (PWM 1100-1900)"]
        Watchdog["odom watchdog: stale > 0.5 s while armed<br/>-> hold neutral 1500, reset PIDs"]

        Arm --> Inner
        Outer --> Inner
        Watchdog --> Inner
    end

    Inner -->|"/cmd/*"| HwBridge

    subgraph HwBridge["hw_bridge, 25 Hz"]
        Gate["disarmed -> fins neutral, mass 0, duty 0"]
        Pkt["EspPacket to /pc_to_esp_cmd"]
        Duty["/cmd/thruster -> duty, event-driven"]
        Gate --> Pkt
    end

    Pkt -->|"/pc_to_esp_cmd"| PcSub
    Duty -->|"/vesc/commands/duty_cycle"| Vesc["vesc_driver x2 (ns vesc1/vesc2)<br/>shared duty/current/rpm topics, invert per side<br/>handshake gate; 0.5 s command watchdog -> zero<br/>telemetry 10 Hz on /vescN/vesc/telemetry/*"]
    Vesc --> Thrusters["thrusters via /dev/vesc_1, /dev/vesc_2"]

    EKF -->|"/odometry/filtered"| Monitor["loki_monitor<br/>/monitor/* Float64s per odom msg<br/>paths capped 2000 poses, published 1 Hz"]
```

## Safety chain (independent layers)

| Layer | Trips after | Action |
|---|---|---|
| controller odom watchdog | 0.5 s without `/odometry/filtered` | `/cmd/*` -> 1500, PIDs reset |
| hw_bridge arm gate | disarmed (default) | fins neutral, mass 0, duty 0 |
| VESC command watchdog | 0.5 s without a command | thruster zeroed |
| VESC current/RPM backstop | telemetry over `max_current_a` / `max_rpm` | thruster zeroed |
| ESP32 firmware failsafe | 2 s without `/pc_to_esp_cmd` | servos neutral |
