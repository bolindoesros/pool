```mermaid
flowchart TD
    Start(["hw_bridge is about to build\nthe /pc_to_esp_cmd packet"])

    Start --> ArmCheck{"Is armed_ true?\n(from /system/arm_state)"}
    ArmCheck -- "NO" --> Force1500A["FORCE elevator/rudder = 1500\n(hw_bridge safety gate)"]

    ArmCheck -- "YES" --> UseVar["Use whatever elevator_/rudder_\nlast came in on /cmd/elevator, /cmd/rudder"]

    UseVar --> WhoSetIt{"What last published\nto /cmd/elevator?"}

    WhoSetIt -- "loki_controller\n(inner_loop)" --> OdomCheck{"Is odometry fresh?\n(< 0.5s old)"}
    WhoSetIt -- "your manual\nros2 topic pub" --> YourValue["Your value goes through\n(e.g. 1200, 1600)"]

    OdomCheck -- "NO (stale/dead)" --> Force1500B["FORCE elevator/rudder = 1500\n(odom watchdog, our new fix)"]
    OdomCheck -- "YES (fresh)" --> RealPID["Real PID output\n(could be anything)"]

    Force1500A --> Output(["/pc_to_esp_cmd pwm = 1500"])
    Force1500B --> Output
    YourValue --> Output2(["/pc_to_esp_cmd pwm = your value\n(until controller publishes again\nand overwrites it)"])
    RealPID --> Output3(["/pc_to_esp_cmd pwm = real PID value"])

    style Force1500A fill:#5b2020
    style Force1500B fill:#5b2020
    style Output fill:#5b2020
```

**Plain-English version:**

`/pc_to_esp_cmd` gets stuck at 1500 for one of two reasons, checked in this order:

1. **Not armed.** `hw_bridge` refuses to send anything but neutral unless `/system/arm_state` says `true`. Check with:
   ```bash
   pixi run ros2 topic echo /system/arm_state --once
   ```

2. **Armed, but odometry is dead.** Even when armed, if `loki_controller` is the one driving `/cmd/elevator` (not your manual test), it now forces neutral whenever odometry is older than 0.5s — because your IMU data still isn't flowing (the ESP32/VESC USB port collision from earlier). This is the fix we just added; it's supposed to force 1500 in this case.

If you manually publish to `/cmd/elevator` or directly to `/pc_to_esp_cmd`, your value only "sticks" until the next thing publishes and overwrites it — `hw_bridge` and `loki_controller` don't know or care that you sent a manual value, they just keep running on their own timers.
