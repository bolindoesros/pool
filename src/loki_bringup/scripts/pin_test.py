#!/usr/bin/env python3
"""
Pin test — drives each servo channel individually so you can see which
physical GPIO pin corresponds to which servo.

Cycles through pwm[0]→pwm[3] one at a time, holding each for 2 s at 1700 µs
then returning to neutral before moving to the next.

  pwm[0] → GPIO 39  (elevator 1)
  pwm[1] → GPIO 40  (elevator 2)
  pwm[2] → GPIO 41  (rudder 1)
  pwm[3] → GPIO 42  (rudder 2)

Usage (inside the container):
  python3 ~/robot_ws/src/loki_bringup/scripts/pin_test.py
"""

import time
import rclpy
from rclpy.node import Node
from loki_msgs.msg import EspPacket
from builtin_interfaces.msg import Time

NEUTRAL  = 1500
DEFLECT  = 1700
HOLD_SEC = 2.0
SETTLE_SEC = 1.0

LABELS = [
    "GPIO 39  (pwm[0] — elevator 1)",
    "GPIO 40  (pwm[1] — elevator 2)",
    "GPIO 41  (pwm[2] — rudder 1)",
    "GPIO 42  (pwm[3] — rudder 2)",
]


class PinTest(Node):
    def __init__(self):
        super().__init__('pin_test')
        self._pub = self.create_publisher(EspPacket, '/pc_to_esp_cmd', 10)
        self._seq = 0

    def _send(self, pwm0=NEUTRAL, pwm1=NEUTRAL, pwm2=NEUTRAL, pwm3=NEUTRAL):
        msg = EspPacket()
        msg.stamp = self.get_clock().now().to_msg()
        msg.seq   = self._seq
        self._seq += 1
        msg.pwm   = [pwm0, pwm1, pwm2, pwm3]
        msg.mass_target_revs = 0
        self._pub.publish(msg)

    def _neutral(self):
        self._send()

    def run(self):
        print("\nPin test starting — all servos to neutral first.\n")
        # Pump a few neutrals so the ESP32 picks them up before we start.
        for _ in range(5):
            self._neutral()
            time.sleep(0.05)

        for ch in range(4):
            print(f">>> Channel {ch}: {LABELS[ch]}  — deflecting to {DEFLECT} µs")
            pwms = [NEUTRAL, NEUTRAL, NEUTRAL, NEUTRAL]
            pwms[ch] = DEFLECT

            # Hold deflection
            end = time.time() + HOLD_SEC
            while time.time() < end:
                self._send(*pwms)
                time.sleep(0.05)

            # Return to neutral and settle
            print(f"    returning to neutral...\n")
            end = time.time() + SETTLE_SEC
            while time.time() < end:
                self._neutral()
                time.sleep(0.05)

        print("Done — all servos neutral.")
        self._neutral()


def main():
    rclpy.init()
    node = PinTest()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
