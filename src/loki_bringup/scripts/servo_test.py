#!/usr/bin/env python3
"""
Direct servo test — bypasses the controller and arm gate.

Publishes fixed PWM values to /cmd/elevator and /cmd/rudder so hw_bridge
forwards them straight to the ESP32 servos.

Usage:
  ros2 run loki_bringup servo_test          # sweep elevator and rudder
  ros2 run loki_bringup servo_test --elevator 1600 --rudder 1400
  ros2 run loki_bringup servo_test --neutral  # return all to 1500
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

NEUTRAL = 1500
SWEEP_STEPS = [1500, 1600, 1500, 1400, 1500]
STEP_DELAY  = 1.5  # seconds per step


class ServoTest(Node):
    def __init__(self, elevator_pwm: int, rudder_pwm: int, sweep: bool):
        super().__init__('servo_test')

        self._elevator_pub = self.create_publisher(Int32, '/cmd/elevator', 10)
        self._rudder_pub   = self.create_publisher(Int32, '/cmd/rudder',   10)

        self._elevator_pwm = elevator_pwm
        self._rudder_pwm   = rudder_pwm
        self._sweep        = sweep

        # give publishers time to connect before the first publish
        self.create_timer(0.1, self._start)
        self._started = False

    def _start(self):
        if self._started:
            return
        self._started = True

        if self._sweep:
            self._run_sweep()
        else:
            self._publish_once(self._elevator_pwm, self._rudder_pwm)

    def _publish_once(self, elevator: int, rudder: int):
        e = Int32(); e.data = elevator
        r = Int32(); r.data = rudder
        self._elevator_pub.publish(e)
        self._rudder_pub.publish(r)
        self.get_logger().info(f'elevator={elevator}µs  rudder={rudder}µs')

    def _run_sweep(self):
        self.get_logger().info('sweeping elevator and rudder through: ' + str(SWEEP_STEPS))
        for pwm in SWEEP_STEPS:
            self._publish_once(pwm, pwm)
            time.sleep(STEP_DELAY)
        self.get_logger().info('sweep done — returning to neutral')
        self._publish_once(NEUTRAL, NEUTRAL)


def main():
    parser = argparse.ArgumentParser(description='Direct servo test')
    parser.add_argument('--elevator', type=int, default=None, help='elevator PWM µs (1100-1900)')
    parser.add_argument('--rudder',   type=int, default=None, help='rudder PWM µs (1100-1900)')
    parser.add_argument('--neutral',  action='store_true',    help='send neutral (1500) to all servos')
    parser.add_argument('--sweep',    action='store_true',    help='sweep through positions (default if no args)')

    # strip ros args before argparse sees them
    args, _ = parser.parse_known_args()

    if args.neutral:
        elevator_pwm, rudder_pwm, sweep = NEUTRAL, NEUTRAL, False
    elif args.elevator is not None or args.rudder is not None:
        elevator_pwm = args.elevator if args.elevator is not None else NEUTRAL
        rudder_pwm   = args.rudder   if args.rudder   is not None else NEUTRAL
        sweep        = False
    else:
        elevator_pwm, rudder_pwm, sweep = NEUTRAL, NEUTRAL, True

    rclpy.init()
    node = ServoTest(elevator_pwm, rudder_pwm, sweep)
    rclpy.spin_once(node, timeout_sec=0.2)  # let timer fire once
    rclpy.shutdown()


if __name__ == '__main__':
    main()
