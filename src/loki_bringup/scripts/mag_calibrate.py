#!/usr/bin/env python3
"""
Magnetometer hard iron calibration procedure.

Spin the vehicle slowly through two full 360 yaw rotations while this runs.
Prints hard iron offsets ready to paste into mag_calib_params.yaml.

Usage:
    ros2 run loki_bringup mag_calibrate.py [--duration 30]
"""

import argparse
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField

BOLD  = '\033[1m'
RESET = '\033[0m'


class MagCalibrator(Node):

    def __init__(self, duration: float):
        super().__init__('mag_calibrator')
        self._duration = duration
        self._samples  = []
        self._start    = None
        self._done     = False

        qos = rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        self.create_subscription(MagneticField, '/imu/mag_raw', self._on_mag, qos)
        self.create_timer(0.5, self._tick)

    def _on_mag(self, msg: MagneticField):
        if self._start is None:
            self._start = self.get_clock().now()
            print(f'{BOLD}Started — spin the vehicle slowly through 2 full yaw rotations{RESET}')
        self._samples.append((
            msg.magnetic_field.x,
            msg.magnetic_field.y,
            msg.magnetic_field.z,
        ))

    def _tick(self):
        if self._done:
            return
        if self._start is None:
            self.get_logger().info('Waiting for /imu/mag_raw...', throttle_duration_sec=2.0)
            return
        elapsed   = (self.get_clock().now() - self._start).nanoseconds * 1e-9
        remaining = self._duration - elapsed
        if remaining > 0:
            sys.stdout.write(
                f'\r  {remaining:.0f}s remaining  ({len(self._samples)} samples)  \033[K')
            sys.stdout.flush()
        else:
            self._done = True
            self._finish()

    def _finish(self):
        print()
        n = len(self._samples)
        if n < 20:
            self.get_logger().error(f'Only {n} samples — is /imu/mag_raw publishing?')
            rclpy.shutdown()
            return

        xs = [s[0] for s in self._samples]
        ys = [s[1] for s in self._samples]
        zs = [s[2] for s in self._samples]

        ox = (max(xs) + min(xs)) / 2.0
        oy = (max(ys) + min(ys)) / 2.0
        oz = (max(zs) + min(zs)) / 2.0

        print(f'  samples: {n}')
        print(f'  hard iron — x: {ox:.6f}  y: {oy:.6f}  z: {oz:.6f}')
        print()
        print(f'{BOLD}Paste into src/loki_bringup/config/mag_calib_params.yaml:{RESET}')
        print()
        print('mag_calib_node:')
        print('  ros__parameters:')
        print(f'    hard_iron: [{ox:.6f}, {oy:.6f}, {oz:.6f}]')
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=30.0,
                        help='Collection time in seconds (default 30)')
    args, _ = parser.parse_known_args()

    rclpy.init()
    rclpy.spin(MagCalibrator(args.duration))


if __name__ == '__main__':
    main()
