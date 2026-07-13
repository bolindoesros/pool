#!/usr/bin/env python3
"""
Magnetometer hard iron calibration node.

Subscribes to /imu/mag_raw, applies hard iron offsets, republishes to /imu/mag.
Default offsets are zero — passthrough until calibration values are set.

Parameters:
    hard_iron  float64[3]  [x, y, z] offsets in Tesla (default [0.0, 0.0, 0.0])
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField


class MagCalibNode(Node):

    def __init__(self):
        super().__init__('mag_calib_node')

        offsets = self.declare_parameter('hard_iron', [0.0, 0.0, 0.0]).value
        self._ox, self._oy, self._oz = offsets

        self.get_logger().info(
            f'hard iron offsets — x: {self._ox:.6f}  y: {self._oy:.6f}  z: {self._oz:.6f}')

        qos = rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        self._pub = self.create_publisher(MagneticField, '/imu/mag', qos)
        self.create_subscription(MagneticField, '/imu/mag_raw', self._on_mag, qos)

    def _on_mag(self, msg: MagneticField):
        msg.magnetic_field.x -= self._ox
        msg.magnetic_field.y -= self._oy
        msg.magnetic_field.z -= self._oz
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MagCalibNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
