#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math

class CircleTest(Node):
    def __init__(self):
        super().__init__('circle_test')

        self.speed_ms    = self.declare_parameter('speed',    1.0).value
        self.yaw_rate    = self.declare_parameter('yaw_rate', 20.0).value
        self.depth_m     = self.declare_parameter('depth',    1.0).value
        self.num_circles = self.declare_parameter('circles',  5).value

        self.depth_pub   = self.create_publisher(Float64, '/target/depth',   10)
        self.heading_pub = self.create_publisher(Float64, '/target/heading', 10)
        self.speed_pub   = self.create_publisher(Float64, '/target/speed',   10)

        self.heading      = 0.0
        self.total_turned = 0.0
        self.done         = False

        radius_m = self.speed_ms / math.radians(self.yaw_rate)
        period_s = 360.0 / self.yaw_rate
        self.get_logger().info(
            f'speed={self.speed_ms}m/s yaw_rate={self.yaw_rate}deg/s '
            f'radius={radius_m:.2f}m period={period_s:.1f}s circles={self.num_circles}'
        )
        self.create_timer(0.1, self.tick) 

    def tick(self):
        if self.done:
            self._pub(self.speed_pub, 0.0)
            self._pub(self.depth_pub, 0.0)
            return

        # increment heading and wrap to [0, 360)
        self.heading = (self.heading + self.yaw_rate * 0.1) % 360.0
        self.total_turned += self.yaw_rate * 0.1

        self._pub(self.depth_pub,   self.depth_m)
        self._pub(self.speed_pub,   self.speed_ms)
        self._pub(self.heading_pub, self.heading)

        if self.total_turned >= 360.0 * self.num_circles:
            self.get_logger().info('circle complete')
            self.done = True

    def _pub(self, pub, val):
        msg = Float64()
        msg.data = val
        pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(CircleTest())
    rclpy.shutdown()

if __name__ == '__main__':
    main()