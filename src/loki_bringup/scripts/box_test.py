#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class BoxTest(Node):
    def __init__(self):
        super().__init__('box_test')

        self.speed_ms  = self.declare_parameter('speed',     1.0).value
        self.depth_m   = self.declare_parameter('depth',     1.0).value
        self.side_time = self.declare_parameter('side_time', 5.0).value  # seconds per side
        self.yaw_rate  = self.declare_parameter('yaw_rate',  15.0).value  # deg/s for corners

        self.depth_pub   = self.create_publisher(Float64, '/target/depth',   10)
        self.heading_pub = self.create_publisher(Float64, '/target/heading', 10)
        self.speed_pub   = self.create_publisher(Float64, '/target/speed',   10)

        self.heading    = 0.0
        self.state      = 'straight'
        self.side_count = 0
        self.elapsed    = 0.0
        self.done       = False

        self.get_logger().info(
            f'speed={self.speed_ms}m/s depth={self.depth_m}m '
            f'side_time={self.side_time}s yaw_rate={self.yaw_rate}deg/s'
        )

        self.create_timer(0.1, self.tick)  # 10 Hz

    def tick(self):
        if self.done:
            self._pub(self.speed_pub, 0.0)
            self._pub(self.depth_pub, 0.0)
            return

        self._pub(self.depth_pub, self.depth_m)
        self._pub(self.speed_pub, self.speed_ms)

        if self.state == 'straight':
            self._pub(self.heading_pub, self.heading)
            self.elapsed += 0.1

            if self.elapsed >= self.side_time:
                self.elapsed = 0.0
                self.side_count += 1

                if self.side_count == 4:
                    self.get_logger().info('box complete')
                    self.done = True
                    return

                self.state = 'turning'
                self.get_logger().info(f'side {self.side_count} done — turning')

        elif self.state == 'turning':
            # increment heading and wrap to [0, 360)
            self.heading  = (self.heading + self.yaw_rate * 0.1) % 360.0
            self.elapsed += self.yaw_rate * 0.1
            self._pub(self.heading_pub, self.heading)

            if self.elapsed >= 90.0:
                self.elapsed = 0.0
                self.state   = 'straight'
                self.get_logger().info(f'turn done — heading={self.heading:.1f}')

    def _pub(self, pub, val):
        msg = Float64()
        msg.data = val
        pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(BoxTest())
    rclpy.shutdown()

if __name__ == '__main__':
    main()