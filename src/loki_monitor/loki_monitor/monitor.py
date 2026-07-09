#!/usr/bin/env python3
"""
Foxglove monitoring node for loki auv.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float64, Int32

PATH_MAX_POSES   = 2000  # cap trail length so paths don't grow unbounded
PATH_PUBLISH_SEC = 1.0   # republish full paths at 1 Hz, not per odom tick

class MonitorNode(Node):

    def __init__(self) -> None:
        super().__init__("loki_monitor")

        # reliable qos
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self._pub = {
            # vehicle state (derived from odometry/filtered)
            "depth":           self.create_publisher(Float64, "/monitor/depth",           qos),
            "pitch":           self.create_publisher(Float64, "/monitor/pitch_deg",       qos),
            "roll":            self.create_publisher(Float64, "/monitor/roll_deg",        qos),
            "heading":         self.create_publisher(Float64, "/monitor/heading_deg",     qos),
            "speed":           self.create_publisher(Float64, "/monitor/speed",           qos),
            "armed":           self.create_publisher(Float64, "/monitor/armed",           qos),
            # commands, cmd_thruster/elevator/rudder are in PWM, cmd_moving_mass is distance (m)
            "cmd_thruster":    self.create_publisher(Float64, "/monitor/cmd/thruster",    qos),
            "cmd_elevator":    self.create_publisher(Float64, "/monitor/cmd/elevator",    qos),
            "cmd_rudder":      self.create_publisher(Float64, "/monitor/cmd/rudder",      qos),
            "cmd_moving_mass": self.create_publisher(Float64, "/monitor/cmd/moving_mass", qos),
        }

        # paths for localisation visualization
        self._ekf_path = Path()
        self._gt_path  = Path()
        self._ekf_path.header.frame_id = "odom"
        self._gt_path.header.frame_id  = "odom"
        self._pub["path_ekf"] = self.create_publisher(Path, "/path/ekf",          10)
        self._pub["path_gt"]  = self.create_publisher(Path, "/path/ground_truth", 10)
        self.create_timer(PATH_PUBLISH_SEC, self._publish_paths)

        # vehicle state
        self.create_subscription(Odometry, "/odometry/filtered",  self._on_odom,        qos)
        self.create_subscription(Odometry, "/ground_truth/odom",  self._on_gt_odom,     10)
        self.create_subscription(Bool,     "/system/arm_state",   self._on_arm_state,   qos)
        # commands
        self.create_subscription(Int32,   "/cmd/thruster",    self._on_cmd_thruster,    qos)
        self.create_subscription(Int32,   "/cmd/elevator",    self._on_cmd_elevator,    qos)
        self.create_subscription(Int32,   "/cmd/rudder",      self._on_cmd_rudder,      qos)
        self.create_subscription(Float64, "/cmd/moving_mass", self._on_cmd_moving_mass, qos)

        self.get_logger().info("MonitorNode ready")

    def _on_odom(self, msg: Odometry) -> None:
        self._pub_f("depth",  msg.pose.pose.position.z) # positive when diving!
        self._pub_f("speed",  msg.twist.twist.linear.x)

        q = msg.pose.pose.orientation
        roll, pitch, yaw = self._quat_to_rpy(q.x, q.y, q.z, q.w)
        self._publish_orientation(roll, pitch, yaw)

        pose = PoseStamped()
        pose.header = msg.header
        pose.pose   = msg.pose.pose
        self._ekf_path.header.stamp = msg.header.stamp
        self._append_capped(self._ekf_path, pose)

    def _on_gt_odom(self, msg: Odometry) -> None:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose   = msg.pose.pose
        self._gt_path.header.stamp = msg.header.stamp
        self._append_capped(self._gt_path, pose)

    def _append_capped(self, path: Path, pose: PoseStamped) -> None:
        path.poses.append(pose)
        if len(path.poses) > PATH_MAX_POSES:
            del path.poses[: len(path.poses) - PATH_MAX_POSES]

    def _publish_paths(self) -> None:
        if self._ekf_path.poses:
            self._pub["path_ekf"].publish(self._ekf_path)
        if self._gt_path.poses:
            self._pub["path_gt"].publish(self._gt_path)

    def _publish_orientation(self, roll: float, pitch: float, yaw: float) -> None:
        # roll, pitch, yaw are in radians, convert to degrees for monitoring
        self._pub_f("pitch", math.degrees(pitch))
        self._pub_f("roll",  math.degrees(roll))
        self._pub_f("heading", self._yaw_to_heading(yaw))

    def _yaw_to_heading(self, yaw: float) -> float:
        # Normalises yaw from [-pi, pi] radians to 0-360 degrees.
        heading = math.degrees(yaw)
        if heading < 0.0:
            heading += 360.0
        return heading

    def _on_arm_state(self, msg: Bool) -> None:
        self._pub_f("armed", 1.0 if msg.data else 0.0)

    def _on_cmd_thruster(self, msg: Int32) -> None:
        self._pub_f("cmd_thruster", float(msg.data))

    def _on_cmd_elevator(self, msg: Int32) -> None:
        self._pub_f("cmd_elevator", float(msg.data))

    def _on_cmd_rudder(self, msg: Int32) -> None:
        self._pub_f("cmd_rudder", float(msg.data))

    def _on_cmd_moving_mass(self, msg: Float64) -> None:
        self._pub_f("cmd_moving_mass", msg.data)

    def _pub_f(self, key: str, value: float) -> None:
        msg = Float64()
        msg.data = value
        self._pub[key].publish(msg)

    def _quat_to_rpy(self, x: float, y: float, z: float, w: float) -> tuple:
        # convert quaternion to rpy in radians
        sinr = 2.0 * (w * x + y * z)
        cosr = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr, cosr)

        sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
        pitch = math.asin(sinp)

        siny = 2.0 * (w * z + x * y)
        cosy = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny, cosy)

        return roll, pitch, yaw


def main() -> None:
    rclpy.init()
    rclpy.spin(MonitorNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()