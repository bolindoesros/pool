#!/usr/bin/env python3
"""
step response logger for loki auv depth/pitch tuning.
exports a csv with time, target depth, actual depth, desired pitch, actual pitch, and elevator command.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64
import csv
import time
import os


class StepLogger(Node):

    def __init__(self) -> None:
        super().__init__("step_logger")

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self._data = []
        self._start = time.time()

        self.create_subscription(Float64, "/monitor/depth",         self._cb("depth"),         qos)
        self.create_subscription(Float64, "/monitor/pitch_deg",     self._cb("pitch"),         qos)
        self.create_subscription(Float64, "/monitor/desired_pitch", self._cb("desired_pitch"), qos)
        self.create_subscription(Float64, "/monitor/cmd/elevator",  self._cb("cmd_elevator"),  qos)
        self.create_subscription(Float64, "/target/depth",          self._cb("target_depth"),  qos)

        self._state = {
            "depth": 0.0,
            "pitch": 0.0,
            "desired_pitch": 0.0,
            "cmd_elevator": 0.0,
            "target_depth": 0.0,
        }

        # log at 10Hz
        self.create_timer(0.1, self._log)

    def _cb(self, key: str):
        def callback(msg: Float64):
            self._state[key] = msg.data
        return callback

    def _log(self) -> None:
        t = round(time.time() - self._start, 2)
        self._data.append([
            t,
            round(self._state["target_depth"],   3),
            round(self._state["depth"],           3),
            round(self._state["desired_pitch"],   3),
            round(self._state["pitch"],           3),
            round(self._state["cmd_elevator"],    3),
        ])

    def save(self) -> None:
        path = os.path.expanduser("~/loki_step_response.csv")
        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["time", "target_depth", "depth", "desired_pitch", "pitch", "cmd_elevator"])
            writer.writerows(self._data)
        self.get_logger().info(f"Saved to {path}")


def main() -> None:
    rclpy.init()
    node = StepLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save()
    rclpy.shutdown()


if __name__ == "__main__":
    main()