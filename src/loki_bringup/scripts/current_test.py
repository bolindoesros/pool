#!/usr/bin/env python3
"""
Current-mode thruster test.

Publishes Float64 (A) directly to /vesc/commands/current, which the VESC
node forwards as COMM_SET_CURRENT. hw_bridge arming is NOT involved btw.

Keys:
  k / j    increase / decrease current
  d        disarm   / zero current
  + / -    change step size
  q        quit (zeros current first)
"""

import argparse
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

DEFAULT_STEP    = 1.0   # A 
DEFAULT_MAX     = 25.0  # A
PUBLISH_HZ      = 20.0

BOLD  = '\033[1m'
RESET = '\033[0m'

HELP = f"""{BOLD}  loki current test  {RESET}
  {BOLD}k/j{RESET}  increase / decrease current    {BOLD}d{RESET}  zero
  {BOLD}+/-{RESET}  step size                      {BOLD}q{RESET}  quit
"""


class CurrentTest(Node):

    def __init__(self, step: float, max_current: float):
        super().__init__('current_test')
        self._max     = max_current
        self._step    = step
        self._current = 0.0

        self._pub = self.create_publisher(Float64, '/vesc/commands/current', 10)
        self.create_timer(1.0 / PUBLISH_HZ, self._tick)

    def handle_key(self, key: str):
        if key == 'k':
            self._current = min(self._current + self._step,  self._max)
        elif key == 'j':
            self._current = max(self._current - self._step, -self._max)
        elif key == 'd':
            self._current = 0.0
        elif key in ('+', '='):
            self._step = round(min(self._step + 0.1, 2.0), 2)
        elif key in ('-', '_'):
            self._step = round(max(self._step - 0.1, 0.1), 2)

    def zero(self):
        self._current = 0.0
        self._pub.publish(Float64(data=0.0))

    def _tick(self):
        self._pub.publish(Float64(data=self._current))
        sys.stdout.write(
            f'\r  current {self._current:+.2f} A  step {self._step:.2f} A  \033[K')
        sys.stdout.flush()


def main():
    parser = argparse.ArgumentParser(description='Direct current command to VESC thrusters')
    parser.add_argument('--step',        type=float, default=DEFAULT_STEP,
                        help=f'Amps per keypress (default {DEFAULT_STEP})')
    parser.add_argument('--max-current', type=float, default=DEFAULT_MAX,
                        help=f'Hard current ceiling (default {DEFAULT_MAX} A)')
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = CurrentTest(args.step, args.max_current)
    print(HELP)

    stdin_fd  = sys.stdin.fileno()
    old_attrs = termios.tcgetattr(stdin_fd)
    try:
        tty.setcbreak(stdin_fd)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=1.0 / PUBLISH_HZ)
            if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
                if key in ('q', '\x03'):
                    break
                node.handle_key(key)
    finally:
        termios.tcsetattr(stdin_fd, termios.TCSADRAIN, old_attrs)
        node.zero()
        print()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
