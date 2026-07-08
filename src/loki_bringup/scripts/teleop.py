#!/usr/bin/env python3
"""
Keyboard teleop — pilot the AUV by publishing PWM to the actuators.

Publishes Int32 PWM µs (1100-1900) to hw_bridge's command topics:
  /cmd/thruster   both VESC thrusters (via duty cycle)
  /cmd/elevator   both elevator servos (pitch)
  /cmd/rudder     both rudder servos  (yaw)

Values are held and re-published at 20 Hz.
"""

import argparse
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_srvs.srv import SetBool

PWM_MIN      = 1100
PWM_MAX      = 1900
NEUTRAL      = 1500
DEFAULT_STEP = 20
PUBLISH_HZ   = 20.0

BOLD  = '\033[1m'
DIM   = '\033[2m'
GREEN = '\033[32m'
RED   = '\033[31m'
RESET = '\033[0m'

HELP = f"""{BOLD}  loki teleop  {RESET}  {RESET}
  {BOLD}8/2{RESET}  elevator             {BOLD}4/6{RESET}  rudder 
  {BOLD}j/k{RESET}  thruster             {BOLD}space{RESET}  stop 
  {BOLD}a/d{RESET}  arm / disarm         {BOLD}+/-{RESET}  step size     {BOLD}q{RESET}  quit
"""


def clamp(pwm: int) -> int:
    return max(PWM_MIN, min(PWM_MAX, pwm))


class Teleop(Node):
    def __init__(self, step: int):
        super().__init__('teleop')

        self._thruster_pub = self.create_publisher(Int32, '/cmd/thruster', 10)
        self._elevator_pub = self.create_publisher(Int32, '/cmd/elevator', 10)
        self._rudder_pub   = self.create_publisher(Int32, '/cmd/rudder',   10)
        self._arm_client   = self.create_client(SetBool, '/system/arm')

        self._step     = step
        self._thruster = NEUTRAL
        self._elevator = NEUTRAL
        self._rudder   = NEUTRAL
        self._armed    = False

        self.create_timer(1.0 / PUBLISH_HZ, self._tick)

    # --- key handling -------------------------------------------------------

    def handle_key(self, key: str):
        if key == '8':
            self._elevator = clamp(self._elevator + self._step)
        elif key == '2':
            self._elevator = clamp(self._elevator - self._step)
        elif key == '6':
            self._rudder = clamp(self._rudder + self._step)
        elif key == '4':
            self._rudder = clamp(self._rudder - self._step)
        elif key == 'k':
            self._thruster = clamp(self._thruster + self._step)
        elif key == 'j':
            self._thruster = clamp(self._thruster - self._step)
        elif key == ' ':
            self._thruster = self._elevator = self._rudder = NEUTRAL
        elif key == 'a':
            self._set_armed(True)
        elif key == 'd':
            self._set_armed(False)
        elif key in ('+', '='):
            self._step = min(self._step + 5, 200)
        elif key in ('-', '_'):
            self._step = max(self._step - 5, 5)

    def _set_armed(self, armed: bool):
        if not self._arm_client.service_is_ready():
            self.get_logger().warn('/system/arm not available — is hw_bridge running?')
            return
        req = SetBool.Request()
        req.data = armed
        self._arm_client.call_async(req)
        self._armed = armed

    # --- publish + status ---------------------------------------------------

    def _tick(self):
        for pub, value in ((self._thruster_pub, self._thruster),
                           (self._elevator_pub, self._elevator),
                           (self._rudder_pub,   self._rudder)):
            msg = Int32()
            msg.data = value
            pub.publish(msg)
        self._draw_status()

    def _draw_status(self):
        state = (f'{GREEN}ARMED   {RESET}' if self._armed
                 else f'{RED}DISARMED{RESET}')
        sys.stdout.write(
            f'\r  {state}  '
            f'elev {self._elevator}  rud {self._rudder}  thr {self._thruster}  '
            f'{DIM}step {self._step}{RESET}  \033[K')
        sys.stdout.flush()

    def neutralize(self):
        self._thruster = self._elevator = self._rudder = NEUTRAL
        self._tick()
        self._set_armed(False)


def main():
    parser = argparse.ArgumentParser(description='Keyboard teleop for the AUV actuators')
    parser.add_argument('--step', type=int, default=DEFAULT_STEP,
                        help='PWM µs change per keypress (default 20)')
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = Teleop(args.step)
    print(HELP)

    stdin_fd = sys.stdin.fileno()
    old_attrs = termios.tcgetattr(stdin_fd)
    try:
        tty.setcbreak(stdin_fd)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=1.0 / PUBLISH_HZ)
            if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
                if key == 'q' or key == '\x03':  # q or Ctrl-C
                    break
                node.handle_key(key)
    finally:
        termios.tcsetattr(stdin_fd, termios.TCSADRAIN, old_attrs)
        node.neutralize()
        print()  # drop off the status line
        node.get_logger().info('teleop exiting — disarmed and neutral')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
