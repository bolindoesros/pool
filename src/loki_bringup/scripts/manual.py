#!/usr/bin/env python3
"""
Manual control — no arming required.
Publishes directly to /cmd/thruster, /cmd/elevator, /cmd/rudder.

  UP / DOWN        thruster  (+/- step)
  LEFT / RIGHT     rudder    (+/- step)
  W / S            elevator  (+/- step)
  SPACE            all neutral (1500)
  + / -            change step size
  Q or Ctrl-C      quit (all neutral first)

Output values are ramped toward the target each publish tick (like
turtlebot3's make_simple_profile) instead of jumping instantly, to
avoid sudden PWM discontinuities that cause thruster stutter.
"""
import sys
import tty
import termios
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

NEUTRAL = 1500
MIN_PWM = 1100
MAX_PWM = 1900


def _getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.buffer.read(1)
        if ch == b'\x1b':
            ch += sys.stdin.buffer.read(2)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


class ManualControl(Node):
    def __init__(self):
        super().__init__('manual')
        qos = 10
        self._pub_thrust = self.create_publisher(Int32, '/cmd/thruster', qos)
        self._pub_elev   = self.create_publisher(Int32, '/cmd/elevator', qos)
        self._pub_rudd   = self.create_publisher(Int32, '/cmd/rudder',   qos)

        # target = what key presses set instantly (the goal)
        self._target_thrust = NEUTRAL
        self._target_elev   = NEUTRAL
        self._target_rudd   = NEUTRAL

        # control = what actually gets published, ramped smoothly toward target
        self._thrust = NEUTRAL
        self._elev   = NEUTRAL
        self._rudd   = NEUTRAL

        self._step   = 50
        self._slop   = 15   # max PWM change per publish tick (tune to taste)
        self._running = True

        # Keep-alive: republish at 10 Hz so the VESC watchdog never fires
        self.create_timer(0.1, self._send_all)

    def _clamp(self, v):
        return max(MIN_PWM, min(MAX_PWM, v))

    def _ramp(self, current, target):
        if target > current:
            return min(target, current + self._slop)
        elif target < current:
            return max(target, current - self._slop)
        return current

    def _pub_i32(self, pub, val):
        m = Int32(); m.data = val; pub.publish(m)

    def _send_all(self):
        # ramp control values toward target every tick, regardless of
        # whether a key was just pressed
        self._thrust = self._ramp(self._thrust, self._target_thrust)
        self._elev   = self._ramp(self._elev,   self._target_elev)
        self._rudd   = self._ramp(self._rudd,   self._target_rudd)

        self._pub_i32(self._pub_thrust, self._thrust)
        self._pub_i32(self._pub_elev,   self._elev)
        self._pub_i32(self._pub_rudd,   self._rudd)

    def _print_state(self):
        bar_t = self._bar(self._thrust)
        bar_e = self._bar(self._elev)
        bar_r = self._bar(self._rudd)
        print(f"\r\033[K"
              f"Thrust {bar_t} {self._thrust:4d} (tgt {self._target_thrust:4d})  "
              f"Elev {bar_e} {self._elev:4d} (tgt {self._target_elev:4d})  "
              f"Rudd {bar_r} {self._rudd:4d} (tgt {self._target_rudd:4d})  "
              f"step={self._step}  "
              f"[arrows=thrust/rudd  W/S=elev  SPACE=neutral  +/-=step  Q=quit]",
              end='', flush=True)

    def _bar(self, pwm):
        pos = int((pwm - NEUTRAL) / (MAX_PWM - NEUTRAL) * 5)
        bar = ['─'] * 11
        bar[5 + pos] = '█'
        return ''.join(bar)

    def _input_loop(self):
        print("\nManual control active. No arming needed.\n")
        self._print_state()
        while self._running and rclpy.ok():
            ch = _getch()

            if ch in (b'q', b'Q', b'\x03'):
                # force immediate stop, don't wait for ramp
                self._target_thrust = NEUTRAL
                self._target_elev   = NEUTRAL
                self._target_rudd   = NEUTRAL
                self._thrust = NEUTRAL
                self._elev   = NEUTRAL
                self._rudd   = NEUTRAL
                print("\nAll neutral. Exiting.")
                self._running = False
                break
            elif ch == b' ':
                self._target_thrust = NEUTRAL
                self._target_elev   = NEUTRAL
                self._target_rudd   = NEUTRAL
            elif ch == b'\x1b[A':   # UP
                self._target_thrust = self._clamp(self._target_thrust + self._step)
            elif ch == b'\x1b[B':   # DOWN
                self._target_thrust = self._clamp(self._target_thrust - self._step)
            elif ch == b'\x1b[C':   # RIGHT
                self._target_rudd = self._clamp(self._target_rudd + self._step)
            elif ch == b'\x1b[D':   # LEFT
                self._target_rudd = self._clamp(self._target_rudd - self._step)
            elif ch in (b'w', b'W'):
                self._target_elev = self._clamp(self._target_elev + self._step)
            elif ch in (b's', b'S'):
                self._target_elev = self._clamp(self._target_elev - self._step)
            elif ch in (b'+', b'='):
                self._step = min(200, self._step + 10)
            elif ch in (b'-', b'_'):
                self._step = max(10, self._step - 10)

            self._print_state()

    def run(self):
        t = threading.Thread(target=self._input_loop, daemon=True)
        t.start()
        while self._running and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
        t.join(timeout=1.0)


def main():
    rclpy.init()
    node = ManualControl()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()