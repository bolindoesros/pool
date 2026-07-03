#!/usr/bin/env python3
"""
Manual control v2 — direct hardware access, no arming, bypasses hw_bridge.

Publishes straight to:
  /vesc/commands/duty_cycle  (drives BOTH vesc1 and vesc2 — they currently
                               share this one topic in real.launch.py, so
                               both thrusters move together, mirrored)
  /pc_to_esp_cmd              (EspPacket -> 4 servo PWMs on the ESP32)

  UP / DOWN        thrust    (forward / backward), ramped, published as
                              VESC duty cycle (-1.0..1.0)
  W / X            elevator pair  pwm[0], pwm[1]   += / -= step
  A / D            rudder pair    pwm[2], pwm[3]   += / -= step
  S                all 4 servo PWMs -> 1500 instantly
  SPACE            thrust -> 0 duty instantly (e-stop)
  + / -            change step size
  Q or Ctrl-C      full stop (servos 1500, thrust 0), quit

NOTE: direction of W vs X, A vs D, and UP vs DOWN was not verified against
real hardware — pick one, test, flip _DIR_* below if backwards.

All 4 servo PWMs start at 1500 (neutral) and are ramped toward their target
each publish tick (like turtlebot3's make_simple_profile) to avoid sudden
PWM discontinuities, except S / SPACE / Q which snap instantly for safety.
"""
import sys
import tty
import termios
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from loki_msgs.msg import EspPacket

NEUTRAL = 1500
MIN_PWM = 1100
MAX_PWM = 1900

# Flip these to -1 if a key moves the wrong way on real hardware.
_DIR_ELEV   = 1   # W increases, X decreases
_DIR_RUDD   = 1   # A increases, D decreases
_DIR_THRUST = 1   # UP increases (forward), DOWN decreases (backward)


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


def _pwm_to_duty(pwm):
    return max(-1.0, min(1.0, (pwm - NEUTRAL) / 400.0))


class ManualControl2(Node):
    def __init__(self):
        super().__init__('manual2')
        qos = 10
        self._pub_duty = self.create_publisher(Float64, '/vesc/commands/duty_cycle', qos)
        self._pub_esp  = self.create_publisher(EspPacket, '/pc_to_esp_cmd', qos)

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
        self._seq    = 0
        self._running = True

        # Keep-alive: republish at 10 Hz so ESP32 / VESC watchdogs never fire
        self.create_timer(0.1, self._send_all)

    def _clamp(self, v):
        return max(MIN_PWM, min(MAX_PWM, v))

    def _ramp(self, current, target):
        if target > current:
            return min(target, current + self._slop)
        elif target < current:
            return max(target, current - self._slop)
        return current

    def _send_all(self):
        self._thrust = self._ramp(self._thrust, self._target_thrust)
        self._elev   = self._ramp(self._elev,   self._target_elev)
        self._rudd   = self._ramp(self._rudd,   self._target_rudd)

        duty = Float64()
        duty.data = _pwm_to_duty(self._thrust)
        self._pub_duty.publish(duty)

        pkt = EspPacket()
        pkt.stamp = self.get_clock().now().to_msg()
        pkt.seq = self._seq
        self._seq += 1
        elev = int(self._elev)
        rudd = int(self._rudd)
        # Mirroring of servo pairs is handled in the ESP32 firmware.
        pkt.pwm = [elev, elev, rudd, rudd]
        pkt.mass_target_revs = 0
        self._pub_esp.publish(pkt)

    def _print_state(self):
        bar_t = self._bar(self._thrust)
        bar_e = self._bar(self._elev)
        bar_r = self._bar(self._rudd)
        print(f"\r\033[K"
              f"Thrust {bar_t} {self._thrust:4d} duty={_pwm_to_duty(self._thrust):+.2f} (tgt {self._target_thrust:4d})  "
              f"Elev {bar_e} {self._elev:4d} (tgt {self._target_elev:4d})  "
              f"Rudd {bar_r} {self._rudd:4d} (tgt {self._target_rudd:4d})  "
              f"step={self._step}  "
              f"[UP/DOWN=thrust  W/X=elev  A/D=rudd  SPACE=stop-thrust  S=servos-neutral  +/-=step  Q=quit]",
              end='', flush=True)

    def _bar(self, pwm):
        pos = int((pwm - NEUTRAL) / (MAX_PWM - NEUTRAL) * 5)
        bar = ['─'] * 11
        bar[5 + pos] = '█'
        return ''.join(bar)

    def _input_loop(self):
        print("\nManual control v2 active. No arming needed. Direct to VESC + ESP32.\n")
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
                # e-stop thrust only, instantly
                self._target_thrust = NEUTRAL
                self._thrust = NEUTRAL
            elif ch in (b's', b'S'):
                # servos to neutral only, instantly
                self._target_elev = NEUTRAL
                self._target_rudd = NEUTRAL
                self._elev = NEUTRAL
                self._rudd = NEUTRAL
            elif ch == b'\x1b[A':   # UP
                self._target_thrust = self._clamp(self._target_thrust + _DIR_THRUST * self._step)
            elif ch == b'\x1b[B':   # DOWN
                self._target_thrust = self._clamp(self._target_thrust - _DIR_THRUST * self._step)
            elif ch in (b'w', b'W'):
                self._target_elev = self._clamp(self._target_elev + _DIR_ELEV * self._step)
            elif ch in (b'x', b'X'):
                self._target_elev = self._clamp(self._target_elev - _DIR_ELEV * self._step)
            elif ch in (b'a', b'A'):
                self._target_rudd = self._clamp(self._target_rudd + _DIR_RUDD * self._step)
            elif ch in (b'd', b'D'):
                self._target_rudd = self._clamp(self._target_rudd - _DIR_RUDD * self._step)
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
    node = ManualControl2()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
