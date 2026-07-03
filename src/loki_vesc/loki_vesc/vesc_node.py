"""ROS 2 driver for a VESC motor controller over USB serial.

Speaks the VESC binary UART protocol directly. Motor commands are gated
behind a firmware-version handshake so we never drive a link that isn't
confirmed up, and a watchdog zeroes the thruster if commands stop arriving.

Subscribes:
    vesc/commands/duty_cycle  (Float64)  duty in -1..1, clamped to ±duty_cycle_max
    vesc/commands/current     (Float64)  motor current in A, clamped to ±max_current_a
    vesc/commands/rpm         (Float64)  electrical RPM, clamped to ±max_rpm

Publishes (10 Hz, SENSOR_DATA QoS):
    vesc/telemetry/{current_motor, current_in, rpm, duty, voltage_in, temp_fet}  (Float64)
    vesc/telemetry/fault  (Bool)

Parameters:
    port, baud                serial connection (port is required, e.g. /dev/vesc_1; baud 115200)
    duty_cycle_max            duty clamp, 0..1 (default 0.5)
    command_timeout_sec       watchdog: zero thruster after this silence (default 0.5)
    max_current_a, max_rpm    clamp + telemetry cutoff; <= 0 disables (default off)
    can_forward_id            also forward commands to this CAN id; < 0 disables
    invert                    flip sign of commands and telemetry (default False)
"""

import struct

import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64

# ────────────────────────── VESC protocol layer ──────────────────────────
# Framing: 0x02, payload_len(1), payload, crc16(2), 0x03   (payloads < 256 B)
# payload[0] is the command id from the firmware's COMM_PACKET_ID enum.

COMM_FW_VERSION  = 0x00
COMM_GET_VALUES  = 0x04
COMM_SET_DUTY    = 0x05
COMM_SET_CURRENT = 0x06
COMM_SET_RPM     = 0x08
COMM_FORWARD_CAN = 0x22

# SET_* values are big-endian int32 in fixed-point units.
DUTY_SCALE    = 100000  # duty -1..1
CURRENT_SCALE = 1000    # A -> mA
RPM_SCALE     = 1       # eRPM is already integer

FAULT_NAMES = {
    0: 'NONE', 1: 'OVER_VOLTAGE', 2: 'UNDER_VOLTAGE', 3: 'DRV',
    4: 'ABS_OVER_CURRENT', 5: 'OVER_TEMP_FET', 6: 'OVER_TEMP_MOTOR',
}

# GET_VALUES response fields are big-endian fixed-point integers
# (buffer_append_float16/32 in the firmware), NOT IEEE floats.
# (offset from payload start, struct format, scale divisor)
_GET_VALUES_FIELDS = {
    'temp_fet':      (1,  '>h', 10.0),
    'temp_motor':    (3,  '>h', 10.0),
    'current_motor': (5,  '>i', 100.0),
    'current_in':    (9,  '>i', 100.0),
    # id/iq at +13/+17, amp/watt-hours at +29..44, tachometers at +45..52: unused
    'duty_now':      (21, '>h', 1000.0),
    'rpm':           (23, '>i', 1.0),
    'v_in':          (27, '>h', 10.0),
}
_FAULT_OFFSET       = 53
_GET_VALUES_MIN_LEN = _FAULT_OFFSET + 1


def _crc16(data: bytes) -> int:
    """CRC16/XMODEM (poly 0x1021), as used by the VESC firmware."""
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1) & 0xFFFF
    return crc


def _make_packet(payload: bytes) -> bytes:
    header = bytes([0x02, len(payload)])
    crc = _crc16(payload)
    return header + payload + bytes([crc >> 8, crc & 0xFF, 0x03])


def _command_packet(cmd: int, value: float, scale: float, can_id: int = None) -> bytes:
    """Build a SET_* command packet, optionally wrapped for CAN forwarding."""
    body = struct.pack('>i', int(value * scale))
    if can_id is None:
        payload = bytes([cmd]) + body
    else:
        payload = bytes([COMM_FORWARD_CAN, can_id & 0xFF, cmd]) + body
    return _make_packet(payload)


def _read_packet(ser: serial.Serial):
    """Read one response packet; returns its payload, or None if the frame
    is missing, truncated, or fails the CRC check."""
    start = ser.read(1)
    if not start or start[0] != 0x02:
        return None
    length_b = ser.read(1)
    if not length_b:
        return None
    payload = ser.read(length_b[0])
    crc_b = ser.read(2)
    end = ser.read(1)
    if len(payload) != length_b[0] or len(crc_b) != 2 or not end or end[0] != 0x03:
        return None
    if _crc16(payload) != (crc_b[0] << 8 | crc_b[1]):
        return None
    return payload


def _parse_values(payload: bytes):
    """Decode a GET_VALUES payload into a dict, or None if malformed."""
    if len(payload) < _GET_VALUES_MIN_LEN or payload[0] != COMM_GET_VALUES:
        return None
    values = {
        name: struct.unpack_from(fmt, payload, offset)[0] / scale
        for name, (offset, fmt, scale) in _GET_VALUES_FIELDS.items()
    }
    values['fault'] = payload[_FAULT_OFFSET]
    return values


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


# ────────────────────────────── ROS 2 node ───────────────────────────────

class VescNode(Node):

    def __init__(self):
        super().__init__('vesc_driver')

        port         = self.declare_parameter('port',                '').value
        baud         = self.declare_parameter('baud',                115200).value
        max_dc       = self.declare_parameter('duty_cycle_max',      0.50).value
        timeout      = self.declare_parameter('command_timeout_sec', 0.5).value
        can_fwd_id   = self.declare_parameter('can_forward_id',      -1).value
        max_current  = self.declare_parameter('max_current_a',       -1.0).value
        max_rpm      = self.declare_parameter('max_rpm',             -1.0).value
        invert       = self.declare_parameter('invert',              False).value

        self._max_dc      = min(float(max_dc), 1.0)
        self._timeout     = float(timeout)
        self._can_fwd_id  = int(can_fwd_id)
        self._max_current = float(max_current)
        self._max_rpm     = float(max_rpm)
        self._invert      = bool(invert)

        if not port:
            self.get_logger().fatal("Parameter 'port' is required (e.g. -p port:=/dev/vesc_1)")
            raise SystemExit(1)
        try:
            self._ser = serial.Serial(port, baud, timeout=0.1)
        except serial.SerialException as e:
            self.get_logger().fatal(f'Cannot open {port}: {e}')
            raise SystemExit(1)
        fwd = f', CAN forwarding to ID {self._can_fwd_id}' if self._can_fwd_id >= 0 else ''
        self.get_logger().info(f'Connected to {port}{fwd}, waiting for VESC handshake')

        self._operating = False  # no motor commands until the handshake succeeds
        self._got_cmd   = False
        self._last_cmd  = self.get_clock().now()

        self.create_subscription(Float64, 'vesc/commands/duty_cycle', self._on_duty,    10)
        self.create_subscription(Float64, 'vesc/commands/current',    self._on_current, 10)
        self.create_subscription(Float64, 'vesc/commands/rpm',        self._on_rpm,     10)

        qos = rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        self._pub = {
            name: self.create_publisher(Float64, f'vesc/telemetry/{name}', qos)
            for name in ('current_motor', 'current_in', 'rpm', 'duty', 'voltage_in', 'temp_fet')
        }
        self._pub_fault = self.create_publisher(Bool, 'vesc/telemetry/fault', qos)

        self._init_timer = self.create_timer(0.25, self._handshake)
        self.create_timer(0.1, self._watchdog)
        self.create_timer(0.1, self._request_telemetry)

    # ── Handshake ─────────────────────────────────────────────

    def _handshake(self):
        """Request the firmware version until the VESC answers, then enable
        motor commands from a known-safe (zeroed) state."""
        try:
            self._ser.reset_input_buffer()
            self._ser.write(_make_packet(bytes([COMM_FW_VERSION])))
            payload = _read_packet(self._ser)
        except serial.SerialException as e:
            self.get_logger().error(f'Handshake serial error: {e}')
            return
        if payload is None or len(payload) < 3 or payload[0] != COMM_FW_VERSION:
            self.get_logger().warn('Waiting for VESC firmware-version response...',
                                   throttle_duration_sec=2.0)
            return
        self.get_logger().info(f'VESC handshake OK, firmware {payload[1]}.{payload[2]}')
        self._stop()
        self._operating = True
        self.destroy_timer(self._init_timer)

    # ── Command path ──────────────────────────────────────────

    def _on_duty(self, msg: Float64):
        self._send_command(COMM_SET_DUTY, _clamp(msg.data, self._max_dc), DUTY_SCALE)

    def _on_current(self, msg: Float64):
        current = _clamp(msg.data, self._max_current) if self._max_current > 0 else msg.data
        self._send_command(COMM_SET_CURRENT, current, CURRENT_SCALE)

    def _on_rpm(self, msg: Float64):
        erpm = _clamp(msg.data, self._max_rpm) if self._max_rpm > 0 else msg.data
        self._send_command(COMM_SET_RPM, erpm, RPM_SCALE)

    def _send_command(self, cmd: int, value: float, scale: float):
        """Write a SET_* command (plus its CAN-forward twin, if configured) and
        reset the command watchdog. `invert` is applied here so callers never
        have to think about sign conventions."""
        if not self._operating:
            self.get_logger().warn('Dropping command: VESC handshake not complete',
                                   throttle_duration_sec=2.0)
            return
        physical = -value if self._invert else value
        try:
            self._ser.write(_command_packet(cmd, physical, scale))
            if self._can_fwd_id >= 0:
                self._ser.write(_command_packet(cmd, physical, scale, self._can_fwd_id))
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write failed: {e}')
            return
        self._last_cmd = self.get_clock().now()
        self._got_cmd  = True

    def _stop(self):
        """Zero the thruster via duty=0, ignoring serial errors (best-effort)."""
        try:
            self._ser.write(_command_packet(COMM_SET_DUTY, 0.0, DUTY_SCALE))
            if self._can_fwd_id >= 0:
                self._ser.write(_command_packet(COMM_SET_DUTY, 0.0, DUTY_SCALE, self._can_fwd_id))
        except serial.SerialException:
            pass

    # ── Telemetry ─────────────────────────────────────────────

    def _request_telemetry(self):
        if not self._operating:
            return
        try:
            self._ser.reset_input_buffer()
            self._ser.write(_make_packet(bytes([COMM_GET_VALUES])))
            payload = _read_packet(self._ser)
        except serial.SerialException as e:
            self.get_logger().error(f'Telemetry read error: {e}')
            return
        values = _parse_values(payload) if payload is not None else None
        if values is None:
            self.get_logger().warn('No valid VESC response to GET_VALUES — check serial connection',
                                   throttle_duration_sec=2.0)
            return

        if self._invert:
            for key in ('current_motor', 'current_in', 'rpm', 'duty_now'):
                values[key] = -values[key]

        for name, key in (('current_motor', 'current_motor'), ('current_in', 'current_in'),
                          ('rpm', 'rpm'), ('duty', 'duty_now'),
                          ('voltage_in', 'v_in'), ('temp_fet', 'temp_fet')):
            self._pub[name].publish(Float64(data=float(values[key])))
        self._pub_fault.publish(Bool(data=values['fault'] != 0))

        fault_name = FAULT_NAMES.get(values['fault'], f"UNKNOWN({values['fault']})")
        self.get_logger().info(
            f"VESC | rpm={values['rpm']:.0f}  duty={values['duty_now']:.3f}"
            f"  I={values['current_motor']:.2f}A  Vin={values['v_in']:.1f}V"
            f"  Tfet={values['temp_fet']:.1f}°C  fault={fault_name}",
            throttle_duration_sec=1.0)

        self._check_limits(values)

    # ── Safety ────────────────────────────────────────────────

    def _check_limits(self, values: dict):
        """Zero the thruster if measured current or RPM exceed the configured
        maxima (a software backstop behind the VESC's own limits)."""
        tripped = False
        if self._max_current > 0 and abs(values['current_motor']) > self._max_current:
            self.get_logger().error(
                f"Current {values['current_motor']:.2f}A exceeds max "
                f"{self._max_current:.2f}A — zeroing thruster")
            tripped = True
        if self._max_rpm > 0 and abs(values['rpm']) > self._max_rpm:
            self.get_logger().error(
                f"RPM {values['rpm']:.0f} exceeds max {self._max_rpm:.0f} — zeroing thruster")
            tripped = True
        if tripped:
            self._stop()
            self._got_cmd = False

    def _watchdog(self):
        if not self._got_cmd:
            return
        age = (self.get_clock().now() - self._last_cmd).nanoseconds * 1e-9
        if age > self._timeout:
            self.get_logger().warn(f'No command for {age:.2f}s — zeroing thruster')
            self._stop()
            self._got_cmd = False

    def destroy_node(self):
        try:
            self._stop()
            self._ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VescNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
