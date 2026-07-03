import struct
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool

COMM_GET_VALUES   = 0x04
COMM_SET_DUTY     = 0x05
COMM_SET_CURRENT  = 0x06
COMM_SET_RPM      = 0x08
COMM_FORWARD_CAN  = 0x21

# Fixed-point scale factors applied before packing each command as an int32.
DUTY_SCALE    = 100000  # duty (-1..1)      -> VESC fixed-point
CURRENT_SCALE = 1000    # amps              -> milliamps
RPM_SCALE     = 1       # electrical RPM    -> already integer eRPM

FAULT_NAMES = {
    0: 'NONE', 1: 'OVER_VOLTAGE', 2: 'UNDER_VOLTAGE', 3: 'DRV',
    4: 'ABS_OVER_CURRENT', 5: 'OVER_TEMP_FET', 6: 'OVER_TEMP_MOTOR',
}


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _crc16(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc


def _make_packet(payload: bytes) -> bytes:
    n = len(payload)
    header = bytes([0x02, n]) if n < 256 else bytes([0x03, n >> 8, n & 0xFF])
    crc = _crc16(payload)
    return header + payload + bytes([crc >> 8, crc & 0xFF, 0x03])


def _command_packet(cmd: int, value: float, scale: float, can_id: int = None) -> bytes:
    """Build a SET_* command packet, optionally wrapped for CAN forwarding."""
    value_i = int(value * scale)
    body = struct.pack('>i', value_i)
    if can_id is None:
        payload = bytes([cmd]) + body
    else:
        payload = bytes([COMM_FORWARD_CAN, can_id & 0xFF, cmd]) + body
    return _make_packet(payload)


def _parse_values(data: bytes):
    # COMM_GET_VALUES response payload (after the command byte):
    # temp_fet(f), temp_motor(f), current_motor(f), current_in(f),
    # id(f), iq(f), duty_now(f), rpm(f), v_in(f),
    # amp_hours(f), amp_hours_charged(f), watt_hours(f), watt_hours_charged(f),
    # tachometer(i), tachometer_abs(i), fault(b)
    if len(data) < 65:
        return None
    try:
        offset = 1  # skip command byte
        temp_fet      = struct.unpack_from('>f', data, offset)[0]; offset += 4
        temp_motor    = struct.unpack_from('>f', data, offset)[0]; offset += 4
        current_motor = struct.unpack_from('>f', data, offset)[0]; offset += 4
        current_in    = struct.unpack_from('>f', data, offset)[0]; offset += 4
        _             = struct.unpack_from('>f', data, offset)[0]; offset += 4  # id
        _             = struct.unpack_from('>f', data, offset)[0]; offset += 4  # iq
        duty_now      = struct.unpack_from('>f', data, offset)[0]; offset += 4
        rpm           = struct.unpack_from('>f', data, offset)[0]; offset += 4
        v_in          = struct.unpack_from('>f', data, offset)[0]; offset += 4
        offset       += 16  # skip amp/watt hours (4 floats)
        _             = struct.unpack_from('>i', data, offset)[0]; offset += 4  # tacho
        _             = struct.unpack_from('>i', data, offset)[0]; offset += 4  # tacho_abs
        fault         = data[offset]
        return dict(temp_fet=temp_fet, temp_motor=temp_motor,
                    current_motor=current_motor, current_in=current_in,
                    duty_now=duty_now, rpm=rpm, v_in=v_in, fault=fault)
    except Exception:
        return None


def _read_packet(ser: serial.Serial):
    """Read one VESC response packet from serial. Returns payload bytes or None."""
    start = ser.read(1)
    if not start or start[0] != 0x02:
        return None
    length_b = ser.read(1)
    if not length_b:
        return None
    length = length_b[0]
    payload = ser.read(length)
    _crc = ser.read(2)
    end = ser.read(1)
    if len(payload) != length or not end or end[0] != 0x03:
        return None
    return payload


class VescNode(Node):
    def __init__(self):
        super().__init__('vesc_driver')

        port         = self.declare_parameter('port',                '/dev/ttyVESC1').value
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

        try:
            self._ser = serial.Serial(port, baud, timeout=0.1)
            fwd_info = f', CAN forwarding to ID {self._can_fwd_id}' if self._can_fwd_id >= 0 else ''
            self.get_logger().info(f'Connected to VESC on {port}{fwd_info}')
        except serial.SerialException as e:
            self.get_logger().fatal(f'Cannot open {port}: {e}')
            raise SystemExit(1)

        self._last_cmd = self.get_clock().now()
        self._got_cmd  = False

        self.create_subscription(Float64, 'vesc/commands/duty_cycle', self._on_duty,    10)
        self.create_subscription(Float64, 'vesc/commands/current',    self._on_current, 10)
        self.create_subscription(Float64, 'vesc/commands/rpm',        self._on_rpm,     10)

        qos = rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        self._pub_current_motor = self.create_publisher(Float64, 'vesc/telemetry/current_motor', qos)
        self._pub_current_in    = self.create_publisher(Float64, 'vesc/telemetry/current_in',    qos)
        self._pub_rpm           = self.create_publisher(Float64, 'vesc/telemetry/rpm',           qos)
        self._pub_duty          = self.create_publisher(Float64, 'vesc/telemetry/duty',          qos)
        self._pub_voltage       = self.create_publisher(Float64, 'vesc/telemetry/voltage_in',    qos)
        self._pub_temp_fet      = self.create_publisher(Float64, 'vesc/telemetry/temp_fet',      qos)
        self._pub_fault         = self.create_publisher(Bool,    'vesc/telemetry/fault',         qos)

        self.create_timer(0.1, self._watchdog)
        self.create_timer(1.0, self._request_telemetry)

    def _send_command(self, cmd: int, value: float, scale: float):
        """Write a SET_* command (plus its CAN-forward twin, if configured) and
        reset the command watchdog. `value` is in the node's own sign convention;
        `invert` is applied here so callers never have to think about it."""
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

    def _on_duty(self, msg: Float64):
        self._send_command(COMM_SET_DUTY, _clamp(msg.data, self._max_dc), DUTY_SCALE)

    def _on_current(self, msg: Float64):
        current = _clamp(msg.data, self._max_current) if self._max_current > 0 else msg.data
        self._send_command(COMM_SET_CURRENT, current, CURRENT_SCALE)

    def _on_rpm(self, msg: Float64):
        erpm = _clamp(msg.data, self._max_rpm) if self._max_rpm > 0 else msg.data
        self._send_command(COMM_SET_RPM, erpm, RPM_SCALE)

    def _pub_f64(self, pub, value: float):
        msg = Float64()
        msg.data = value
        pub.publish(msg)

    def _request_telemetry(self):
        try:
            self._ser.write(_make_packet(bytes([COMM_GET_VALUES])))
            payload = _read_packet(self._ser)
            if payload is None:
                self.get_logger().warn('No VESC response to GET_VALUES — check serial connection')
                return
            v = _parse_values(payload)
            if v is None:
                self.get_logger().warn('Could not parse VESC values response')
                return

            if self._invert:
                for key in ('current_motor', 'current_in', 'rpm', 'duty_now'):
                    v[key] = -v[key]

            self._pub_f64(self._pub_current_motor, v['current_motor'])
            self._pub_f64(self._pub_current_in,    v['current_in'])
            self._pub_f64(self._pub_rpm,           v['rpm'])
            self._pub_f64(self._pub_duty,          v['duty_now'])
            self._pub_f64(self._pub_voltage,       v['v_in'])
            self._pub_f64(self._pub_temp_fet,      v['temp_fet'])
            fault_active = v['fault'] != 0
            msg_b = Bool(); msg_b.data = fault_active
            self._pub_fault.publish(msg_b)

            fault_name = FAULT_NAMES.get(v['fault'], f'UNKNOWN({v["fault"]})')
            self.get_logger().info(
                f"VESC | rpm={v['rpm']:.0f}  duty={v['duty_now']:.3f}"
                f"  I={v['current_motor']:.2f}A  Vin={v['v_in']:.1f}V"
                f"  Tfet={v['temp_fet']:.1f}°C  fault={fault_name}"
            )

            tripped = False
            if self._max_current > 0 and abs(v['current_motor']) > self._max_current:
                self.get_logger().error(
                    f'Current {v["current_motor"]:.2f}A exceeds max {self._max_current:.2f}A — zeroing thruster')
                tripped = True
            if self._max_rpm > 0 and abs(v['rpm']) > self._max_rpm:
                self.get_logger().error(
                    f'RPM {v["rpm"]:.0f} exceeds max {self._max_rpm:.0f} — zeroing thruster')
                tripped = True
            if tripped:
                self._stop()
                self._got_cmd = False
        except serial.SerialException as e:
            self.get_logger().error(f'Telemetry read error: {e}')

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
