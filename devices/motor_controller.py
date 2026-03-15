"""
오토닉스 PMC-2HSP 모터 드라이버 통신 모듈
- 드라이버 2개, 모터 4개 (각 드라이버당 X/Y 2축)
- Modbus RTU: 연속운전, 속도/가감속 설정
- P1 전용 프로토콜: 절대/상대 좌표 이동, 직선 보간
- 선형축(5상): 분해능 100 내장, 0.72°/pulse, 5mm 리드, 500pps=5mm → 100 pulse/mm
- 회전축: 400pulse/모터회전, 기어비90:1 → 스테이지 0.01°/pulse (36,000pulse=1바퀴)
"""
try:
    from pymodbus.client import ModbusSerialClient
except ImportError:
    from pymodbus.client.sync import ModbusSerialClient
from typing import Optional, Dict, Callable
from dataclasses import dataclass
from enum import Enum
import inspect
import threading
import time


def _detect_slave_kw(client) -> str:
    """pymodbus 버전에 맞는 slave ID 키워드를 자동 감지"""
    try:
        sig = inspect.signature(client.write_register)
        for kw in ('device_id', 'slave', 'unit'):
            if kw in sig.parameters:
                return kw
    except Exception:
        pass
    for kw in ('device_id', 'slave', 'unit'):
        try:
            client.read_holding_registers(0, count=1, **{kw: 1})
            return kw
        except TypeError:
            continue
        except Exception:
            return kw
    return 'device_id'

_SLAVE_KW = 'device_id'

# 9600 baud 환경 권장 명령 간격
CMD_GAP_SEC = 0.05


# ==================== 상수 정의 ====================

# 선형축(5상 스테이지): 분해능 내장, 1,000 pulse = 1 mm (0.001 mm/pulse)
PULSE_PER_REV = 500
MM_PER_REV = 5
PULSE_PER_MM = 1000

# 회전축: 드라이버 입력 400pulse = 모터 1회전, 기어비 90:1
# 스테이지 1회전(360°) = 400 × 90 = 36,000 pulse
# → 스테이지 기준 0.01°/pulse
ROTATE_PULSE_PER_MOTOR_REV = 400
ROTATE_GEAR_RATIO = 90
ROTATE_DEG_PER_PULSE = 360.0 / (ROTATE_PULSE_PER_MOTOR_REV * ROTATE_GEAR_RATIO)  # = 0.01°/pulse

# Backward-compat alias: 회전 관련 기존 코드에서 참조
STEP_ANGLE = ROTATE_DEG_PER_PULSE

CMD_REGISTER = 0x0000   # 40001: P0 명령 전용
P1_REGISTER = 0x0001   # 40002: P1 명령 (0001~0009 = P1 명령표)

# P1 명령 코드
P1_SPEED_SET = 0x61       # 속도 설정 (6byte DATA)
P1_ABSOLUTE_MOVE = 0x71   # 절대좌표 이동
P1_RELATIVE_MOVE = 0x72   # 상대좌표 이동
P1_INTERPOLATION = 0x73   # 직선 보간

# P1 축 선택
P1_AXIS_X    = 0x01
P1_AXIS_Y    = 0x02
P1_AXIS_XY   = 0x03

# 파라미터 18 설정그룹: Bit 8 = 위치 카운터 클리어
# X축 41053(0x041C), Y축 41058(0x0421)
PARAM18_X = 0x041C
PARAM18_Y = 0x0421
POS_COUNTER_CLEAR_BIT = 8  # Bit 8: 1=Enable → 현재 위치를 0으로

X_REGISTERS = {
    # Base address 0 기준 (사용자 제공표 반영)
    'speed_ratio': 0x044E,
    'accel': 0x044F,
    'decel': 0x0450,
    'start_speed': 0x0451,
    'drive_speed1': 0x0452,
    'drive_speed2': 0x0453,
    'drive_speed3': 0x0454,
    'drive_speed4': 0x0455,
    'end_pulse_width': 0x045D,
    'scale_num': 0x045E,
    'scale_den': 0x045F,
    'jerk': 0x0472,
}

Y_REGISTERS = {
    'speed_ratio': 0x0460,
    'accel': 0x0461,
    'decel': 0x0462,
    'start_speed': 0x0463,
    'drive_speed1': 0x0464,
    'drive_speed2': 0x0465,
    'drive_speed3': 0x0466,
    'drive_speed4': 0x0467,
    'post_timer1': 0x0468,
    'post_timer2': 0x0469,
    'post_timer3': 0x046A,
    'end_pulse_width': 0x046F,
    'scale_num': 0x0470,
    'scale_den': 0x0471,
    'jerk': 0x0473,
}

class MotorCommand:
    X_PLUS_CONTINUOUS = (0x01, 0x20)
    X_MINUS_CONTINUOUS = (0x01, 0x10)
    Y_PLUS_CONTINUOUS = (0x01, 0x02)
    Y_MINUS_CONTINUOUS = (0x01, 0x01)

    X_DECEL_STOP = (0x05, 0x01)
    Y_DECEL_STOP = (0x05, 0x02)
    X_IMMEDIATE_STOP = (0x05, 0x10)
    Y_IMMEDIATE_STOP = (0x05, 0x20)

    X_SPEED_1 = (0x04, 0x10)
    X_SPEED_2 = (0x04, 0x20)
    X_SPEED_3 = (0x04, 0x30)
    X_SPEED_4 = (0x04, 0x40)
    Y_SPEED_1 = (0x04, 0x01)
    Y_SPEED_2 = (0x04, 0x02)
    Y_SPEED_3 = (0x04, 0x03)
    Y_SPEED_4 = (0x04, 0x04)

    # 원점복귀 (P0 06H, 40001/0x0000) Broadcast 가능
    # Lo: 01H=X축, 02H=Y축, 03H=X|Y 동시
    HOME_RETURN_X = (0x06, 0x01)
    HOME_RETURN_Y = (0x06, 0x02)
    HOME_RETURN_XY = (0x06, 0x03)   # X+Y 동시


class MotorAxis(Enum):
    X = "X"
    Y = "Y"


class MotorDirection(Enum):
    PLUS = "plus"
    MINUS = "minus"
    CW = "cw"
    CCW = "ccw"


@dataclass
class MotorStatus:
    connected: bool = False
    running: bool = False
    speed: int = 0
    position_pulse: int = 0
    direction: Optional[MotorDirection] = None
    error: Optional[str] = None


class PMC2HSPDriver:
    def __init__(self, slave_id: int = 1, port: str = '/dev/ttyS1', baudrate: int = 9600):
        self.slave_id = slave_id
        self.port = port
        self.baudrate = baudrate
        self.client: Optional[ModbusSerialClient] = None
        self.connected = False
        self.x_status = MotorStatus()
        self.y_status = MotorStatus()
        self.on_log: Optional[Callable] = None

    def log(self, message: str):
        if self.on_log:
            self.on_log(f"[Driver {self.slave_id}] {message}")
        print(f"[Driver {self.slave_id}] {message}")

    def connect(self, client: Optional[ModbusSerialClient] = None) -> bool:
        try:
            if client:
                self.client = client
                self.connected = client.connected
            else:
                self.client = ModbusSerialClient(
                    port=self.port, baudrate=self.baudrate,
                    parity='N', stopbits=1, bytesize=8, timeout=1,
                )
                self.connected = self.client.connect()
            if self.connected:
                self.log(f"연결 성공 ({self.port})")
                self.x_status.connected = True
                self.y_status.connected = True
            else:
                self.log(f"연결 실패 ({self.port})")
            return self.connected
        except Exception as e:
            self.log(f"연결 오류: {e}")
            return False

    def disconnect(self):
        if self.client:
            self.client.close()
            self.client = None
        self.connected = False
        self.x_status.connected = False
        self.y_status.connected = False

    def _send_command(self, hi: int, lo: int) -> bool:
        if not self.connected or not self.client:
            return False
        try:
            value = (hi << 8) | lo
            result = self.client.write_register(address=CMD_REGISTER, value=value, **{_SLAVE_KW: self.slave_id})
            if result.isError():
                self.log(f"명령 실패: 0x{value:04X}")
                return False
            self.log(f"명령 전송: 0x{value:04X}")
            return True
        except Exception as e:
            self.log(f"통신 오류: {e}")
            return False

    def _write_register(self, address: int, value: int) -> bool:
        if not self.connected or not self.client:
            return False
        try:
            # Register write values are always integer (uint16).
            # Coerce any float/string input to int and clamp to 0..65535.
            try:
                value = int(float(value))
            except Exception:
                value = 0
            value = max(0, min(65535, value))

            # Prefer FC06 (single register write), and fallback to FC16 for devices
            # that only accept multiple-register write framing.
            result = self.client.write_register(
                address=address, value=value, **{_SLAVE_KW: self.slave_id}
            )
            if not result.isError():
                return True
            self.log(f"FC06 실패(addr=0x{address:04X}, val={value}) -> FC16 재시도")
            result16 = self.client.write_registers(
                address=address, values=[value], **{_SLAVE_KW: self.slave_id}
            )
            if result16.isError():
                self.log(f"FC16 실패(addr=0x{address:04X}, val={value}): {result16}")
                return False
            return True
        except Exception as e:
            self.log(f"레지스터 쓰기 오류: {e}")
            return False

    def _read_register(self, address: int) -> Optional[int]:
        if not self.connected or not self.client:
            return None
        try:
            result = self.client.read_holding_registers(address, count=1, **{_SLAVE_KW: self.slave_id})
            if result.isError():
                return None
            return result.registers[0]
        except Exception as e:
            self.log(f"레지스터 읽기 오류: {e}")
            return None

    def set_speed(self, axis: MotorAxis, speed: int, speed_num: int = 1) -> bool:
        if speed < 1 or speed > 500000:
            return False
        registers = X_REGISTERS if axis == MotorAxis.X else Y_REGISTERS
        speed_key = f'drive_speed{speed_num}'
        if speed_key not in registers:
            return False
        return self._write_register(registers[speed_key], speed)

    def set_accel(self, axis: MotorAxis, accel: int) -> bool:
        if accel < 1 or accel > 500000:
            return False
        registers = X_REGISTERS if axis == MotorAxis.X else Y_REGISTERS
        return self._write_register(registers['accel'], accel)

    def set_decel(self, axis: MotorAxis, decel: int) -> bool:
        if decel < 1 or decel > 500000:
            return False
        registers = X_REGISTERS if axis == MotorAxis.X else Y_REGISTERS
        return self._write_register(registers['decel'], decel)

    def set_pulse_scale(self, axis: MotorAxis, numerator: int = 1, denominator: int = 100) -> bool:
        registers = X_REGISTERS if axis == MotorAxis.X else Y_REGISTERS
        ok = self._write_register(registers['scale_num'], numerator)
        ok = ok and self._write_register(registers['scale_den'], denominator)
        return ok

    def select_speed(self, axis: MotorAxis, speed_num: int = 1) -> bool:
        if axis == MotorAxis.X:
            cmds = [MotorCommand.X_SPEED_1, MotorCommand.X_SPEED_2,
                    MotorCommand.X_SPEED_3, MotorCommand.X_SPEED_4]
        else:
            cmds = [MotorCommand.Y_SPEED_1, MotorCommand.Y_SPEED_2,
                    MotorCommand.Y_SPEED_3, MotorCommand.Y_SPEED_4]
        if speed_num < 1 or speed_num > 4:
            return False
        return self._send_command(*cmds[speed_num - 1])

    def start_continuous(self, axis: MotorAxis, direction: MotorDirection) -> bool:
        if axis == MotorAxis.X:
            cmd = MotorCommand.X_PLUS_CONTINUOUS if direction in (MotorDirection.PLUS, MotorDirection.CW) \
                else MotorCommand.X_MINUS_CONTINUOUS
            self.x_status.running = True
            self.x_status.direction = direction
        else:
            cmd = MotorCommand.Y_PLUS_CONTINUOUS if direction in (MotorDirection.PLUS, MotorDirection.CW) \
                else MotorCommand.Y_MINUS_CONTINUOUS
            self.y_status.running = True
            self.y_status.direction = direction
        return self._send_command(*cmd)

    def stop(self, axis: MotorAxis, immediate: bool = False) -> bool:
        if axis == MotorAxis.X:
            cmd = MotorCommand.X_IMMEDIATE_STOP if immediate else MotorCommand.X_DECEL_STOP
            self.x_status.running = False
            self.x_status.direction = None
            self.x_status.speed = 0
        else:
            cmd = MotorCommand.Y_IMMEDIATE_STOP if immediate else MotorCommand.Y_DECEL_STOP
            self.y_status.running = False
            self.y_status.direction = None
            self.y_status.speed = 0
        return self._send_command(*cmd)

    def stop_all(self, immediate: bool = False) -> bool:
        ok_x = self.stop(MotorAxis.X, immediate)
        ok_y = self.stop(MotorAxis.Y, immediate)
        return ok_x and ok_y

    def write_drive_speed1(self, axis: MotorAxis, speed: int) -> bool:
        """Drive Speed 1 레지스터(X:0x0458-1, Y:0x0464-1)에 속도값 쓰기"""
        regs = X_REGISTERS if axis == MotorAxis.X else Y_REGISTERS
        addr = regs['drive_speed1']
        ok = self._write_register(addr, speed)
        self.log(f"write_drive_speed1: axis={axis.value} speed={speed} "
                 f"addr=0x{addr:04X} → {'OK' if ok else 'FAIL'}")
        return ok

    def clear_position_counter(self, axis: MotorAxis) -> bool:
        """현재 위치를 0으로 설정 (41053/0x041C Bit 8 = 1)"""
        addr = PARAM18_X if axis == MotorAxis.X else PARAM18_Y
        cur = self._read_register(addr)
        if cur is None:
            cur = 0
        val = cur | (1 << POS_COUNTER_CLEAR_BIT)
        ok = self._write_register(addr, val)
        self.log(f"clear_position_counter: axis={axis.value} addr=0x{addr:04X} "
                 f"cur=0x{cur:04X} val=0x{val:04X} → {'OK' if ok else 'FAIL'}")
        return ok

    def home_return(self, axis: MotorAxis) -> bool:
        """원점복귀 실행 (P0 06H, 40001/0x0000) - 단일 축"""
        cmd = MotorCommand.HOME_RETURN_X if axis == MotorAxis.X else MotorCommand.HOME_RETURN_Y
        ok = self._send_command(*cmd)
        self.log(f"home_return: axis={axis.value} → {'OK' if ok else 'FAIL'}")
        return ok

    def home_return_xy(self) -> bool:
        """원점복귀 실행 (P0 06H) - X,Y축 동시 (Lo=03H)"""
        ok = self._send_command(*MotorCommand.HOME_RETURN_XY)
        self.log(f"home_return_xy: X|Y → {'OK' if ok else 'FAIL'}")
        return ok

    def move_with_speed(self, axis: MotorAxis, direction: MotorDirection,
                        speed: int) -> bool:
        self.log(f"move_with_speed: axis={axis.value} dir={direction.value} speed={speed}PPS")
        ok_sel = self.select_speed(axis, 1)
        self.log(f"  select_speed(1) → {'OK' if ok_sel else 'FAIL'}")
        if not ok_sel:
            return False
        time.sleep(CMD_GAP_SEC)
        ok = self.start_continuous(axis, direction)
        self.log(f"  start_continuous → {'OK' if ok else 'FAIL'}")
        if ok:
            status = self.x_status if axis == MotorAxis.X else self.y_status
            status.speed = speed
        return ok

    # ── P1 전용 프로토콜 (STX/ETX 프레임) ──

    def _get_raw_serial(self):
        """pymodbus 클라이언트의 하위 serial.Serial 객체 반환 (pymodbus 버전 호환)"""
        if not self.client:
            return None
        for attr in ('socket', 'serial', '_socket', '_serial', 'transport'):
            obj = getattr(self.client, attr, None)
            if obj is not None and hasattr(obj, 'write'):
                return obj
        return None

    @staticmethod
    def _signed_to_4bytes(val: int) -> bytes:
        """signed int → 4바이트 big-endian"""
        if val < 0:
            val += 0x100000000
        return val.to_bytes(4, 'big')

    def _build_p1_packet(self, data: bytes) -> bytes:
        """
        P1 명령 패킷 생성
        [STX] [body: ID(2) + "P1"(2) + DATA(hex-ascii)] [ETX] [BCC]
        body 는 ASCII-hex 인코딩
        """
        id_str = f"{self.slave_id:02X}"
        data_hex = data.hex().upper()
        body_str = id_str + "P1" + data_hex
        body_bytes = body_str.encode('ascii')

        bcc = 0
        for b in body_bytes:
            bcc ^= b

        return bytes([0x02]) + body_bytes + bytes([0x03, bcc])

    def _send_p1(self, data: bytes) -> bool:
        """P1 패킷을 raw serial 로 전송"""
        ser = self._get_raw_serial()
        if not ser:
            self.log("P1 전송 실패: 시리얼 포트 없음")
            return False
        try:
            packet = self._build_p1_packet(data)
            ser.reset_input_buffer()
            ser.write(packet)
            self.log(f"P1 TX: {packet.hex(' ').upper()}")
            time.sleep(0.1)
            if ser.in_waiting:
                resp = ser.read(ser.in_waiting)
                self.log(f"P1 RX: {resp.hex(' ').upper()}")
            return True
        except Exception as e:
            self.log(f"P1 통신 오류: {e}")
            return False

    def set_speed_p1(self, axis: int, x_speed: int = 1000, y_speed: int = 1000) -> bool:
        """61H: P1 속도 설정"""
        x_speed = max(1, min(500000, x_speed))
        y_speed = max(1, min(500000, y_speed))
        data = bytes([P1_SPEED_SET, axis]) \
               + x_speed.to_bytes(2, 'big') \
               + y_speed.to_bytes(2, 'big')
        ok = self._send_p1(data)
        self.log(f"set_speed_p1: axis=0x{axis:02X} x={x_speed} y={y_speed} → {'OK' if ok else 'FAIL'}")
        return ok

    def move_absolute_p1(self, x_pulse: int = 0, y_pulse: int = 0,
                         axis: int = P1_AXIS_XY) -> bool:
        """71H: 절대좌표 이동"""
        data = bytes([P1_ABSOLUTE_MOVE, axis]) \
               + self._signed_to_4bytes(x_pulse) \
               + self._signed_to_4bytes(y_pulse)
        return self._send_p1(data)

    def move_relative_p1(self, x_delta: int = 0, y_delta: int = 0,
                         axis: int = P1_AXIS_XY) -> bool:
        """72H: 상대좌표 이동"""
        data = bytes([P1_RELATIVE_MOVE, axis]) \
               + self._signed_to_4bytes(x_delta) \
               + self._signed_to_4bytes(y_delta)
        return self._send_p1(data)

    def set_interpolation(self, on: bool) -> bool:
        """73H: 직선 보간 ON/OFF"""
        data = bytes([P1_INTERPOLATION, 0x01 if on else 0x00]) + bytes(8)
        return self._send_p1(data)


def mm_to_pulse(mm: float) -> int:
    return int(mm * PULSE_PER_MM)

def pulse_to_mm(pulse: int) -> float:
    return pulse / PULSE_PER_MM

def degree_to_pulse(degree: float) -> int:
    return int(degree / ROTATE_DEG_PER_PULSE)

def pulse_to_degree(pulse: int) -> float:
    return pulse * ROTATE_DEG_PER_PULSE

def speed_pps_to_mm_per_sec(pps: int) -> float:
    return pps / PULSE_PER_MM

def speed_pps_to_deg_per_sec(pps: int) -> float:
    return pps * ROTATE_DEG_PER_PULSE


class MotorController:
    MOTOR_MAP = {
        # linear 스테이지 X,Y 교환: upper=X, lower=Y
        # slave1 X -> upper_rotate, slave1 Y -> lower_rotate
        'upper_stage':  {'driver': 2, 'axis': MotorAxis.X, 'name': '상부 스테이지', 'type': 'linear'},
        'lower_stage':  {'driver': 2, 'axis': MotorAxis.Y, 'name': '하부 스테이지', 'type': 'linear'},
        'upper_rotate': {'driver': 1, 'axis': MotorAxis.X, 'name': '상부 회전',     'type': 'rotate'},
        'lower_rotate': {'driver': 1, 'axis': MotorAxis.Y, 'name': '하부 회전',     'type': 'rotate'},
    }

    def __init__(self, port: str = 'COM7', baudrate: int = 9600,
                 parity: str = 'N', rs485_mode: bool = False):
        self.port = port
        self.baudrate = baudrate
        self.parity = parity
        self.rs485_mode = rs485_mode
        self.client: Optional[ModbusSerialClient] = None
        self.connected = False
        self.driver1 = PMC2HSPDriver(slave_id=1, port=port, baudrate=baudrate)
        self.driver2 = PMC2HSPDriver(slave_id=2, port=port, baudrate=baudrate)
        self.motor_speeds: Dict[str, int] = {k: 0 for k in self.MOTOR_MAP}
        self.motor_directions: Dict[str, Optional[MotorDirection]] = {k: None for k in self.MOTOR_MAP}
        self.on_log: Optional[Callable] = None

        # 역방향 전환 시 정지 후 대기 시간 (초)
        self.reverse_guard_delay: float = 0.15

    def log(self, msg: str):
        if self.on_log:
            self.on_log(msg)
        print(f"[MotorController] {msg}")

    def connect(self) -> bool:
        global _SLAVE_KW
        try:
            self.client = ModbusSerialClient(
                port=self.port, baudrate=self.baudrate,
                parity=self.parity, stopbits=1, bytesize=8, timeout=1,
            )
            if not self.client.connect():
                self.log(f"Modbus 연결 실패: {self.port}")
                return False

            if self.rs485_mode:
                try:
                    import serial.rs485
                    raw_ser = self._get_raw_serial_from_client()
                    if raw_ser:
                        raw_ser.rs485_mode = serial.rs485.RS485Settings(
                            rts_level_for_tx=True,
                            rts_level_for_rx=False,
                            delay_before_tx=None,
                            delay_before_rx=0.005,
                        )
                        self.log("RS-485 mode enabled")
                    else:
                        self.log("RS-485 mode: could not access serial object")
                except Exception as e:
                    self.log(f"RS-485 mode failed: {e}")

            _SLAVE_KW = _detect_slave_kw(self.client)
            self.log(f"pymodbus keyword: {_SLAVE_KW}")
            self.connected = True
            self.log(f"Modbus 연결 성공: {self.port}")
            self.driver1.connect(self.client)
            self.driver2.connect(self.client)
            self._initialize_drivers()
            return True
        except Exception as e:
            self.log(f"연결 오류: {e}")
            return False

    def _get_raw_serial_from_client(self):
        """pymodbus client에서 pyserial Serial 객체 추출"""
        if not self.client:
            return None
        for attr in ('socket', 'serial', '_socket', '_serial', 'transport'):
            obj = getattr(self.client, attr, None)
            if obj and hasattr(obj, 'rs485_mode'):
                return obj
        return None

    def disconnect(self):
        self.driver1.disconnect()
        self.driver2.disconnect()
        if self.client:
            self.client.close()
            self.client = None
        self.connected = False

    def _initialize_drivers(self):
        """연결 직후 속도 배율을 1로 초기화.
        드라이버에 이전 설정값(≠1)이 남아 있으면 PPS 계산이 틀어지므로
        X/Y 속도 배율 레지스터(0x044E, 0x0460)를 반드시 1로 씀.
        """
        import time as _t
        for drv in (self.driver1, self.driver2):
            for axis, regs in [(MotorAxis.X, X_REGISTERS), (MotorAxis.Y, Y_REGISTERS)]:
                addr = regs['speed_ratio']
                try:
                    r = self.client.write_register(
                        address=addr, value=1, **{_SLAVE_KW: drv.slave_id}
                    )
                    ok = not r.isError()
                    drv.log(f"speed_ratio reset: axis={axis.value} addr=0x{addr:04X} → {'OK' if ok else 'FAIL'}")
                except Exception as e:
                    drv.log(f"speed_ratio reset error: axis={axis.value} {e}")
                _t.sleep(CMD_GAP_SEC)
        self.log("드라이버 초기화 완료 (speed_ratio=1 강제 설정)")

    def verify_connection(self) -> dict:
        """각 드라이버 실제 통신 테스트 (레지스터 읽기)"""
        result = {}
        for drv_id, drv in [(1, self.driver1), (2, self.driver2)]:
            try:
                resp = drv.client.read_holding_registers(
                    0x0000, count=1, **{_SLAVE_KW: drv.slave_id}
                )
                resp_str = str(resp)
                if hasattr(resp, 'exception_code'):
                    result[drv_id] = f"OK (응답 확인, exc={resp.exception_code})"
                    drv.log(f"통신 검증 성공 (exception response = 장비 응답 있음)")
                elif resp.isError():
                    result[drv_id] = f"응답 오류: {resp}"
                    drv.log(f"통신 검증 실패: {resp}")
                else:
                    result[drv_id] = f"OK (reg0={resp.registers[0]:#06x})"
                    drv.log(f"통신 검증 성공: {resp.registers[0]:#06x}")
            except Exception as e:
                result[drv_id] = f"예외: {e}"
                drv.log(f"통신 검증 예외: {e}")
        return result

    def _get_driver_axis(self, motor_id: str):
        cfg = self.MOTOR_MAP[motor_id]
        drv = self.driver1 if cfg['driver'] == 1 else self.driver2
        return drv, cfg['axis']

    def set_speed_all(self, speed: int) -> bool:
        """Drive Speed 1만 사용: 각 드라이버 X/Y축 drive_speed1에 속도 쓰기"""
        if not self.connected:
            return False
        ok = True
        for drv in (self.driver1, self.driver2):
            if not drv.write_drive_speed1(MotorAxis.X, speed):
                ok = False
            time.sleep(CMD_GAP_SEC)
            if not drv.write_drive_speed1(MotorAxis.Y, speed):
                ok = False
            time.sleep(CMD_GAP_SEC)
        return ok

    def set_speed_for_motor(self, motor_id: str, speed: int) -> bool:
        """특정 모터의 축 drive_speed1에 속도 쓰기"""
        if not self.connected:
            return False
        drv, axis = self._get_driver_axis(motor_id)
        ok = drv.write_drive_speed1(axis, speed)
        time.sleep(CMD_GAP_SEC)
        return ok

    def start_motor(self, motor_id: str, direction: str, speed: int = 1000) -> bool:
        if not self.connected:
            return False
        dir_map = {
            'plus': MotorDirection.PLUS, 'minus': MotorDirection.MINUS,
            'up': MotorDirection.PLUS, 'down': MotorDirection.MINUS,
            'cw': MotorDirection.CW, 'ccw': MotorDirection.CCW,
        }
        d = dir_map.get(direction.lower())
        if not d:
            return False

        # 역방향 전환 가드: 반대 방향 명령 시 정지 → 대기 → 재시작
        _fwd = {MotorDirection.PLUS, MotorDirection.CW}
        _bwd = {MotorDirection.MINUS, MotorDirection.CCW}
        cur = self.motor_directions.get(motor_id)
        if cur is not None and self.motor_speeds.get(motor_id, 0) > 0:
            is_reverse = (cur in _fwd and d in _bwd) or (cur in _bwd and d in _fwd)
            if is_reverse:
                drv, axis = self._get_driver_axis(motor_id)
                drv.stop(axis, immediate=True)
                self.motor_speeds[motor_id] = 0
                self.motor_directions[motor_id] = None
                self.log(f"[ReverseGuard] {motor_id}: stop ({cur.value}→{d.value}), wait {self.reverse_guard_delay}s")
                time.sleep(self.reverse_guard_delay)

        drv, axis = self._get_driver_axis(motor_id)
        ok = drv.move_with_speed(axis, d, speed)
        if ok:
            self.motor_speeds[motor_id] = speed
            self.motor_directions[motor_id] = d
        return ok

    def stop_motor(self, motor_id: str, immediate: bool = False) -> bool:
        if not self.connected:
            return False
        drv, axis = self._get_driver_axis(motor_id)
        ok = drv.stop(axis, immediate)
        if ok:
            self.motor_speeds[motor_id] = 0
            self.motor_directions[motor_id] = None
        return ok

    def stop_all(self, immediate: bool = False) -> bool:
        self.driver1.stop_all(immediate)
        self.driver2.stop_all(immediate)
        for k in self.motor_speeds:
            self.motor_speeds[k] = 0
            self.motor_directions[k] = None
        return True

    def clear_position_counter_all(self) -> bool:
        """4축 모두 현재 위치를 0으로 설정 (Set Home)"""
        if not self.connected:
            return False
        ok = True
        for drv in (self.driver1, self.driver2):
            ok = drv.clear_position_counter(MotorAxis.X) and ok
            time.sleep(CMD_GAP_SEC)
            ok = drv.clear_position_counter(MotorAxis.Y) and ok
            time.sleep(CMD_GAP_SEC)
        return ok

    def home_return_all(self) -> bool:
        """브로드캐스트 시도 → 실패 시 슬레이브별 전송 (broadcast는 응답 없음으로 타임아웃 가능)"""
        if not self.connected or not self.client:
            return False
        value = (MotorCommand.HOME_RETURN_XY[0] << 8) | MotorCommand.HOME_RETURN_XY[1]
        try:
            result = self.client.write_register(
                address=CMD_REGISTER, value=value, **{_SLAVE_KW: 0}
            )
            if not result.isError():
                self.log(f"home_return broadcast (0x{value:04X}) → OK")
                return True
        except Exception as e:
            self.log(f"home_return broadcast: {e} (fallback to per-slave)")
        # Fallback: broadcast 응답 없음/타임아웃 시 슬레이브별 X|Y 동시 전송
        ok = True
        for drv in (self.driver1, self.driver2):
            ok = drv.home_return_xy() and ok
            time.sleep(CMD_GAP_SEC)
        return ok

    def move_absolute(self, motor_id: str, target_pulse: int,
                      speed: int = 1000) -> bool:
        """P1 절대좌표 이동 (단일 축)"""
        if not self.connected:
            return False
        drv, axis = self._get_driver_axis(motor_id)
        clamped = min(max(1, speed), 8000)
        drv.set_speed(axis, clamped, 1)
        time.sleep(0.03)
        drv.select_speed(axis, 1)
        time.sleep(0.03)
        if axis == MotorAxis.X:
            return drv.move_absolute_p1(x_pulse=target_pulse, y_pulse=0,
                                        axis=P1_AXIS_X)
        else:
            return drv.move_absolute_p1(x_pulse=0, y_pulse=target_pulse,
                                        axis=P1_AXIS_Y)

    def move_absolute_xy(self, driver_id: int,
                         x_pulse: int, y_pulse: int,
                         speed: int = 1000) -> bool:
        """P1 XY 동시 절대좌표 이동"""
        if not self.connected:
            return False
        drv = self.driver1 if driver_id == 1 else self.driver2
        clamped = min(max(1, speed), 8000)
        drv.set_speed(MotorAxis.X, clamped, 1)
        time.sleep(0.02)
        drv.set_speed(MotorAxis.Y, clamped, 1)
        time.sleep(0.02)
        drv.select_speed(MotorAxis.X, 1)
        time.sleep(0.02)
        drv.select_speed(MotorAxis.Y, 1)
        time.sleep(0.02)
        return drv.move_absolute_p1(x_pulse, y_pulse, axis=P1_AXIS_XY)

    def move_relative(self, motor_id: str, delta_pulse: int,
                      speed: int = 1000) -> bool:
        """P1 상대좌표 이동 (단일 축) - Modbus 40002(0x0001) P1 블록에 72H 전송"""
        return self.move_relative_modbus_p1(motor_id, delta_pulse, speed)

    def _write_p1_speed_modbus(self, drv, speed: int) -> bool:
        """P1 61H: Modbus로 X/Y축 속도 설정"""
        clamped = min(max(1, speed), 500000)
        ax = P1_AXIS_XY  # X,Y 둘 다 동일 속도로 설정
        regs = [(P1_SPEED_SET << 8) | ax, clamped, clamped]  # 3 registers
        try:
            r = self.client.write_registers(
                address=P1_REGISTER, values=regs, **{_SLAVE_KW: drv.slave_id}
            )
            ok = not r.isError()
            if ok:
                self.log(f"P1 61H speed={clamped} (X,Y) → OK")
            return ok
        except Exception as e:
            self.log(f"P1 61H speed set error: {e}")
            return False

    def move_relative_modbus_p1(self, motor_id: str, delta_pulse: int,
                               speed: int = 1000) -> bool:
        """P1 72H 상대이동을 Modbus 40002(0x0001) P1 블록에 FC16으로 전송"""
        if not self.connected or not self.client:
            return False
        drv, axis = self._get_driver_axis(motor_id)
        clamped = min(max(1, speed), 500000)
        # P0 drive_speed1 + select_speed (연속운전과 동일, P1도 이 속도 사용 가능)
        drv.write_drive_speed1(axis, clamped)
        time.sleep(CMD_GAP_SEC)
        drv.select_speed(axis, 1)
        time.sleep(CMD_GAP_SEC)
        # P1 61H 속도 설정
        self._write_p1_speed_modbus(drv, clamped)
        time.sleep(0.15)

        x_delta = delta_pulse if axis == MotorAxis.X else 0
        y_delta = delta_pulse if axis == MotorAxis.Y else 0
        ax = P1_AXIS_X if axis == MotorAxis.X else P1_AXIS_Y

        def signed_to_regs(v: int) -> tuple:
            v32 = v & 0xFFFFFFFF
            return (v32 >> 16) & 0xFFFF, v32 & 0xFFFF

        x_hi, x_lo = signed_to_regs(x_delta)
        y_hi, y_lo = signed_to_regs(y_delta)
        # 40002(0x0001): 상위=명령(72H), 하위=축(01/02/03), X/Y 32bit (Hi,Lo)
        regs = [(P1_RELATIVE_MOVE << 8) | ax, x_hi, x_lo, y_hi, y_lo]
        try:
            result = self.client.write_registers(
                address=P1_REGISTER, values=regs, **{_SLAVE_KW: drv.slave_id}
            )
            ok = not result.isError()
            self.log(f"move_relative_modbus_p1 {motor_id} axis=0x{ax:02X} delta={delta_pulse} "
                     f"Reg40002={regs[0]:04X} → {'OK' if ok else 'FAIL'}")
            if not ok:
                exc = getattr(result, 'exception_code', None)
                self.log(f"  FC16 response: {result} exception_code={exc}")
            if ok:
                return True
            # FC16 실패 시 FC06 x5로 개별 레지스터 쓰기 시도
            self.log(f"move_relative_modbus_p1 {motor_id} FC06 x5 fallback")
            for i, val in enumerate(regs):
                r = self.client.write_register(
                    address=P1_REGISTER + i, value=val, **{_SLAVE_KW: drv.slave_id}
                )
                if r.isError():
                    self.log(f"move_relative_modbus_p1 FC06 reg[{i}] FAIL: {r}")
                    return False
                time.sleep(CMD_GAP_SEC)
            return True
        except Exception as e:
            self.log(f"move_relative_modbus_p1 {motor_id} error: {e}")
            return False

    def move_relative_xy(self, driver_id: int,
                         x_delta: int, y_delta: int,
                         speed: int = 1000) -> bool:
        """P1 XY 동시 상대좌표 이동"""
        if not self.connected:
            return False
        drv = self.driver1 if driver_id == 1 else self.driver2
        clamped = min(max(1, speed), 8000)
        drv.set_speed(MotorAxis.X, clamped, 1)
        time.sleep(0.02)
        drv.set_speed(MotorAxis.Y, clamped, 1)
        time.sleep(0.02)
        drv.select_speed(MotorAxis.X, 1)
        time.sleep(0.02)
        drv.select_speed(MotorAxis.Y, 1)
        time.sleep(0.02)
        return drv.move_relative_p1(x_delta, y_delta, axis=P1_AXIS_XY)

    def get_motor_type(self, motor_id: str) -> str:
        return self.MOTOR_MAP[motor_id]['type']

    def get_motor_name(self, motor_id: str) -> str:
        return self.MOTOR_MAP[motor_id]['name']

    def get_operating_mode(self, driver_id: int = 1) -> Optional[str]:
        """Func 02: MODE0(10020), MODE1(10021) 읽어 동작 모드 반환 (Jog/Continuous/Index/Program)"""
        if not self.connected or not self.client:
            return None
        drv = self.driver1 if driver_id == 1 else self.driver2
        try:
            # 주소 0x0013=19 (MODE0), 0x0014=20 (MODE1) - 매뉴얼 10020,10021
            r = self.client.read_discrete_inputs(
                address=0x0013, count=2, **{_SLAVE_KW: drv.slave_id}
            )
            if r.isError():
                return None
            m0 = r.bits[0] if r.bits else False
            m1 = r.bits[1] if len(r.bits) > 1 else False
            mode = (1 if m1 else 0) << 1 | (1 if m0 else 0)
            return {0: "Jog", 1: "Continuous", 2: "Index", 3: "Program"}.get(mode, f"Unknown({mode})")
        except Exception as e:
            self.log(f"get_operating_mode error: {e}")
            return None
