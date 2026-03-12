"""
오토닉스 PMC-2HSP 모터 드라이버 통신 모듈
- 드라이버 2개, 모터 4개 (각 드라이버당 X/Y 2축)
- Modbus RTU: 연속운전, 속도/가감속 설정
- P1 전용 프로토콜: 절대/상대 좌표 이동, 직선 보간
- 스텝각: 0.36°, 1회전 1000펄스, 리드 5mm
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


# ==================== 상수 정의 ====================

STEP_ANGLE = 0.72
PULSE_PER_REV = 500
MM_PER_REV = 5
PULSE_PER_MM = 100

CMD_REGISTER = 0x0000

# P1 명령 코드
P1_ABSOLUTE_MOVE = 0x71   # 절대좌표 이동
P1_RELATIVE_MOVE = 0x72   # 상대좌표 이동
P1_INTERPOLATION = 0x73   # 직선 보간

# P1 축 선택
P1_AXIS_X    = 0x01
P1_AXIS_Y    = 0x02
P1_AXIS_XY   = 0x03

X_REGISTERS = {
    'speed_ratio': 0x0454 - 0x0001,
    'accel': 0x0455 - 0x0001,
    'decel': 0x0456 - 0x0001,
    'start_speed': 0x0457 - 0x0001,
    'drive_speed1': 0x0458 - 0x0001,
    'drive_speed2': 0x0459 - 0x0001,
    'drive_speed3': 0x045A - 0x0001,
    'drive_speed4': 0x045B - 0x0001,
    'end_pulse_width': 0x045D - 0x0001,
    'scale_num': 0x045E - 0x0001,
    'scale_den': 0x045F - 0x0001,
    'jerk': 0x0472 - 0x0001,
}

Y_REGISTERS = {
    'speed_ratio': 0x0460 - 0x0001,
    'accel': 0x0461 - 0x0001,
    'decel': 0x0462 - 0x0001,
    'start_speed': 0x0463 - 0x0001,
    'drive_speed1': 0x0464 - 0x0001,
    'drive_speed2': 0x0465 - 0x0001,
    'drive_speed3': 0x0466 - 0x0001,
    'drive_speed4': 0x0467 - 0x0001,
    'post_timer1': 0x0468 - 0x0001,
    'post_timer2': 0x0469 - 0x0001,
    'post_timer3': 0x046A - 0x0001,
    'end_pulse_width': 0x046F - 0x0001,
    'scale_num': 0x0470 - 0x0001,
    'scale_den': 0x0471 - 0x0001,
    'jerk': 0x0473 - 0x0001,
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
            result = self.client.write_register(address=address, value=value, **{_SLAVE_KW: self.slave_id})
            if result.isError():
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
        if speed < 1 or speed > 8000:
            return False
        registers = X_REGISTERS if axis == MotorAxis.X else Y_REGISTERS
        speed_key = f'drive_speed{speed_num}'
        if speed_key not in registers:
            return False
        return self._write_register(registers[speed_key], speed)

    def set_accel(self, axis: MotorAxis, accel: int) -> bool:
        if accel < 1 or accel > 8000:
            return False
        registers = X_REGISTERS if axis == MotorAxis.X else Y_REGISTERS
        return self._write_register(registers['accel'], accel)

    def set_decel(self, axis: MotorAxis, decel: int) -> bool:
        if decel < 1 or decel > 8000:
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
        return self.stop(MotorAxis.X, immediate) and self.stop(MotorAxis.Y, immediate)

    def write_speed_ratio(self, axis: MotorAxis, speed: int) -> bool:
        """PDF 41107(X) / 41113(Y) speed_ratio 레지스터에 속도값 쓰기"""
        regs = X_REGISTERS if axis == MotorAxis.X else Y_REGISTERS
        addr = regs['speed_ratio']
        ok = self._write_register(addr, speed)
        self.log(f"write_speed_ratio: axis={axis.value} speed={speed} "
                 f"addr=0x{addr:04X} → {'OK' if ok else 'FAIL'}")
        return ok

    def move_with_speed(self, axis: MotorAxis, direction: MotorDirection,
                        speed: int) -> bool:
        self.log(f"move_with_speed: axis={axis.value} dir={direction.value} speed={speed}PPS")
        ok = self.start_continuous(axis, direction)
        self.log(f"  start_continuous → {'OK' if ok else 'FAIL'}")
        if ok:
            status = self.x_status if axis == MotorAxis.X else self.y_status
            status.speed = speed
        return ok

    # ── P1 전용 프로토콜 (STX/ETX 프레임) ──

    def _get_raw_serial(self):
        """pymodbus 클라이언트의 하위 serial.Serial 객체 반환"""
        if self.client and hasattr(self.client, 'socket') and self.client.socket:
            return self.client.socket
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
    return int(degree / STEP_ANGLE)

def pulse_to_degree(pulse: int) -> float:
    return pulse * STEP_ANGLE

def speed_pps_to_mm_per_sec(pps: int) -> float:
    return pps / PULSE_PER_MM

def speed_pps_to_deg_per_sec(pps: int) -> float:
    return pps * STEP_ANGLE


class MotorController:
    MOTOR_MAP = {
        'upper_stage':  {'driver': 1, 'axis': MotorAxis.X, 'name': '상부 스테이지', 'type': 'linear'},
        'upper_rotate': {'driver': 1, 'axis': MotorAxis.Y, 'name': '상부 회전',     'type': 'rotate'},
        'lower_stage':  {'driver': 2, 'axis': MotorAxis.X, 'name': '하부 스테이지', 'type': 'linear'},
        'lower_rotate': {'driver': 2, 'axis': MotorAxis.Y, 'name': '하부 회전',     'type': 'rotate'},
    }

    def __init__(self, port: str = '/dev/ttyS1', baudrate: int = 9600,
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
        self.on_log: Optional[Callable] = None

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
                            delay_before_tx=0.0,
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
        for drv in (self.driver1, self.driver2):
            drv.set_pulse_scale(MotorAxis.X, 1, 1)
            drv.set_pulse_scale(MotorAxis.Y, 1, 1)
        self.log("드라이버 초기화 완료 (scale=1/1, PULSE_PER_MM=100)")

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
        """모든 드라이버의 X/Y 축 speed_ratio 레지스터에 속도 쓰기 (PDF 41107)"""
        if not self.connected:
            return False
        ok = True
        for drv in (self.driver1, self.driver2):
            for axis in (MotorAxis.X, MotorAxis.Y):
                if not drv.write_speed_ratio(axis, speed):
                    ok = False
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
        drv, axis = self._get_driver_axis(motor_id)
        ok = drv.move_with_speed(axis, d, speed)
        if ok:
            self.motor_speeds[motor_id] = speed
        return ok

    def stop_motor(self, motor_id: str, immediate: bool = False) -> bool:
        if not self.connected:
            return False
        drv, axis = self._get_driver_axis(motor_id)
        ok = drv.stop(axis, immediate)
        if ok:
            self.motor_speeds[motor_id] = 0
        return ok

    def stop_all(self, immediate: bool = False) -> bool:
        self.driver1.stop_all(immediate)
        self.driver2.stop_all(immediate)
        for k in self.motor_speeds:
            self.motor_speeds[k] = 0
        return True

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
        """P1 상대좌표 이동 (단일 축)"""
        if not self.connected:
            return False
        drv, axis = self._get_driver_axis(motor_id)
        clamped = min(max(1, speed), 8000)
        drv.set_speed(axis, clamped, 1)
        time.sleep(0.03)
        drv.select_speed(axis, 1)
        time.sleep(0.03)
        if axis == MotorAxis.X:
            return drv.move_relative_p1(x_delta=delta_pulse, y_delta=0,
                                        axis=P1_AXIS_X)
        else:
            return drv.move_relative_p1(x_delta=0, y_delta=delta_pulse,
                                        axis=P1_AXIS_Y)

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
