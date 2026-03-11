"""
오토닉스 PMC-2HSP 모터 드라이버 통신 모듈
- 드라이버 2개, 모터 4개 (각 드라이버당 X/Y 2축)
- Modbus RTU 통신
- 스텝각: 0.36°, 1회전 1000펄스, 리드 5mm
"""
from pymodbus.client import ModbusSerialClient
from typing import Optional, Dict, Callable
from dataclasses import dataclass
from enum import Enum
import threading
import time


# ==================== 상수 정의 ====================

STEP_ANGLE = 0.36
PULSE_PER_REV = 1000
MM_PER_REV = 5
PULSE_PER_MM = 200

CMD_REGISTER = 0x0000

# 절대좌표 위치 결정 레지스터 (매뉴얼 주소 - 1 = 0-based)
POS_REGISTERS = {
    'present_pos':  1000 - 1,   # INT32, 현재 절대좌표 (펄스)
    'target_pos':   1002 - 1,   # INT32, 목표 절대좌표
    'pos_speed':    1004 - 1,   # INT32, 이동 속도
    'pos_accel':    1006 - 1,   # INT32, 가속도
    'pos_decel':    1008 - 1,   # INT32, 감속도
    'motion_start': 1010 - 1,   # INT16, 1 입력 시 이동 시작
    'motion_stop':  1011 - 1,   # INT16, 1 입력 시 정지
    'alarm_status': 1012 - 1,   # INT16, 알람 상태
    'servo_on':     1013 - 1,   # INT16, 모터 enable
    'home_start':   1014 - 1,   # INT16, 원점 복귀 시작
}

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
    def __init__(self, slave_id: int = 1, port: str = 'COM7', baudrate: int = 9600):
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
            result = self.client.write_register(CMD_REGISTER, value, slave=self.slave_id)
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
            result = self.client.write_register(address, value, slave=self.slave_id)
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
            result = self.client.read_holding_registers(address, 1, slave=self.slave_id)
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

    def move_with_speed(self, axis: MotorAxis, direction: MotorDirection, speed: int) -> bool:
        if not self.set_speed(axis, speed, 1):
            return False
        time.sleep(0.05)
        if not self.select_speed(axis, 1):
            return False
        time.sleep(0.05)
        ok = self.start_continuous(axis, direction)
        if ok:
            status = self.x_status if axis == MotorAxis.X else self.y_status
            status.speed = speed
        return ok

    # ── 절대좌표 위치 결정 운전 ──

    def _read_int32(self, address: int) -> Optional[int]:
        if not self.connected or not self.client:
            return None
        try:
            result = self.client.read_holding_registers(address, 2, slave=self.slave_id)
            if result.isError():
                return None
            hi, lo = result.registers[0], result.registers[1]
            value = (hi << 16) | lo
            if value >= 0x80000000:
                value -= 0x100000000
            return value
        except Exception as e:
            self.log(f"INT32 읽기 오류 @{address}: {e}")
            return None

    def _write_int32(self, address: int, value: int) -> bool:
        if not self.connected or not self.client:
            return False
        try:
            if value < 0:
                value += 0x100000000
            hi = (value >> 16) & 0xFFFF
            lo = value & 0xFFFF
            r1 = self.client.write_register(address, hi, slave=self.slave_id)
            r2 = self.client.write_register(address + 1, lo, slave=self.slave_id)
            return not r1.isError() and not r2.isError()
        except Exception as e:
            self.log(f"INT32 쓰기 오류 @{address}: {e}")
            return False

    def read_present_position(self) -> Optional[int]:
        return self._read_int32(POS_REGISTERS['present_pos'])

    def move_to_position(self, target_pulse: int, speed: int = 1000,
                         accel: int = 500, decel: int = 500) -> bool:
        ok = self._write_int32(POS_REGISTERS['target_pos'], target_pulse)
        time.sleep(0.02)
        ok = ok and self._write_int32(POS_REGISTERS['pos_speed'], speed)
        time.sleep(0.02)
        ok = ok and self._write_int32(POS_REGISTERS['pos_accel'], accel)
        time.sleep(0.02)
        ok = ok and self._write_int32(POS_REGISTERS['pos_decel'], decel)
        time.sleep(0.02)
        if ok:
            ok = self._write_register(POS_REGISTERS['motion_start'], 1)
            self.log(f"절대좌표 이동: target={target_pulse}, speed={speed}")
        return ok

    def stop_positioning(self) -> bool:
        return self._write_register(POS_REGISTERS['motion_stop'], 1)

    def set_servo(self, on: bool) -> bool:
        return self._write_register(POS_REGISTERS['servo_on'], 1 if on else 0)

    def start_home_return(self) -> bool:
        self.log("원점 복귀 시작")
        return self._write_register(POS_REGISTERS['home_start'], 1)

    def read_alarm(self) -> Optional[int]:
        return self._read_register(POS_REGISTERS['alarm_status'])


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

    def __init__(self, port: str = 'COM7', baudrate: int = 9600):
        self.port = port
        self.baudrate = baudrate
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
        try:
            self.client = ModbusSerialClient(
                port=self.port, baudrate=self.baudrate,
                parity='N', stopbits=1, bytesize=8, timeout=1,
            )
            if not self.client.connect():
                self.log(f"Modbus 연결 실패: {self.port}")
                return False
            self.connected = True
            self.log(f"Modbus 연결 성공: {self.port}")
            self.driver1.connect(self.client)
            self.driver2.connect(self.client)
            self._initialize_drivers()
            return True
        except Exception as e:
            self.log(f"연결 오류: {e}")
            return False

    def disconnect(self):
        self.driver1.disconnect()
        self.driver2.disconnect()
        if self.client:
            self.client.close()
            self.client = None
        self.connected = False

    def _initialize_drivers(self):
        for drv in (self.driver1, self.driver2):
            drv.set_pulse_scale(MotorAxis.X, 1, 200)
            drv.set_pulse_scale(MotorAxis.Y, 1, 200)
        self.log("드라이버 초기화 완료")

    def _get_driver_axis(self, motor_id: str):
        cfg = self.MOTOR_MAP[motor_id]
        drv = self.driver1 if cfg['driver'] == 1 else self.driver2
        return drv, cfg['axis']

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

    def read_position(self, motor_id: str) -> Optional[int]:
        if not self.connected:
            return None
        drv, axis = self._get_driver_axis(motor_id)
        return drv.read_present_position()

    def move_absolute(self, motor_id: str, target_pulse: int,
                      speed: int = 1000, accel: int = 500, decel: int = 500) -> bool:
        if not self.connected:
            return False
        drv, axis = self._get_driver_axis(motor_id)
        return drv.move_to_position(target_pulse, speed, accel, decel)

    def home_return(self, motor_id: str) -> bool:
        if not self.connected:
            return False
        drv, axis = self._get_driver_axis(motor_id)
        return drv.start_home_return()

    def get_motor_type(self, motor_id: str) -> str:
        return self.MOTOR_MAP[motor_id]['type']

    def get_motor_name(self, motor_id: str) -> str:
        return self.MOTOR_MAP[motor_id]['name']
