"""
요코가와 UT32A 온도 컨트롤러 통신 모듈
- 모델: UT32A-V10-11-00
- RS-485 Modbus RTU 통신
- 기능코드: 03 (Read), 06/16 (Write)
"""
from pymodbus.client import ModbusSerialClient
from typing import Optional, Dict, Callable
from dataclasses import dataclass
from enum import Enum
import time
import random

# ==================== 레지스터 주소 정의 ====================

# 운전 모니터링 영역 (읽기)
MONITOR_REGISTERS = {
    'PV': 0x0001,           # 현재 측정값
    'SV': 0x0002,           # 설정값
    'MV': 0x0003,           # 출력값 (%)
    'DEV': 0x0004,          # 편차 (PV-SV)
    'RUN_STATUS': 0x0005,   # 운전 상태
    'ALARM_STATUS': 0x0006, # 알람 상태
    'AT_STATUS': 0x0007,    # 오토튜닝 상태
}

# 제어 설정 영역 (읽기/쓰기)
CONTROL_REGISTERS = {
    'SV_SET': 0x0101,       # SV 설정
    'RAMP_UP': 0x0102,      # 램프 상승속도
    'RAMP_DOWN': 0x0103,    # 램프 하강속도
    'OUT_HIGH': 0x0104,     # 출력 상한
    'OUT_LOW': 0x0105,      # 출력 하한
    'MANUAL_OUT': 0x0106,   # 수동 출력값 (MAN 모드)
}

# PID 파라미터
PID_REGISTERS = {
    'P': 0x0201,            # 비례대
    'I': 0x0202,            # 적분시간
    'D': 0x0203,            # 미분시간
    'HYSTERESIS': 0x0204,   # 히스테리시스
    'DEADBAND': 0x0205,     # 데드밴드
}

# 알람 설정
ALARM_REGISTERS = {
    'ALARM1_TYPE': 0x0301,
    'ALARM1_VALUE': 0x0302,
    'ALARM2_TYPE': 0x0303,
    'ALARM2_VALUE': 0x0304,
    'ALARM3_TYPE': 0x0305,
    'ALARM3_VALUE': 0x0306,
}

# 입력 설정
INPUT_REGISTERS = {
    'INPUT_TYPE': 0x0401,   # TC/RTD 등
    'UNIT': 0x0402,         # ℃/℉
    'DECIMAL': 0x0403,      # 소수점 위치
    'FILTER': 0x0404,       # 필터 시간
    'CORRECTION': 0x0405,   # 입력 보정값
}

# 출력 설정
OUTPUT_REGISTERS = {
    'OUTPUT_TYPE': 0x0501,
    'CONTROL_DIR': 0x0502,  # 정/역 방향
    'CYCLE_TIME': 0x0503,
    'SCALE_HIGH': 0x0504,
    'SCALE_LOW': 0x0505,
}

# 통신 설정
COMM_REGISTERS = {
    'SLAVE_ID': 0x0601,
    'BAUD_RATE': 0x0602,
    'PARITY': 0x0603,
    'STOP_BIT': 0x0604,
    'DATA_LENGTH': 0x0605,
}

# 시스템 영역
SYSTEM_REGISTERS = {
    'MODEL_CODE': 0x0701,
    'VERSION': 0x0702,
    'INIT_SETTINGS': 0x0703,
    'MODE_SWITCH': 0x0704,
}


class ControlMode(Enum):
    """제어 모드"""
    AUTO = 0
    MANUAL = 1


class ControlDirection(Enum):
    """제어 방향"""
    DIRECT = 0      # 정방향 (가열)
    REVERSE = 1     # 역방향 (냉각)


class AlarmType(Enum):
    """알람 타입"""
    OFF = 0
    HIGH = 1        # 상한 알람
    LOW = 2         # 하한 알람
    DEVIATION_HIGH = 3
    DEVIATION_LOW = 4
    BAND = 5


class InputType(Enum):
    """입력 타입"""
    TC_K = 0
    TC_J = 1
    TC_T = 2
    TC_E = 3
    TC_R = 4
    TC_S = 5
    TC_B = 6
    TC_N = 7
    RTD_PT100 = 10
    RTD_JPT100 = 11
    V_0_10 = 20
    mA_4_20 = 21


@dataclass
class UT32AData:
    """UT32A 온도 컨트롤러 데이터"""
    pv: float = 0.0             # 현재 온도
    sv: float = 0.0             # 설정 온도
    mv: float = 0.0             # 출력값 (%)
    deviation: float = 0.0      # 편차
    is_running: bool = False    # 운전 상태
    alarm_status: int = 0       # 알람 비트
    at_status: int = 0          # 오토튜닝 상태
    
    # PID 파라미터
    p_value: float = 0.0
    i_value: float = 0.0
    d_value: float = 0.0
    
    # 상태
    connected: bool = False
    error: Optional[str] = None
    unit: str = "℃"
    decimal_point: int = 1      # 소수점 위치 (1 = x0.1, 2 = x0.01)


class UT32AController:
    """
    요코가와 UT32A 온도 컨트롤러 클래스
    """
    
    def __init__(self, slave_id: int = 1, port: str = 'COM8',
                baudrate: int = 9600, simulator: bool = False):

        self.slave_id = slave_id
        self.port = port
        self.baudrate = baudrate
        self.simulator = simulator

        self.client: Optional[ModbusSerialClient] = None
        self.connected = False
        self.data = UT32AData()

        # 스케일
        self.scale_factor = 10.0

        # 🔥 시뮬레이터 상태 변수
        if self.simulator:
            self._sim_pv = 25.0
            self._sim_sv = 25.0
            self._sim_mv = 0.0

        self.on_log: Optional[Callable] = None
        self.on_data_update: Optional[Callable] = None
    
    def log(self, message: str):
        """로그 출력"""
        if self.on_log:
            self.on_log(f"[UT32A-{self.slave_id}] {message}")
        print(f"[UT32A-{self.slave_id}] {message}")
    
    def connect(self, client: Optional[ModbusSerialClient] = None) -> bool:
        """
        Modbus 연결
        - client가 주어지면 공유, 없으면 새로 생성
        """
        #=============시뮬레이션모드====================
        if self.simulator:
            self.connected = True
            self.data.connected = True
            self.log("Simulation Mode 연결 완료")
            return True
        
        #=============실제연결모드=========================
        try:
            if client:
                self.client = client
                self.connected = client.is_socket_open()
            else:
                self.client = ModbusSerialClient(
                    port=self.port,
                    baudrate=self.baudrate,
                    parity='N',
                    stopbits=1,
                    bytesize=8,
                    timeout=1
                )
                self.connected = self.client.connect()
            
            if self.connected:
                self.log(f"연결 성공 ({self.port})")
                self.data.connected = True
                # 소수점 위치 읽기
                self._read_decimal_point()
            else:
                self.log(f"연결 실패 ({self.port})")
            
            return self.connected
            
        except Exception as e:
            self.log(f"연결 오류: {e}")
            return False
    
    def disconnect(self):
        """연결 해제"""
        if self.client and not self._is_shared_client():
            self.client.close()
        self.client = None
        self.connected = False
        self.data.connected = False
        self.log("연결 해제됨")
    
    def _is_shared_client(self) -> bool:
        """공유 클라이언트 여부"""
        # 외부에서 클라이언트를 전달받았으면 닫지 않음
        return hasattr(self, '_shared_client') and self._shared_client
    
    def _read_register(self, address: int) -> Optional[int]:
        """레지스터 읽기 (16bit)"""
        if not self.connected or not self.client:
            return None
        
        try:
            result = self.client.read_holding_registers(
                address=address,
                count=1,
                slave=self.slave_id
            )
            
            if result.isError():
                return None
            
            return result.registers[0]
            
        except Exception as e:
            self.log(f"읽기 오류 (0x{address:04X}): {e}")
            return None
    
    def _write_register(self, address: int, value: int) -> bool:
        """레지스터 쓰기 (16bit)"""
        if not self.connected or not self.client:
            return False
        
        try:
            result = self.client.write_register(
                address=address,
                value=value,
                slave=self.slave_id
            )
            
            if result.isError():
                self.log(f"쓰기 실패 (0x{address:04X})")
                return False
            
            self.log(f"쓰기 성공: 0x{address:04X} = {value}")
            return True
            
        except Exception as e:
            self.log(f"쓰기 오류 (0x{address:04X}): {e}")
            return False
    
    def _convert_to_signed(self, value: int) -> int:
        """부호 없는 16bit를 부호 있는 값으로 변환 (2의 보수)"""
        if value >= 0x8000:
            return value - 0x10000
        return value
    
    def _read_decimal_point(self):
        """소수점 위치 읽기"""
        decimal = self._read_register(INPUT_REGISTERS['DECIMAL'])
        if decimal is not None:
            self.data.decimal_point = decimal
            # 스케일 팩터 계산 (0=x1, 1=x10, 2=x100)
            self.scale_factor = 10 ** decimal
            self.log(f"소수점 위치: {decimal} (스케일: {self.scale_factor})")
    
    # ==================== 읽기 함수 ====================
    
    def read_pv(self) -> Optional[float]:
        """현재 온도(PV) 읽기"""
        raw = self._read_register(MONITOR_REGISTERS['PV'])
        if raw is not None:
            value = self._convert_to_signed(raw) / self.scale_factor
            self.data.pv = value
            return value
        return None
    
    def read_sv(self) -> Optional[float]:
        """설정 온도(SV) 읽기"""
        raw = self._read_register(MONITOR_REGISTERS['SV'])
        if raw is not None:
            value = self._convert_to_signed(raw) / self.scale_factor
            self.data.sv = value
            return value
        return None
    
    def read_mv(self) -> Optional[float]:
        """출력값(MV) 읽기 (%)"""
        raw = self._read_register(MONITOR_REGISTERS['MV'])
        if raw is not None:
            value = raw / 10.0  # MV는 0.1% 단위
            self.data.mv = value
            return value
        return None
    
    def read_deviation(self) -> Optional[float]:
        """편차 읽기"""
        raw = self._read_register(MONITOR_REGISTERS['DEV'])
        if raw is not None:
            value = self._convert_to_signed(raw) / self.scale_factor
            self.data.deviation = value
            return value
        return None
    
    def read_status(self) -> Optional[int]:
        """운전 상태 읽기"""
        status = self._read_register(MONITOR_REGISTERS['RUN_STATUS'])
        if status is not None:
            self.data.is_running = (status & 0x01) != 0
            return status
        return None
    
    def read_alarm_status(self) -> Optional[int]:
        """알람 상태 읽기"""
        alarm = self._read_register(MONITOR_REGISTERS['ALARM_STATUS'])
        if alarm is not None:
            self.data.alarm_status = alarm
            return alarm
        return None
    
    def read_pid(self) -> Dict[str, float]:
        """PID 파라미터 읽기"""
        p = self._read_register(PID_REGISTERS['P'])
        i = self._read_register(PID_REGISTERS['I'])
        d = self._read_register(PID_REGISTERS['D'])
        
        result = {}
        if p is not None:
            self.data.p_value = p / 10.0
            result['P'] = self.data.p_value
        if i is not None:
            self.data.i_value = i
            result['I'] = self.data.i_value
        if d is not None:
            self.data.d_value = d
            result['D'] = self.data.d_value
        
        return result
    
    def read_all(self) -> UT32AData:

        if self.simulator:
            # ===== 1차 시스템 수렴 모델 =====
            error = self._sim_sv - self._sim_pv

            # PV 수렴 (히터 응답 속도)
            self._sim_pv += error * 0.07

            # MV 계산 (단순 P 제어 느낌)
            self._sim_mv = max(0.0, min(100.0, abs(error) * 2))

            # 노이즈 추가
            pv_noise = random.uniform(-0.2, 0.2)

            self.data.pv = round(self._sim_pv + pv_noise, 2)
            self.data.sv = self._sim_sv
            self.data.mv = round(self._sim_mv, 1)
            self.data.deviation = round(self.data.pv - self.data.sv, 2)
            self.data.is_running = True
            self.data.alarm_status = 0
            self.data.connected = True

            if self.on_data_update:
                self.on_data_update(self.data)

            return self.data

        # ===== 실제 장비 모드 =====
        self.read_pv()
        self.read_sv()
        self.read_mv()
        self.read_deviation()
        self.read_status()
        self.read_alarm_status()

        if self.on_data_update:
            self.on_data_update(self.data)

        return self.data
    
    # ==================== 쓰기 함수 ====================
    
    def write_sv(self, temperature: float) -> bool:
#=============시뮬===============
        if self.simulator:
            self._sim_sv = temperature
            self.data.sv = temperature
            self.log(f"[SIM] SV 설정: {temperature}℃")
            return True
#================진짜==================
        """설정 온도(SV) 쓰기"""
        # 스케일 적용
        raw_value = int(temperature * self.scale_factor)
        
        # 음수 처리 (2의 보수)
        if raw_value < 0:
            raw_value = raw_value + 0x10000
        
        success = self._write_register(CONTROL_REGISTERS['SV_SET'], raw_value)
        if success:
            self.data.sv = temperature
            self.log(f"SV 설정: {temperature}℃")
        return success
    
    def write_output_limits(self, high: float, low: float) -> bool:
        """출력 상/하한 설정"""
        high_raw = int(high * 10)  # 0.1% 단위
        low_raw = int(low * 10)
        
        success = self._write_register(CONTROL_REGISTERS['OUT_HIGH'], high_raw)
        success = success and self._write_register(CONTROL_REGISTERS['OUT_LOW'], low_raw)
        
        if success:
            self.log(f"출력 범위: {low}% ~ {high}%")
        return success
    
    def write_ramp_rate(self, up_rate: float, down_rate: float) -> bool:
        """램프 속도 설정 (℃/min)"""
        up_raw = int(up_rate * self.scale_factor)
        down_raw = int(down_rate * self.scale_factor)
        
        success = self._write_register(CONTROL_REGISTERS['RAMP_UP'], up_raw)
        success = success and self._write_register(CONTROL_REGISTERS['RAMP_DOWN'], down_raw)
        
        if success:
            self.log(f"램프 속도: 상승 {up_rate}℃/min, 하강 {down_rate}℃/min")
        return success
    
    def write_pid(self, p: float, i: int, d: int) -> bool:
        """PID 파라미터 설정"""
        p_raw = int(p * 10)  # P는 0.1 단위
        
        success = self._write_register(PID_REGISTERS['P'], p_raw)
        success = success and self._write_register(PID_REGISTERS['I'], i)
        success = success and self._write_register(PID_REGISTERS['D'], d)
        
        if success:
            self.log(f"PID 설정: P={p}, I={i}, D={d}")
            self.data.p_value = p
            self.data.i_value = i
            self.data.d_value = d
        return success
    
    def write_manual_output(self, output: float) -> bool:
        """수동 출력값 설정 (%)"""
        raw = int(output * 10)  # 0.1% 단위
        success = self._write_register(CONTROL_REGISTERS['MANUAL_OUT'], raw)
        if success:
            self.log(f"수동 출력: {output}%")
        return success
    
    def write_alarm(self, alarm_num: int, alarm_type: AlarmType, value: float) -> bool:
        """알람 설정"""
        if alarm_num < 1 or alarm_num > 3:
            self.log(f"잘못된 알람 번호: {alarm_num}")
            return False
        
        type_addr = ALARM_REGISTERS[f'ALARM{alarm_num}_TYPE']
        value_addr = ALARM_REGISTERS[f'ALARM{alarm_num}_VALUE']
        
        value_raw = int(value * self.scale_factor)
        if value_raw < 0:
            value_raw = value_raw + 0x10000
        
        success = self._write_register(type_addr, alarm_type.value)
        success = success and self._write_register(value_addr, value_raw)
        
        if success:
            self.log(f"알람{alarm_num}: {alarm_type.name} = {value}℃")
        return success
    
    # ==================== 유틸리티 ====================
    
    def start_auto_tuning(self) -> bool:
        """오토튜닝 시작"""
        # AT 시작 명령 (모델에 따라 다를 수 있음)
        self.log("오토튜닝 시작")
        return self._write_register(SYSTEM_REGISTERS['MODE_SWITCH'], 0x0001)
    
    def get_alarm_string(self) -> str:
        """알람 상태 문자열"""
        if self.data.alarm_status == 0:
            return "정상"
        
        alarms = []
        if self.data.alarm_status & 0x01:
            alarms.append("AL1")
        if self.data.alarm_status & 0x02:
            alarms.append("AL2")
        if self.data.alarm_status & 0x04:
            alarms.append("AL3")
        
        return ", ".join(alarms) if alarms else "정상"
    
    def is_alarm_active(self) -> bool:
        """알람 활성화 여부"""
        return self.data.alarm_status != 0


# ==================== 통합 컨트롤러 ====================

class TemperatureController:
    """
    다중 UT32A 온도 컨트롤러 통합 관리
    - 여러 대의 UT32A를 하나의 RS-485 라인에서 관리
    """
    
    def __init__(self, port: str = 'COM8',
                baudrate: int = 9600,
                simulator: bool = False):

        self.port = port
        self.baudrate = baudrate
        self.simulator = simulator

        self.client: Optional[ModbusSerialClient] = None
        self.connected = False
        self.controllers: Dict[int, UT32AController] = {}

        self.on_log: Optional[Callable] = None
        
    def log(self, message: str):
        """로그 출력"""
        if self.on_log:
            self.on_log(message)
        print(f"[TempController] {message}")
    
    def add_controller(self, slave_id: int, name: str = "") -> UT32AController:

        controller = UT32AController(
            slave_id=slave_id,
            port=self.port,
            baudrate=self.baudrate,
            simulator=self.simulator   # 🔥 전달
        )

        controller.on_log = self.on_log
        self.controllers[slave_id] = controller
        self.log(f"컨트롤러 추가: Slave ID {slave_id} ({name})")

        return controller
    
    def connect(self) -> bool:
        """모든 컨트롤러 연결"""
#==================시뮬===============
        if self.simulator:
            self.connected = True
            for controller in self.controllers.values():
                controller.connect()
            self.log("Simulation Mode 전체 연결 완료")
            return True
#===================진짜==============
        try:
            self.client = ModbusSerialClient(
                port=self.port,
                baudrate=self.baudrate,
                parity='N',
                stopbits=1,
                bytesize=8,
                timeout=1
            )
            
            if not self.client.connect():
                self.log(f"Modbus 연결 실패: {self.port}")
                return False
            
            self.connected = True
            self.log(f"Modbus 연결 성공: {self.port}")
            
            # 각 컨트롤러에 공유 클라이언트 전달
            for controller in self.controllers.values():
                controller._shared_client = True
                controller.connect(self.client)
            
            return True
            
        except Exception as e:
            self.log(f"연결 오류: {e}")
            return False
    
    def disconnect(self):
        """연결 해제"""
        for controller in self.controllers.values():
            controller.connected = False
            controller.data.connected = False
        
        if self.client:
            self.client.close()
            self.client = None
        
        self.connected = False
        self.log("연결 해제됨")
    
    def get_controller(self, slave_id: int) -> Optional[UT32AController]:
        """특정 컨트롤러 가져오기"""
        return self.controllers.get(slave_id)
    
    def read_all_controllers(self) -> Dict[int, UT32AData]:
        """모든 컨트롤러 데이터 읽기"""
        result = {}
        for slave_id, controller in self.controllers.items():
            result[slave_id] = controller.read_all()
        return result
    
    def write_sv_all(self, temperature: float) -> bool:
        """모든 컨트롤러에 동일한 SV 설정"""
        success = True
        for controller in self.controllers.values():
            if not controller.write_sv(temperature):
                success = False
        return success


# ==================== 테스트 ====================

if __name__ == "__main__":
    print("=== 요코가와 UT32A 온도 컨트롤러 테스트 ===")
    print(f"모델: UT32A-V10-11-00")
    print()
    
    print("=== 레지스터 주소 ===")
    print("모니터링:")
    for name, addr in MONITOR_REGISTERS.items():
        print(f"  {name}: 0x{addr:04X}")
    print()
    
    print("제어 설정:")
    for name, addr in CONTROL_REGISTERS.items():
        print(f"  {name}: 0x{addr:04X}")
    print()
    
    print("PID:")
    for name, addr in PID_REGISTERS.items():
        print(f"  {name}: 0x{addr:04X}")
    print()
    
    # 컨트롤러 생성 테스트
    controller = TemperatureController(port='COM8')
    controller.add_controller(slave_id=1, name="메인 히터")
    print("TemperatureController 생성 완료")

