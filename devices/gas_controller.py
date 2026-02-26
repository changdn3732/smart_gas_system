"""
MFC / BPR 가스 제어기 통신 모듈
- Modbus RTU 통신
- Slave ID: 5 (MFC), 6 (BPR)
- Alicat 장비 기반 레지스터 맵
"""
from pymodbus.client import ModbusSerialClient
from typing import Optional, Dict, Callable
from dataclasses import dataclass
from enum import Enum
import struct
import time
import random

# ==================== 레지스터 주소 정의 ====================
# New Firmware (10v07+) — Input Register로 읽기 (FC04)

REGISTERS = {
    # 읽기용 (Input Register, FC04)
    'GAS_NUMBER': 1346,         # uint16 — 현재 가스 번호
    'SETPOINT': 1349,           # float32 (2 registers)
    'VALVE_DRIVE': 1351,        # float32
    'PRESSURE': 1353,           # float32
    'TEMPERATURE': 1359,        # float32
    'VOLUMETRIC_FLOW': 1361,    # float32
    'MASS_FLOW': 1363,          # float32

    # 쓰기용 (Holding Register, FC06/16)
    'GAS_SELECT': 1004,         # 가스 선택 쓰기 주소
    'SETPOINT_WRITE': 1349,     # Setpoint 쓰기
}

# Alicat 가스 테이블 (ID, 약어, 전체 이름)
ALICAT_GAS_LIST = [
    (0, "Air", "Air (Clean Dry)"),
    (1, "Ar", "Argon"),
    (2, "CH4", "Methane"),
    (3, "CO", "Carbon Monoxide"),
    (4, "CO2", "Carbon Dioxide"),
    (5, "C2H6", "Ethane"),
    (6, "H2", "Hydrogen"),
    (7, "He", "Helium"),
    (8, "N2", "Nitrogen"),
    (9, "N2O", "Nitrous Oxide"),
    (10, "Ne", "Neon"),
    (11, "O2", "Oxygen"),
    (12, "C3H8", "Propane"),
    (13, "nC4H10", "Normal Butane"),
    (14, "C2H2", "Acetylene"),
    (15, "C2H4", "Ethylene"),
    (16, "iC4H10", "Isobutane"),
    (17, "Kr", "Krypton"),
    (18, "Xe", "Xenon"),
    (19, "SF6", "Sulfur Hexafluoride"),
    (20, "C-25", "25% CO2, 75% Ar"),
    (21, "C-10", "10% CO2, 90% Ar"),
    (22, "C-8", "8% CO2, 92% Ar"),
    (23, "C-2", "2% CO2, 98% Ar"),
    (24, "C-75", "75% CO2, 25% Ar"),
    (25, "He-25", "25% He, 75% Ar"),
    (26, "He-75", "75% He, 25% Ar"),
    (27, "AT105", "90% He, 7.5% Ar, 2.5% CO2"),
    (28, "Star29", "90% Ar, 8% CO2, 2% O2"),
    (29, "P-5", "5% CH4, 95% Ar"),
]

# 가스 번호 → 가스 이름 매핑
GAS_TABLE = {g[0]: g[1] for g in ALICAT_GAS_LIST}

# 단위 코드 매핑
UNIT_CODES = {
    # Setpoint/Flow units
    0: "SCCM",
    1: "SLPM",
    2: "SCFH",
    3: "SCFM",
    # Pressure units
    10: "Pa",
    11: "kPa",
    12: "bar",
    13: "mbar",
    14: "psi",
    15: "atm",
    # Temperature units
    20: "°C",
    21: "K",
    22: "°F",
}


class DeviceType(Enum):
    """장치 타입"""
    MFC = "MFC"     # Mass Flow Controller
    BPR = "BPR"     # Back Pressure Regulator
    BASIS = "BASIS" # Gauge


@dataclass
class GasDeviceData:
    """가스 장치 데이터"""
    pressure: float = 0.0
    temperature: float = 0.0
    setpoint: float = 0.0
    gas_index: int = 0
    gas_name: str = ""
    pressure_unit: str = ""
    temperature_unit: str = ""
    setpoint_unit: str = ""
    connected: bool = False
    error: Optional[str] = None


class GasDeviceReader:
    """
    가스 장치 통신 클래스 (MFC / BPR / BASIS)
    """
    
    def __init__(self, slave_id: int, device_type: DeviceType = DeviceType.MFC, simulator: bool = False):
        self.slave_id = slave_id
        self.device_type = device_type
        self.simulator = simulator
        self.client: Optional[ModbusSerialClient] = None
        self.connected = simulator  # 시뮬레이터면 연결된 것으로 처리
        self.data = GasDeviceData()
        
        # 시뮬레이션용 내부 상태
        self._sim_pressure = 0.0
        self._sim_temperature = 25.0
        self._sim_setpoint = 0.0
        
        self.on_log: Optional[Callable] = None
    
    def log(self, message: str):
        """로그 출력"""
        if self.on_log:
            self.on_log(f"[Gas {self.slave_id}] {message}")
        print(f"[Gas {self.slave_id}] {message}")
    
    def connect(self, client: ModbusSerialClient) -> bool:
        """공유 클라이언트로 연결"""
        self.client = client
        self.connected = client.is_socket_open() if client else False
        self.data.connected = self.connected
        return self.connected
    
    def disconnect(self):
        """연결 해제 (공유 클라이언트이므로 실제 닫지 않음)"""
        self.connected = False
        self.data.connected = False
    
    # ==================== 읽기 함수 (Input Register, FC04) ====================

    def _read_float_input(self, address: int) -> Optional[float]:
        """Input Register에서 Float32 읽기 (Big Endian)"""
        if not self.connected or not self.client:
            return None

        try:
            result = self.client.read_input_registers(
                address=address,
                count=2,
                device_id=self.slave_id
            )

            if result.isError():
                return None

            raw = struct.pack('>HH', result.registers[0], result.registers[1])
            return struct.unpack('>f', raw)[0]

        except Exception as e:
            self.log(f"Float 읽기 오류 ({address}): {e}")
            return None

    def _read_uint16_input(self, address: int) -> Optional[int]:
        """Input Register에서 uint16 읽기"""
        if not self.connected or not self.client:
            return None

        try:
            result = self.client.read_input_registers(
                address=address,
                count=1,
                device_id=self.slave_id
            )

            if result.isError():
                return None

            return result.registers[0]

        except Exception as e:
            self.log(f"uint16 읽기 오류 ({address}): {e}")
            return None

    def read_pressure(self) -> Optional[float]:
        return self._read_float_input(REGISTERS['PRESSURE'])

    def read_temperature(self) -> Optional[float]:
        return self._read_float_input(REGISTERS['TEMPERATURE'])

    def read_setpoint(self) -> Optional[float]:
        if self.device_type == DeviceType.BASIS:
            return None
        return self._read_float_input(REGISTERS['SETPOINT'])

    def read_mass_flow(self) -> Optional[float]:
        return self._read_float_input(REGISTERS['MASS_FLOW'])

    def read_valve_drive(self) -> Optional[float]:
        return self._read_float_input(REGISTERS['VALVE_DRIVE'])

    def read_gas_index(self) -> Optional[int]:
        if self.device_type != DeviceType.MFC:
            return None
        return self._read_uint16_input(REGISTERS['GAS_NUMBER'])
    
    def read_all(self) -> GasDeviceData:
        """모든 데이터 읽기"""
        
        # ================== 시뮬레이션 모드 ==================
        if self.simulator:
            # Setpoint 기준으로 압력 수렴
            self._sim_pressure += (self._sim_setpoint - self._sim_pressure) * 0.15
            pressure_noise = random.uniform(-0.5, 0.5)
            
            # 온도는 천천히 변화
            self._sim_temperature += random.uniform(-0.1, 0.1)
            
            self.data.pressure = round(self._sim_pressure + pressure_noise, 2)
            self.data.temperature = round(self._sim_temperature, 2)
            self.data.setpoint = self._sim_setpoint
            
            # 기본값
            self.data.gas_index = 8
            self.data.gas_name = "N2"
            self.data.pressure_unit = "kPa"
            self.data.temperature_unit = "°C"
            self.data.setpoint_unit = "SCCM"
            self.data.connected = True
            
            return self.data
        
        # ================== 실제 장비 모드 ==================
        self.data.pressure = self.read_pressure() or 0.0
        self.data.temperature = self.read_temperature() or 0.0
        self.data.setpoint = self.read_setpoint() or 0.0

        gas_idx = self.read_gas_index()
        if gas_idx is not None:
            self.data.gas_index = gas_idx
            self.data.gas_name = GAS_TABLE.get(gas_idx, f"Gas #{gas_idx}")

        self.data.connected = True
        return self.data
    # ==================== 쓰기 함수 ====================
    # no_response_expected=True: RS-232 RX 미연결 환경에서도 쓰기 가능

    def _write_float(self, address: int, value: float) -> bool:
        """Holding Register에 Float32 쓰기 (Big Endian)"""
        if not self.client:
            self.log("쓰기 실패: client 없음")
            return False
        if not self.connected:
            self.log("쓰기 실패: connected=False")
            return False

        try:
            raw = struct.pack('>f', value)
            registers = list(struct.unpack('>HH', raw))

            self.log(f"쓰기 시도: addr={address}, val={value}, "
                     f"regs={registers}, device_id={self.slave_id}, "
                     f"socket_open={self.client.connected}")

            self.client.write_registers(
                address=address,
                values=registers,
                device_id=self.slave_id,
                no_response_expected=True,
            )
            self.log(f"쓰기 완료: addr={address}, val={value}")
            return True

        except Exception as e:
            self.log(f"Float 쓰기 오류 ({address}): {e}")
            return False

    def _write_multi_registers(self, address: int, values: list) -> bool:
        """Holding Register에 다중 값 쓰기"""
        if not self.connected or not self.client:
            return False

        try:
            self.client.write_registers(
                address=address,
                values=values,
                device_id=self.slave_id,
                no_response_expected=True,
            )
            return True

        except Exception as e:
            self.log(f"Multi write 오류 ({address}): {e}")
            return False
    
    def write_setpoint(self, value: float) -> bool:
        if self.simulator:
            self._sim_setpoint = value
            self.data.setpoint = value
            return True

        if self.device_type == DeviceType.BASIS:
            return False

        success = self._write_float(REGISTERS['SETPOINT_WRITE'], value)
        if success:
            self.data.setpoint = value
        return success

    def write_gas(self, gas_index: int) -> bool:
        """Gas 변경 (MFC만, 2-step: 1002에 1 → 1004에 gas_id)"""
        if self.device_type != DeviceType.MFC:
            self.log("MFC만 Gas 변경 지원")
            return False

        if not self.connected or not self.client:
            return False

        # Step 1: 주소 1002에 값 1 (가스 변경 활성화)
        ok1 = self._write_multi_registers(1002, [1, 0])
        if not ok1:
            self.log("Gas enable (1002) 실패")
            return False

        # Step 2: 주소 1004에 gas_id
        ok2 = self._write_multi_registers(REGISTERS['GAS_SELECT'], [gas_index, 0])
        if not ok2:
            self.log("Gas write (1004) 실패")
            return False

        gas_name = GAS_TABLE.get(gas_index, f"Gas #{gas_index}")
        self.log(f"Gas → {gas_index} ({gas_name})")
        self.data.gas_index = gas_index
        self.data.gas_name = gas_name
        return True
    
    def set_valve_open(self) -> bool:
        """밸브 열기 (Setpoint 최대값 설정)"""
        return self.write_setpoint(10000.0)  # 최대값
    
    def set_valve_close(self) -> bool:
        """밸브 닫기 (Setpoint 0 설정)"""
        return self.write_setpoint(0.0)


# ==================== 통합 컨트롤러 ====================

class GasController:
    """
    4 Slave 독립 구조 가스 컨트롤러
    """

    def __init__(
        self,
        port: str = "COM7",
        baudrate: int = 19200,
        slave_ids=None,
        simulator: bool = False,
    ):
        self.port = port
        self.baudrate = baudrate
        self.simulator = simulator

        self.client: Optional[ModbusSerialClient] = None
        self.connected = simulator

        # 기본 slave 1~4
        if slave_ids is None:
            slave_ids = [1, 2, 3, 4]

        self.devices: Dict[int, GasDeviceReader] = {}

        for sid in slave_ids:
            self.devices[sid] = GasDeviceReader(
                slave_id=sid,
                device_type=DeviceType.MFC,
                simulator=simulator,
            )

    # ---------------- 연결 ----------------

    def connect(self) -> bool:
        if self.simulator:
            self.connected = True
            print("Simulation Mode 연결 완료")
            return True

        try:
            self.client = ModbusSerialClient(
                port=self.port,
                baudrate=self.baudrate,
                parity="N",
                stopbits=1,
                bytesize=8,
                timeout=1,
            )

            if not self.client.connect():
                print("Modbus 연결 실패")
                return False

            self.connected = True
            print("Modbus 연결 성공")

            # 각 slave 연결 확인
            for sid, dev in self.devices.items():
                dev.connect(self.client)

            return True

        except Exception as e:
            print("연결 오류:", e)
            return False

    def disconnect(self):
        for dev in self.devices.values():
            dev.disconnect()

        if self.client:
            self.client.close()
            self.client = None

        self.connected = False
        print("연결 해제")

    # ---------------- 데이터 ----------------

    def read_all_devices(self) -> Dict[int, GasDeviceData]:
        result = {}
        for sid, device in self.devices.items():
            result[sid] = device.read_all()
        return result

    def get_device(self, slave_id: int) -> Optional[GasDeviceReader]:
        return self.devices.get(slave_id)

    def write_setpoint(self, slave_id: int, value: float) -> bool:
        device = self.get_device(slave_id)
        if not device:
            return False
        return device.write_setpoint(value)

    def write_gas(self, slave_id: int, gas_index: int) -> bool:
        device = self.get_device(slave_id)
        if not device:
            return False
        return device.write_gas(gas_index)

# ==================== 테스트 ====================

if __name__ == "__main__":
    print("=== 가스 제어기 테스트 ===")
    print(f"레지스터 주소:")
    for name, addr in REGISTERS.items():
        print(f"  {name}: {addr}")
    print()
    
    print(f"가스 테이블 (처음 10개):")
    for gas_id, gas_short, gas_full in ALICAT_GAS_LIST[:10]:
        print(f"  {gas_id}: {gas_short} - {gas_full}")
    print()
    
    # 컨트롤러 생성
    controller = GasController(port='COM7')
    print("GasController 생성 완료")

