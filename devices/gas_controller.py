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

REGISTERS = {
    # 실시간 출력용
    'PRESSURE': 1354,           # float32 (2 registers)
    'PRESSURE_UNIT': 1654,      # uint16
    'PRESSURE_DECIMAL': 1655,   # uint16
    
    'TEMPERATURE': 1360,        # float32 (2 registers)
    'TEMPERATURE_UNIT': 1660,   # uint16
    'TEMPERATURE_DECIMAL': 1661, # uint16
    
    'SETPOINT': 1350,           # float32 (2 registers)
    'SETPOINT_UNIT': 1650,      # uint16
    'SETPOINT_DECIMAL': 1651,   # uint16
    
    # 제어용
    'GAS_INDEX': 1083,          # uint16
    'GAS_NAME': 1084,           # ASCII (16 registers)
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
    
    # ==================== 읽기 함수 ====================
    
    def _read_float(self, address: int) -> Optional[float]:
        """Holding Register에서 Float32 읽기 (Big Endian)"""
        if not self.connected or not self.client:
            return None
        
        try:
            result = self.client.read_holding_registers(
                address=address,
                count=2,
                slave=self.slave_id
            )
            
            if result.isError():
                return None
            
            # Big Endian 변환
            raw = struct.pack('>HH', result.registers[0], result.registers[1])
            return struct.unpack('>f', raw)[0]
            
        except Exception as e:
            self.log(f"Float 읽기 오류: {e}")
            return None
    
    def _read_uint16(self, address: int) -> Optional[int]:
        """Holding Register에서 uint16 읽기"""
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
            self.log(f"uint16 읽기 오류: {e}")
            return None
    
    def read_pressure(self) -> Optional[float]:
        """압력 읽기"""
        return self._read_float(REGISTERS['PRESSURE'])
    
    def read_temperature(self) -> Optional[float]:
        """온도 읽기"""
        return self._read_float(REGISTERS['TEMPERATURE'])
    
    def read_setpoint(self) -> Optional[float]:
        """Setpoint 읽기"""
        if self.device_type == DeviceType.BASIS:
            return None  # BASIS는 Setpoint 없음
        return self._read_float(REGISTERS['SETPOINT'])
    
    def read_gas_index(self) -> Optional[int]:
        """Gas 인덱스 읽기"""
        if self.device_type != DeviceType.MFC:
            return None  # MFC만 지원
        return self._read_uint16(REGISTERS['GAS_INDEX'])
    
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
        
        pressure_unit = self._read_uint16(REGISTERS['PRESSURE_UNIT'])
        if pressure_unit is not None:
            self.data.pressure_unit = UNIT_CODES.get(pressure_unit, f"Unit {pressure_unit}")
        
        temp_unit = self._read_uint16(REGISTERS['TEMPERATURE_UNIT'])
        if temp_unit is not None:
            self.data.temperature_unit = UNIT_CODES.get(temp_unit, f"Unit {temp_unit}")
        
        sp_unit = self._read_uint16(REGISTERS['SETPOINT_UNIT'])
        if sp_unit is not None:
            self.data.setpoint_unit = UNIT_CODES.get(sp_unit, f"Unit {sp_unit}")
        
        return self.data
    # ==================== 쓰기 함수 ====================
    
    def _write_float(self, address: int, value: float) -> bool:
        """Holding Register에 Float32 쓰기 (Big Endian)"""
        if not self.connected or not self.client:
            return False
        
        try:
            raw = struct.pack('>f', value)
            registers = struct.unpack('>HH', raw)
            
            result = self.client.write_registers(
                address=address,
                values=list(registers),
                slave=self.slave_id
            )
            
            if result.isError():
                self.log(f"Float 쓰기 실패: {address}")
                return False
            
            return True
            
        except Exception as e:
            self.log(f"Float 쓰기 오류: {e}")
            return False
    
    def _write_uint16(self, address: int, value: int) -> bool:
        """Holding Register에 uint16 쓰기"""
        if not self.connected or not self.client:
            return False
        
        try:
            result = self.client.write_register(
                address=address,
                value=value,
                slave=self.slave_id
            )
            
            if result.isError():
                self.log(f"uint16 쓰기 실패: {address}")
                return False
            
            return True
            
        except Exception as e:
            self.log(f"uint16 쓰기 오류: {e}")
            return False
    
    def write_setpoint(self, value: float) -> bool:
        if self.simulator:
            self._sim_setpoint = value
            self.data.setpoint = value
            return True

        if self.device_type == DeviceType.BASIS:
            return False

        success = self._write_float(REGISTERS['SETPOINT'], value)
        if success:
            self.data.setpoint = value
        return success
    
    def write_gas(self, gas_index: int) -> bool:
        """Gas 변경 (MFC만)"""
        if self.device_type != DeviceType.MFC:
            self.log("MFC만 Gas 변경 지원")
            return False
        
        success = self._write_uint16(REGISTERS['GAS_INDEX'], gas_index)
        if success:
            gas_name = GAS_TABLE.get(gas_index, f"Gas #{gas_index}")
            self.log(f"Gas → {gas_index} ({gas_name})")
            self.data.gas_index = gas_index
            self.data.gas_name = gas_name
        return success
    
    def write_unit(self, unit_type: str, unit_code: int) -> bool:
        """단위 변경"""
        unit_registers = {
            'setpoint': REGISTERS['SETPOINT_UNIT'],
            'pressure': REGISTERS['PRESSURE_UNIT'],
        }
        
        if unit_type not in unit_registers:
            return False
        
        return self._write_uint16(unit_registers[unit_type], unit_code)
    
    def set_valve_open(self) -> bool:
        """밸브 열기 (Setpoint 최대값 설정)"""
        return self.write_setpoint(10000.0)  # 최대값
    
    def set_valve_close(self) -> bool:
        """밸브 닫기 (Setpoint 0 설정)"""
        return self.write_setpoint(0.0)


# ==================== 통합 컨트롤러 ====================

class GasController:
    """
    가스 장치 통합 컨트롤러
    - Slave ID 5: MFC (Mass Flow Controller)
    - Slave ID 6: BPR (Back Pressure Regulator)
    """
    
    def __init__(self, port: str = 'COM7', baudrate: int = 19200, simulator: bool = False):
        self.port = port
        self.baudrate = baudrate
        self.simulator = simulator
        
        self.client: Optional[ModbusSerialClient] = None
        self.connected = simulator
        
        self.mfc = GasDeviceReader(
            slave_id=5,
            device_type=DeviceType.MFC,
            simulator=simulator
        )
        
        self.bpr = GasDeviceReader(
            slave_id=6,
            device_type=DeviceType.BPR,
            simulator=simulator
        )
        
        self.devices = {
            'mfc': self.mfc,
            'bpr': self.bpr,
        }
        
    def log(self, message: str):
        """로그 출력"""
        if self.on_log:
            self.on_log(message)
        print(f"[GasController] {message}")
    
    def connect(self) -> bool:
        """모든 장치 연결"""
    # ================== 시뮬레이션 모드 ==================
        if self.simulator:
            self.connected = True
            self.log("Simulation Mode 연결 완료")
            return True
        
        
    # ================== 실제 장비 모드 ==================
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
            
            # 장치에 공유 클라이언트 전달
            self.mfc.connect(self.client)
            self.bpr.connect(self.client)
            
            return True
            
        except Exception as e:
            self.log(f"연결 오류: {e}")
            return False
    
    def disconnect(self):
        """연결 해제"""
        self.mfc.disconnect()
        self.bpr.disconnect()
        
        if self.client:
            self.client.close()
            self.client = None
        
        self.connected = False
        self.log("연결 해제됨")
    
    def read_all_devices(self) -> Dict[str, GasDeviceData]:
        """모든 장치 데이터 읽기"""
        result = {}
        for device_id, device in self.devices.items():
            result[device_id] = device.read_all()
        return result
    
    def get_device(self, device_id: str) -> Optional[GasDeviceReader]:
        """장치 인스턴스 가져오기"""
        return self.devices.get(device_id)
    
    def set_valve(self, device_id: str, is_open: bool) -> bool:
        """밸브 열기/닫기"""
        device = self.get_device(device_id)
        if not device:
            return False
        
        if is_open:
            return device.set_valve_open()
        else:
            return device.set_valve_close()
    
    def write_setpoint(self, device_id: str, value: float) -> bool:
        """Setpoint 쓰기"""
        device = self.get_device(device_id)
        if not device:
            return False
        return device.write_setpoint(value)
    
    def write_gas(self, device_id: str, gas_index: int) -> bool:
        """Gas 변경"""
        device = self.get_device(device_id)
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

