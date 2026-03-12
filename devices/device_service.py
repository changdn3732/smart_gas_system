from devices.gas_controller import GasDeviceReader, DeviceType
from devices.temperature_controller import TemperatureController
try:
    from pymodbus.client import ModbusSerialClient
except ImportError:
    from pymodbus.client.sync import ModbusSerialClient
import struct

_SLAVE_KW = 'unit'


class DeviceService:
    def __init__(self):
        self.mode = "simulator"
        self.temp_client = None
        self.gas_client = None
        self.gas_devices: list[GasDeviceReader] = []
        self.temp_device = None
        self.connection_status = {
            "temperature": False,
            "gas": [False, False, False, False],
        }
        self.gas_readable = [False, False, False, False]

        self.connect_simulator()

    # =====================================================
    # Simulator 모드
    # =====================================================
    def connect_simulator(self):
        self.mode = "simulator"

        self.gas_devices = [
            GasDeviceReader(slave_id=i + 1, device_type=DeviceType.MFC, simulator=True)
            for i in range(4)
        ]

        self.temp_device = TemperatureController(simulator=True)
        self.temp_device.add_controller(slave_id=1, name="Simulator Heater")
        self.temp_device.connect()

        self.connection_status = {
            "temperature": True,
            "gas": [True, True, True, True],
        }
        self.gas_readable = [True, True, True, True]

    # =====================================================
    # Serial 모드 (RS-485 + RS-232 분리)
    # =====================================================
    def connect_serial(self, temp_port, temp_baud, gas_port, gas_baud):
        self.mode = "serial"
        self.disconnect_all()

        # Temperature 포트 (RS-485)
        if temp_port:
            try:
                self.temp_client = ModbusSerialClient(
                    port=temp_port, baudrate=temp_baud,
                    parity="N", stopbits=1, bytesize=8,
                    timeout=1, retries=1,
                )
                if self.temp_client.connect():
                    print(f"Temp port {temp_port} 열기 성공")
                    self.temp_device = TemperatureController(
                        client=self.temp_client, simulator=False
                    )
                    self.temp_device.add_controller(slave_id=1, name="Main Heater")
                    self.temp_device.connect()
                else:
                    print(f"Temp port {temp_port} 열기 실패")
                    self.temp_client = None
            except Exception as e:
                print(f"Temp port 오류: {e}")
                self.temp_client = None

        # Gas 포트 (RS-232)
        if gas_port:
            if gas_port == temp_port and self.temp_client:
                self.gas_client = self.temp_client
                print(f"Gas: Temp 포트({gas_port}) 공유")
            else:
                try:
                    self.gas_client = ModbusSerialClient(
                        port=gas_port, baudrate=gas_baud,
                        parity="N", stopbits=1, bytesize=8,
                        timeout=0.5, retries=0,
                    )
                    if self.gas_client.connect():
                        print(f"Gas port {gas_port} 열기 성공")
                    else:
                        print(f"Gas port {gas_port} 열기 실패")
                        self.gas_client = None
                except Exception as e:
                    print(f"Gas port 오류: {e}")
                    self.gas_client = None

        # Gas 디바이스 오브젝트 생성 (device_id 2~5)
        self.gas_devices = []
        for sid in range(2, 6):
            reader = GasDeviceReader(slave_id=sid, device_type=DeviceType.MFC, simulator=False)
            if self.gas_client:
                reader.connect(self.gas_client)
            self.gas_devices.append(reader)

        return True

    # =====================================================
    # 연결 해제
    # =====================================================
    def disconnect_all(self):
        if self.gas_client and self.gas_client is not self.temp_client:
            try:
                self.gas_client.close()
            except Exception:
                pass
        self.gas_client = None

        if self.temp_client:
            try:
                self.temp_client.close()
            except Exception:
                pass
        self.temp_client = None

        self.temp_device = None
        self.gas_devices = []
        self.connection_status = {
            "temperature": False,
            "gas": [False, False, False, False],
        }
        self.gas_readable = [False, False, False, False]

    # =====================================================
    # 연결 상태 확인
    # =====================================================
    def check_connections(self):
        result = {
            "temperature": False,
            "gas": [False, False, False, False],
        }
        self.gas_readable = [False, False, False, False]

        if self.mode == "simulator":
            result["temperature"] = True
            result["gas"] = [True, True, True, True]
            self.gas_readable = [True, True, True, True]
            self.connection_status = result
            return result

        # UT32A (device_id=1)
        if self.temp_client:
            try:
                resp = self.temp_client.read_holding_registers(
                    address=0x0002, count=1, **{_SLAVE_KW: 1}
                )
                if not resp.isError():
                    result["temperature"] = True
                    pv_raw = resp.registers[0]
                    print(f"[UT32A] PV raw = {pv_raw} (온도 = {pv_raw / 10.0}°C)")
                else:
                    print(f"[UT32A] 읽기 응답 에러: {resp}")
                    result["temperature"] = False
            except Exception as e:
                print(f"[UT32A] 읽기 예외: {e}")
                result["temperature"] = False

        # Gas: 읽기를 시도하지 않음 (RX 미연결 시 읽기 실패가 연결을 끊음)
        # 포트가 열려있으면 write-only로 사용 가능
        if self.gas_client:
            port_open = self.gas_client.connected
            print(f"Gas client connected: {port_open}")
            for i in range(4):
                result["gas"][i] = port_open
                self.gas_readable[i] = False

        self.connection_status = result
        return result

    def _ensure_gas_connected(self) -> bool:
        """Gas 클라이언트가 실제로 연결되어 있는지 확인하고, 끊어졌으면 재연결"""
        if not self.gas_client:
            return False
        if not self.gas_client.connected:
            print("[DeviceService] Gas client 재연결 시도...")
            try:
                ok = self.gas_client.connect()
                print(f"[DeviceService] Gas client 재연결: {'성공' if ok else '실패'}")
                if not ok:
                    return False
            except Exception as e:
                print(f"[DeviceService] Gas client 재연결 오류: {e}")
                return False
        for dev in self.gas_devices:
            dev.client = self.gas_client
            dev.connected = True
        return True

    # =====================================================
    # Gas 제어 (연결된 장치만)
    # =====================================================
    def set_gas(self, device_index, channel, value):
        if 0 <= device_index < len(self.gas_devices):
            if not self._ensure_gas_connected():
                print(f"Gas CH{device_index+1}: 연결 끊김, 전송 불가")
                return
            try:
                dev = self.gas_devices[device_index]
                dev.connected = True
                print(f"[SET_GAS] CH{device_index+1} (ID:{dev.slave_id}) setpoint={value}, "
                      f"client.connected={self.gas_client.connected}")
                ok = dev.write_setpoint(value)
                print(f"[SET_GAS] write_setpoint 결과: {ok}")
            except Exception as e:
                print(f"Gas CH{device_index+1} setpoint 오류: {e}")

    def write_gas_type(self, device_index, gas_index):
        if 0 <= device_index < len(self.gas_devices):
            if not self._ensure_gas_connected():
                print(f"Gas CH{device_index+1}: 연결 끊김, gas type 전송 불가")
                return False
            try:
                dev = self.gas_devices[device_index]
                dev.connected = True
                ok = dev.write_gas(gas_index)
                print(f"[SET_GAS_TYPE] CH{device_index+1} gas_index={gas_index} 결과: {ok}")
                return ok
            except Exception as e:
                print(f"Gas CH{device_index+1} gas type 오류: {e}")
                return False
        return False

    # =====================================================
    # Temperature 제어 (연결된 장치만)
    # =====================================================
    def set_temperature(self, value):
        if not self.connection_status["temperature"]:
            return
        if self.temp_device:
            try:
                ctrl = self.temp_device.get_controller(1)
                if ctrl:
                    ctrl.write_sv(value)
            except Exception as e:
                print(f"Temperature SV 설정 오류: {e}")

    # =====================================================
    # 전체 읽기 (읽기 가능한 장치만)
    # =====================================================
    def read_all(self):
        result = {}

        # Temperature
        if self.connection_status["temperature"]:
            try:
                all_data = self.temp_device.read_all_controllers()
                if 1 in all_data:
                    result["temperature"] = all_data[1].pv
                else:
                    result["temperature"] = None
            except Exception:
                result["temperature"] = None
        else:
            result["temperature"] = None

        # Gas (읽기 가능한 장치만)
        gas_values = []
        for i, gas_reader in enumerate(self.gas_devices):
            if self.connection_status["gas"][i] and self.gas_readable[i]:
                try:
                    data = gas_reader.read_all()
                    gas_values.append({
                        "gas": data.gas_name,
                        "sp": data.setpoint,
                        "pv": data.pressure,
                        "pressure": data.pressure,
                    })
                except Exception:
                    gas_values.append(None)
            else:
                gas_values.append(None)

        result["gas"] = gas_values

        return result
