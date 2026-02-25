from devices.gas_controller import GasController
from devices.temperature_controller import TemperatureController


class DeviceService:
    def __init__(self):
        self.gas = None
        self.temp = None

    def connect_simulator(self):
        self.gas = GasController(simulator=True)
        self.temp = TemperatureController(simulator=True)

    def connect_serial(self, port, baudrate):
        self.gas = GasController(port=port, baudrate=baudrate)
        self.temp = TemperatureController(port=port, baudrate=baudrate)

    def set_gas(self, ch, value):
        if self.gas:
            self.gas.set_flow(ch, value)

    def set_temperature(self, value):
        if self.temp:
            self.temp.set_temperature(value)

    def read_all(self):

        result = {}

        # ===== Temperature =====
        if self.temp_controller:
            for ctrl in self.temp_controller.controllers.values():
                data = ctrl.read_all()
                result["temperature"] = data.pv

        # ===== Gas =====
        if self.gas_controller:
            gas_data = self.gas_controller.read_all()
            result["gas"] = [
                gas_data.ch1,
                gas_data.ch2,
                gas_data.ch3,
                gas_data.ch4
            ]

        return result