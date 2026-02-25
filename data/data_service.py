from collections import deque


class DataService:
    def __init__(self):
        # 600초 버퍼
        self.temperature = deque([25.0] * 600, maxlen=600)
        self.gas = [deque([0.0] * 600, maxlen=600) for _ in range(4)]

    def update(self, device_data: dict):
        if device_data.get("temperature") is not None:
            self.temperature.append(device_data["temperature"])

        gas_list = device_data.get("gas", [0, 0, 0, 0])
        for i in range(4):
            self.gas[i].append(gas_list[i])