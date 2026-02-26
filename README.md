# Smart Gas System

Flet 기반 Temperature & Gas Scheduler UI

## Features
- Temperature scheduling: 8 steps (Temp, Duration, Rate 자동 계산)
- Gas scheduling: 4채널 MFC 제어 (Mixing / Manual 모드)
- 실시간 트렌드 그래프 (스케줄 점선 + 측정값 실선)
- 터치패널 지원 (숫자 키패드 내장)
- 시리얼 포트 자동 스캔 (RS-485 / RS-232)
- UT32A 온도 컨트롤러 + Alicat MFC 연동

## Ubuntu 설치 및 실행 (복사-붙여넣기)

```bash
sudo apt update && sudo apt install -y python3-venv python3-pip git
cd ~
rm -rf smart_gas_system
git clone https://github.com/changdn3732/smart_gas_system.git
cd smart_gas_system
chmod +x run.sh
./run.sh
```

## Ubuntu 업데이트 (이미 설치된 경우)

```bash
cd ~/smart_gas_system
rm -rf venv
git checkout -- .
git pull origin main
chmod +x run.sh
./run.sh
```

## Windows 실행

```bash
cd scheduler_project
python -m venv venv
venv\Scripts\activate
pip install -r requirements.txt
python main.py
```

## 장치 연결 정보
| 장치 | 통신 | 기본 포트 | Slave ID | Baud Rate |
|------|------|-----------|----------|-----------|
| UT32A (온도) | RS-485 | COM7 / /dev/ttyUSB0 | 1 | 19200 |
| Alicat MFC CH1 | RS-232 | COM5 / /dev/ttyUSB1 | 2 | 19200 |
| Alicat MFC CH2 | RS-232 | (동일 포트) | 3 | 19200 |
| Alicat MFC CH3 | RS-232 | (동일 포트) | 4 | 19200 |
| Alicat MFC CH4 | RS-232 | (동일 포트) | 5 | 19200 |
