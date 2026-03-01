# Smart Gas & Motor Control System

Flet 기반 온도/가스/모터 통합 제어 시스템

## 기능

### Gas/Temp 제어 (`main.py`)
- Temperature scheduling: 8 steps (Temp, Duration, Rate 자동 계산)
- Gas scheduling: 4채널 MFC 제어 (Mixing / Manual 모드)
- 실시간 트렌드 그래프 (스케줄 점선 + 측정값 실선)
- UT32A 온도 컨트롤러 + Alicat MFC 연동

### Motor 제어 (`app_motor.py`)
- PMC-2HSP 모터 드라이버 2개 (4축: 상부/하부 스테이지 + 회전)
- Schedule 모드: 속도(mm/s, cm/s, m/s) + Duration + 방향 설정, 거리 자동 계산
- Manual 모드: 조그 버튼 (hold-to-run), 저속/고속 토글
- Gantt 타임라인 + 4축 동시 속도 그래프
- 비상정지 (확인 다이얼로그)

### 공통
- 터치패널 지원 (숫자/시간 키패드 내장)
- 시리얼 포트 자동 스캔 (RS-485 / RS-232)
- Ubuntu + Windows 지원

---

## Ubuntu 원클릭 설치 (처음 설치)

터미널에 아래를 **한 번에 복사-붙여넣기** 하세요:

```bash
sudo apt update && sudo apt install -y python3-venv python3-pip git
cd ~
git clone https://github.com/changdn3732/smart_gas_system.git
cd smart_gas_system
chmod +x setup_all.sh
./setup_all.sh
```

이것만 하면 **설치 → 환경설정 → 바탕화면 아이콘 생성**까지 전부 완료됩니다.

설치 후 **재로그인** (시리얼 포트 권한 적용) 하면 바탕화면에서:
- **Smart Gas Control** — 온도/가스 제어
- **Smart Motor Control** — 모터 제어

아이콘을 클릭하여 실행할 수 있습니다.

> 아이콘에 "신뢰할 수 없는 앱" 경고가 뜨면: **우클릭 → Allow Launching**

---

## Ubuntu 업데이트 (이미 설치된 경우)

```bash
cd ~/smart_gas_system
chmod +x setup_all.sh
./setup_all.sh
```

같은 스크립트를 다시 실행하면 자동으로 `git pull` + 환경 재구성 + 아이콘 갱신을 수행합니다.

---

## Ubuntu 수동 실행 (아이콘 없이)

```bash
# Gas/Temp 제어
cd ~/smart_gas_system && ./run.sh

# Motor 제어
cd ~/smart_gas_system && ./run_motor.sh
```

---

## Windows 실행

```bash
cd scheduler_project
python -m venv venv
venv\Scripts\activate
pip install -r requirements.txt

# Gas/Temp 제어
python main.py

# Motor 제어
python app_motor.py
```

---

## 장치 연결 정보

### Gas/Temp

| 장치 | 통신 | 기본 포트 | Slave ID | Baud Rate |
|------|------|-----------|----------|-----------|
| UT32A (온도) | RS-485 | COM7 / /dev/ttyUSB0 | 1 | 19200 |
| Alicat MFC CH1 | RS-232 | COM5 / /dev/ttyUSB1 | 2 | 19200 |
| Alicat MFC CH2 | RS-232 | (동일 포트) | 3 | 19200 |
| Alicat MFC CH3 | RS-232 | (동일 포트) | 4 | 19200 |
| Alicat MFC CH4 | RS-232 | (동일 포트) | 5 | 19200 |

### Motor

| 장치 | 통신 | Slave ID | 축 | 설명 |
|------|------|----------|----|------|
| PMC-2HSP #1 | RS-485 | 1 | X축 | 상부 스테이지 (리니어) |
| PMC-2HSP #1 | RS-485 | 1 | Y축 | 상부 회전 |
| PMC-2HSP #2 | RS-485 | 2 | X축 | 하부 스테이지 (리니어) |
| PMC-2HSP #2 | RS-485 | 2 | Y축 | 하부 회전 |

> Motor는 하나의 시리얼 포트(RS-485)에 slave ID로 구분하여 다중 연결됩니다.

---

## 파일 구조

```
smart_gas_system/
├── main.py                  # Gas/Temp 제어 앱
├── app_motor.py             # Motor 제어 앱
├── requirements.txt         # Python 의존성
├── run.sh                   # Gas/Temp 실행 (Ubuntu)
├── run_motor.sh             # Motor 실행 (Ubuntu)
├── setup_all.sh             # 원클릭 설치 스크립트
├── install_desktop.sh       # Gas/Temp 바탕화면 아이콘
├── install_desktop_motor.sh # Motor 바탕화면 아이콘
└── devices/
    ├── device_service.py       # Gas/Temp 장치 서비스
    ├── gas_controller.py       # Alicat MFC 통신
    ├── temperature_controller.py # UT32A 통신
    └── motor_controller.py     # PMC-2HSP 통신
```
