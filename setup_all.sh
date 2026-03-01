#!/bin/bash
#
# Smart Gas & Motor Control - 원클릭 설치 스크립트
# Ubuntu 22.04+ 에서 실행
#
# 사용법:
#   curl 또는 git clone 후:
#   chmod +x setup_all.sh && ./setup_all.sh
#

set -e

echo "========================================================"
echo "  Smart Gas & Motor Control - 설치 시작"
echo "========================================================"
echo ""

# 1) 시스템 패키지 설치
echo "[1/6] 시스템 패키지 설치..."
sudo apt update && sudo apt install -y python3-venv python3-pip git

# 2) 시리얼 포트 권한 (재로그인 후 적용)
echo "[2/6] 시리얼 포트 권한 설정..."
sudo usermod -aG dialout "$USER" 2>/dev/null || true
echo "  → dialout 그룹에 추가됨 (재로그인 후 적용)"

# 3) 프로젝트 클론 또는 업데이트
echo "[3/6] 프로젝트 다운로드..."
PROJECT_DIR="$HOME/smart_gas_system"
if [ -d "$PROJECT_DIR/.git" ]; then
    echo "  → 기존 프로젝트 발견, 업데이트 중..."
    cd "$PROJECT_DIR"
    git checkout -- .
    git pull origin main
else
    echo "  → 새로 클론 중..."
    rm -rf "$PROJECT_DIR"
    cd ~
    git clone https://github.com/changdn3732/smart_gas_system.git
fi
cd "$PROJECT_DIR"

# 4) Python 가상환경 + 의존성 설치
echo "[4/6] Python 환경 설정..."
rm -rf venv
python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
deactivate

# 5) 실행 스크립트 권한 부여
echo "[5/6] 스크립트 권한 설정..."
chmod +x run.sh run_motor.sh install_desktop.sh install_desktop_motor.sh setup_all.sh

# 6) 바탕화면 아이콘 생성
echo "[6/6] 바탕화면 아이콘 생성..."
bash install_desktop.sh
bash install_desktop_motor.sh

echo ""
echo "========================================================"
echo "  설치 완료!"
echo "========================================================"
echo ""
echo "  실행 방법:"
echo "    Gas/Temp 제어:   바탕화면 'Smart Gas Control' 아이콘"
echo "    Motor 제어:      바탕화면 'Smart Motor Control' 아이콘"
echo ""
echo "    또는 터미널에서:"
echo "      cd ~/smart_gas_system && ./run.sh"
echo "      cd ~/smart_gas_system && ./run_motor.sh"
echo ""
echo "  ※ 시리얼 포트 사용을 위해 재로그인이 필요할 수 있습니다."
echo ""
