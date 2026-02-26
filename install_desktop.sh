#!/bin/bash

APP_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# run.sh 실행 권한 부여
chmod +x "$APP_DIR/run.sh"

# .desktop 파일 생성
DESKTOP_FILE="$HOME/Desktop/SmartGasControl.desktop"

cat > "$DESKTOP_FILE" << EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=Smart Gas Control
Comment=Temperature & Gas Flow Controller
Exec=bash "$APP_DIR/run.sh"
Path=$APP_DIR
Icon=utilities-system-monitor
Terminal=false
Categories=Utility;Science;
StartupNotify=true
EOF

# 실행 권한 부여
chmod +x "$DESKTOP_FILE"

# Ubuntu 22.04+에서 "신뢰할 수 없는 앱" 경고 없이 실행되도록 허용
if command -v gio &> /dev/null; then
    gio set "$DESKTOP_FILE" metadata::trusted true 2>/dev/null
fi

echo "============================================"
echo " Desktop shortcut created!"
echo " Location: $DESKTOP_FILE"
echo "============================================"
echo ""
echo " If the icon shows 'Untrusted application':"
echo "   Right-click → Allow Launching"
echo ""
