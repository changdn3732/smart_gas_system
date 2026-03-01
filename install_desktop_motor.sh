#!/bin/bash

APP_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

chmod +x "$APP_DIR/run_motor.sh"

DESKTOP_FILE="$HOME/Desktop/SmartMotorControl.desktop"

cat > "$DESKTOP_FILE" << EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=Smart Motor Control
Comment=PMC-2HSP Motor Controller
Exec=bash "$APP_DIR/run_motor.sh"
Path=$APP_DIR
Icon=applications-engineering
Terminal=false
Categories=Utility;Science;
StartupNotify=true
EOF

chmod +x "$DESKTOP_FILE"

if command -v gio &> /dev/null; then
    gio set "$DESKTOP_FILE" metadata::trusted true 2>/dev/null
fi

echo "============================================"
echo " Motor Control desktop shortcut created!"
echo " Location: $DESKTOP_FILE"
echo "============================================"
echo ""
echo " If 'Untrusted application':"
echo "   Right-click → Allow Launching"
echo ""
