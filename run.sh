#!/bin/bash

APP_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$APP_DIR"

# 깨진 venv가 있으면 삭제 후 재생성
if [ -d "venv" ] && [ ! -f "venv/bin/activate" ]; then
    echo "Broken venv detected, removing..."
    rm -rf venv
fi

if [ ! -d "venv" ]; then
    echo "Creating virtual environment..."
    python3 -m venv venv
    source venv/bin/activate
    pip install --upgrade pip
    pip install -r requirements.txt
else
    source venv/bin/activate
fi

python3 main.py