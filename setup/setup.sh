#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
if command -v python3.12 >/dev/null 2>&1; then PY=python3.12; elif command -v python3.11 >/dev/null 2>&1; then PY=python3.11; elif command -v python3.10 >/dev/null 2>&1; then PY=python3.10; else PY=python3; fi
if [ ! -d .venv ]; then "$PY" -m venv .venv; fi
. .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r setup/requirements.txt
git submodule update --init --recursive
python -m pip install -e Src/Propulsion-model
echo "Install complete. Start with: python -m gui"
