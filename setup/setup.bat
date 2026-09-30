@echo off
setlocal
cd /d "%~dp0.."
if not exist .venv (
  py -3.12 -m venv .venv 2>nul || py -3.11 -m venv .venv 2>nul || py -3.10 -m venv .venv 2>nul || py -3 -m venv .venv
)
call .venv\Scripts\activate.bat
python -m pip install --upgrade pip
python -m pip install -r setup\requirements.txt
git submodule update --init --recursive
python -m pip install -e Src\Propulsion-model
echo.
echo Enter your ArduPilot SITL directory (WSL path like /home/you/ardupilot, or \\wsl.localhost\Ubuntu\... from Explorer).
set "ARDU_DIR="
set /p "ARDU_DIR=ArduPilot dir [Enter = ~/ardupilot]: "
set "VENV_ACT="
set /p "VENV_ACT=Python venv activate script [Enter = ~/venv-ardupilot/bin/activate]: "
python -c "import os,sys; sys.path.insert(0,'.'); from gui.launcher import load_state,save_state; s=load_state(); [s.__setitem__(k,os.environ[e].strip().strip('\"')) for k,e in (('ardupilot_dir','ARDU_DIR'),('venv_activate','VENV_ACT')) if os.environ.get(e,'').strip()]; save_state(s); print('Saved:', {k:s.get(k) for k in ('ardupilot_dir','venv_activate')})"
echo Install complete. Start with: python -m gui
endlocal
