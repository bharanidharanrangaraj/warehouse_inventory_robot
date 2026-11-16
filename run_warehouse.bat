@echo off
setlocal
REM Warehouse Inventory Robot - one-click start on Windows 11 with WSL2.
REM Needs Ubuntu 22.04 in WSL with ROS 2 Humble and the packages from README section 3.1.
REM Builds this folder inside WSL, starts Gazebo + RViz, and opens the order page
REM (http://localhost:8088) in your browser. Press Ctrl+C or close this window to stop.
REM
REM Extra arguments are passed to the launch file, for example:
REM    run_warehouse.bat                    retrieval mode (order page)
REM    run_warehouse.bat mode:=audit        inventory audit (starts after 45 s)
REM    run_warehouse.bat rviz:=false        faster: no RViz window
REM
REM Optional settings (environment variables):
REM    WAREHOUSE_WSL_DISTRO   WSL distribution to use (default Ubuntu-22.04; list them with: wsl -l -v)
REM    WAREHOUSE_NO_BROWSER   set to 1 to not open the browser automatically

if not defined WAREHOUSE_WSL_DISTRO set "WAREHOUSE_WSL_DISTRO=Ubuntu-22.04"
set "OPEN=--open-browser"
if "%WAREHOUSE_NO_BROWSER%"=="1" set "OPEN="
cd /d "%~dp0"

wsl.exe -d %WAREHOUSE_WSL_DISTRO% -e true >nul 2>&1
if errorlevel 1 (
  echo WSL distribution "%WAREHOUSE_WSL_DISTRO%" was not found.
  echo Install Ubuntu 22.04 from the Microsoft Store, or set WAREHOUSE_WSL_DISTRO to the name
  echo of your Ubuntu 22.04 distribution ^(see: wsl -l -v^). Setup steps: README.md section 3.
  pause
  exit /b 1
)

REM The order page is served on Windows by a small relay (tools\web_relay_windows.py) that
REM talks to WSL through wsl.exe pipes, so it works even where a firewall blocks network
REM traffic between Windows and WSL. Without Python for Windows, WSL's own localhost
REM forwarding is used instead.
set "PY="
python --version >nul 2>&1
if not errorlevel 1 set "PY=python"
if not defined PY (
  py -3 --version >nul 2>&1
  if not errorlevel 1 set "PY=py -3"
)

if defined PY (
  start "" /b %PY% "%~dp0tools\web_relay_windows.py" --distro %WAREHOUSE_WSL_DISTRO% %OPEN%
  wsl.exe -d %WAREHOUSE_WSL_DISTRO% -e bash tools/run_in_wsl.sh %*
  %PY% "%~dp0tools\web_relay_windows.py" --stop
) else (
  echo Python for Windows not found - open http://localhost:8088 once the robot says "Ready".
  wsl.exe -d %WAREHOUSE_WSL_DISTRO% -e bash tools/run_in_wsl.sh --direct %*
)
pause
