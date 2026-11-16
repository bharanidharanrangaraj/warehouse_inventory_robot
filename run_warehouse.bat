@echo off
REM Warehouse Inventory Robot - one-click start on this laptop.
REM Runs in WSL Ubuntu-22.04 (ROS 2 Humble). Syncs this folder, rebuilds, then opens
REM Gazebo + RViz and the order page (http://localhost:8088) in your browser.
REM Close this window or press Ctrl+C to stop.
REM Extra arguments are passed on, e.g.:
REM    run_warehouse.bat                    retrieval mode (order page)
REM    run_warehouse.bat mode:=audit        inventory audit (starts after 45 s)
REM    run_warehouse.bat scan_mode:=rfid_sim

set "RELAY=%~dp0tools\web_relay_windows.py"

REM The firewall blocks Windows <-> WSL TCP, so the order page is relayed through wsl.exe.
REM The relay first stops any relay left over from an earlier run.
start "" /b python "%RELAY%" --open-browser

wsl.exe -d Ubuntu-22.04 -u ros --cd ~ -- bash -ic "~/run_warehouse.sh %*"

python "%RELAY%" --stop
pause
