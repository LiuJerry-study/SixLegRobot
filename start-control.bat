@echo off
rem ============================================================
rem  SixLegRobot visual control console launcher (gait-sim)
rem  ASCII only on purpose: Chinese text in .bat breaks under GBK.
rem  Usage: double-click, keep this window open while controlling.
rem  Steps: 1) flash MODE_SIX_LEGS_SYNC firmware  2) run this file
rem         3) open http://localhost:8000 in Chrome/Edge
rem         4) click "Connect Serial", pick COM port (115200)
rem ============================================================
title SixLeg Control Console
cd /d "%~dp0gait-sim"
echo ============================================================
echo   Control console:  http://localhost:8000
echo   Keep this window OPEN = server is running.
echo   Close this window to stop the server.
echo   Firmware must be MODE_SIX_LEGS_SYNC.
echo   Close the PlatformIO serial monitor before connecting.
echo ============================================================
start "" "http://localhost:8000"

rem ---- try python, then py launcher, then PlatformIO python ----
python -m http.server 8000
if not errorlevel 1 goto :serve_done

py -3 -m http.server 8000
if not errorlevel 1 goto :serve_done

"%USERPROFILE%\.platformio\penv\Scripts\python.exe" -m http.server 8000
if not errorlevel 1 goto :serve_done

echo.
echo [ERROR] No usable python found. Install Python 3, or run:
echo     cd gait-sim
echo     python -m http.server 8000
pause
exit /b 1

:serve_done
echo.
echo Server stopped. Window stays open; press any key to close.
pause
