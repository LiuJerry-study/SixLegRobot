@echo off
rem ============================================================
rem  SixLegRobot visual control console launcher (gait-sim)
rem  ASCII only on purpose: Chinese text in .bat breaks under GBK.
rem  Usage: double-click, keep this window open while controlling.
rem  What it does (in this order):
rem    1) cd into the gait-sim\ folder next to this file ("%~dp0gait-sim")
rem    2) open http://localhost:8000 in the default browser FIRST
rem    3) serve that folder there with "python -m http.server 8000"
rem       (no build step; port is always 8000; the server is started by the
rem        first python that works: python, then "py -3", then PlatformIO's
rem        own .platformio\penv\Scripts\python.exe; if none works the script
rem        prints instructions and exits with code 1)
rem  The browser is opened BEFORE the server starts, so press F5 once if the
rem    first load fails. Closing this window (or Ctrl+C) stops the server;
rem    the window itself stays open and waits for a key press.
rem  Firmware: ground-mode teleop (click the floor = goto) needs
rem    -DMODE_SIX_LEGS_SYNC; the hanging "servo twin" panel only needs
rem    any mode that writes servos through ServoManager::write(),
rem    since that is where the @SERVO telemetry line is emitted
rem    (platformio.ini as shipped enables -DMODE_SINGLE_LEG_TEST).
rem  Steps: 1) flash firmware  2) run this file
rem         3) browser opens http://localhost:8000 (use Chrome/Edge)
rem         4) in the page press the connect-serial button, pick the
rem            ESP32 COM port (115200)
rem  Note: close the PlatformIO / Arduino serial monitor first (the port
rem    is exclusive); Web Serial needs http(s) - file:// will not do
rem    (double-clicking gait-sim/index.html only gets the offline gait sim).
rem  The echo banner below still says "Firmware must be MODE_SIX_LEGS_SYNC";
rem    that string is left as-is, only ground-mode teleop needs that mode.
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

rem ---- try python, then py launcher, then PlatformIO python (first that runs wins);
rem ---- all three serve the SAME folder and port 8000 from the cd above ----
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
rem server has stopped (Ctrl+C / window close / port busy) - keep the log visible
echo.
echo Server stopped. Window stays open; press any key to close.
pause
