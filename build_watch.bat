@echo off
REM Build and flash ESP32-S3-EYE watch face with camera preview
REM 项目目录: D:\k\gongju\Espressif\frameworks\esp-bsp\examples\display

echo [INFO] Activating ESP-IDF environment...
call "D:\k\gongju\Espressif\esp-idf\export.bat"

echo [INFO] Switching to project directory...
cd /d "D:\k\gongju\Espressif\frameworks\esp-bsp\examples\display"
if errorlevel 1 (
    echo ERROR: Cannot access project directory
    pause
    exit /b 1
)
echo [INFO] Project: %CD%

echo.
echo [1/3] Building...
idf.py build
if errorlevel 1 (
    echo Build FAILED!
    pause
    exit /b 1
)

echo.
echo [2/3] Flashing to COM3...
idf.py -p COM3 flash
if errorlevel 1 (
    echo Flash FAILED!
    pause
    exit /b 1
)

echo.
echo [3/3] Starting monitor...
echo ========================================
echo Build and flash SUCCESSFUL!
echo.
echo To set clock time via serial:
echo   date MMDDHHMMYYYY
echo   e.g. for 17:35 April 1, 2026: date 040117352026
echo ========================================
echo.
idf.py -p COM3 monitor
