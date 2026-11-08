@echo off
rem Builds the trumpet and starts it. Needs CMake, Git and Visual Studio (C++).
cd /d "%~dp0"

cmake -B build
if errorlevel 1 goto fail

cmake --build build --config Release
if errorlevel 1 goto fail

start "" "build\Release\trumpet.exe"
exit /b 0

:fail
echo.
echo Build failed - see the messages above.
pause
exit /b 1
