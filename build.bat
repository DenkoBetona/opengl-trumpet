@echo off
rem Builds the trumpet and starts it. Needs Visual Studio with "Desktop development with C++".
cd /d "%~dp0"

rem CMake from Visual Studio is only on the PATH inside the Developer Command Prompt,
rem so if it isn't found, find Visual Studio and load its environment first.
where cmake >nul 2>nul
if errorlevel 1 call :load_vs

where cmake >nul 2>nul
if errorlevel 1 (
    echo CMake was not found. Install "C++ CMake tools for Windows" in the Visual Studio Installer.
    goto fail
)

cmake -B build
if errorlevel 1 goto fail

cmake --build build --config Release
if errorlevel 1 goto fail

start "" "build\Release\trumpet.exe"
exit /b 0

:load_vs
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" exit /b 0
set "VSDIR="
for /f "usebackq delims=" %%i in (`"%VSWHERE%" -latest -prerelease -products * -property installationPath`) do set "VSDIR=%%i"
if defined VSDIR if exist "%VSDIR%\Common7\Tools\VsDevCmd.bat" (
    echo Loading Visual Studio environment...
    call "%VSDIR%\Common7\Tools\VsDevCmd.bat" -arch=x64 -host_arch=x64 >nul
)
exit /b 0

:fail
echo.
echo Build failed - see the messages above.
pause
exit /b 1
