@echo off
rem ======================================================================
rem  Build ShockMonitor.exe and the Windows installer.
rem
rem  Requirements (one time, on the Windows PC):
rem    1. Python 3.12 (64-bit) from python.org - tick "Add python.exe to PATH"
rem    2. Inno Setup 6 (https://jrsoftware.org/isdl.php) - only for the installer
rem
rem  Then double-click this file (or run it from a command prompt).
rem  Results:
rem    dist\ShockMonitor\ShockMonitor.exe          portable program folder
rem    dist\installer\ShockMonitor-Setup-<ver>.exe  installer
rem ======================================================================
setlocal
cd /d "%~dp0\.."

echo [1/5] Creating build environment (.venv-build)...
if not exist .venv-build (
    py -3.12 -m venv .venv-build 2>nul || python -m venv .venv-build
)
if errorlevel 1 goto :fail
call .venv-build\Scripts\activate.bat

echo [2/5] Installing packages...
python -m pip install --upgrade pip >nul
python -m pip install -r dashboard\requirements.txt "pyinstaller>=6.6"
if errorlevel 1 goto :fail

echo [3/5] Running tests...
python -m unittest discover -s dashboard/tests -t .
if errorlevel 1 goto :fail

echo [4/5] Building ShockMonitor.exe...
pyinstaller --noconfirm --clean packaging\shock_monitor.spec
if errorlevel 1 goto :fail

echo [5/5] Building installer...
set ISCC="%ProgramFiles(x86)%\Inno Setup 6\ISCC.exe"
if not exist %ISCC% set ISCC="%ProgramFiles%\Inno Setup 6\ISCC.exe"
if not exist %ISCC% (
    echo Inno Setup 6 not found - skipping the installer.
    echo The program is ready to run: dist\ShockMonitor\ShockMonitor.exe
    goto :done
)
for /f %%v in ('python -c "import dashboard; print(dashboard.__version__)"') do set APPVER=%%v
%ISCC% /DAppVersion=%APPVER% packaging\installer.iss
if errorlevel 1 goto :fail

:done
echo.
echo Build finished.
echo   Program:   dist\ShockMonitor\ShockMonitor.exe
if exist dist\installer echo   Installer: dist\installer\
pause
exit /b 0

:fail
echo.
echo BUILD FAILED - see the messages above.
pause
exit /b 1
