@echo off
REM ============================================================
REM  HDrive-ETC  -  Build & Publish to PyPI
REM ============================================================
REM  Usage:
REM    publish.bat           Build + upload to PyPI (production)
REM    publish.bat --test    Build + upload to TestPyPI first
REM ============================================================

setlocal

set "PROJECT_DIR=%~dp0"
cd /d "%PROJECT_DIR%"

REM ---- Check Python available ----
python --version >nul 2>&1
if errorlevel 1 (
    echo [ERROR] Python is not installed or not in PATH.
    exit /b 1
)

REM ---- Install / upgrade build tools ----
echo.
echo [1/4] Installing build tools ...
python -m pip install --upgrade pip setuptools wheel build twine

REM ---- Clean previous builds ----
echo.
echo [2/4] Cleaning previous builds ...
if exist dist  rmdir /s /q dist
if exist build rmdir /s /q build
if exist hdrive_etc.egg-info rmdir /s /q hdrive_etc.egg-info

REM ---- Build the package ----
echo.
echo [3/4] Building package ...
python -m build
if errorlevel 1 (
    echo [ERROR] Build failed.
    exit /b 1
)

REM ---- Show what was built ----
echo.
echo Built artifacts:
dir /b dist\

REM ---- Upload ----
echo.
if "%~1"=="--upload" (
     echo [4/4] Uploading to PyPI ...
    python -m twine upload dist/*
    echo.
    echo Done! Install with:
    echo pip install hdrive-etc)
else(
    echo [4/4] Installing from source ...
    pip install -e D:\git\henschel-robotics\GitHub\python-hdrive-etc
    echo.
    echo Done! Install with:
    echo pip install -e D:\git\henschel-robotics\GitHub\python-hdrive-etc)


endlocal
