@echo off

REM Establecer la ruta relativa al directorio objetivo
set "DIR_PATH=%~dp0..\..\Docker"

REM Convertir la ruta a una ruta absoluta
for %%i in ("%DIR_PATH%") do set "ABSOLUTE_DIR_PATH=%%~fi"

REM Verificar si el directorio existe
if not exist "%DIR_PATH%" (
    echo Directorio creado
    mkdir "%DIR_PATH%"
)

docker build -t frans8/fiubasat . --no-cache
docker run -it --name fiubasat -p 4444:4444 -v "%ABSOLUTE_DIR_PATH%/app":/usr/src/app --privileged -v /dev/bus/usb:/dev/bus/usb frans8/fiubasat /bin/bash
endlocal

REM USB
setlocal enabledelayedexpansion

:: Inicializa variables
set "STLINK_ID="

:: Leer toda la salida en una variable
for /f "tokens=*" %%i in ('usbipd list') do (
    set "line=%%i"
    echo !line! | findstr /c:"STM32 STLink" >nul
    if not errorlevel 1 (
        for /f "tokens=1" %%a in ("!line!") do set "STLINK_ID=%%a"
    )
)

:: Verificar si se encontró el ST-Link
if "%STLINK_ID%"=="" (
    echo No se encontró el ST-Link. Asegúrate de que el dispositivo esté conectado.
    exit /b 1
)

:: Comprobar si el ST-Link ya está compartido
for /f "tokens=*" %%i in ('usbipd list') do (
    echo %%i | findstr /c:"%STLINK_ID%" | findstr /c:"Shared" >nul
    if not errorlevel 1 (
        echo El ST-Link ya está compartido.
        set "already_shared=true"
    )
)

:: Si no está compartido, hacer bind
if not defined already_shared (
    echo Compartiendo ST-Link con BUSID: %STLINK_ID%
    usbipd bind --busid %STLINK_ID%
    if errorlevel 1 (
        echo Error al compartir el ST-Link.
        exit /b 1
    )
)

:: Comprobar si el ST-Link ya está adjunto
for /f "tokens=*" %%i in ('usbipd list') do (
    echo %%i | findstr /c:"%STLINK_ID%" | findstr /c:"Attached" >nul
    if not errorlevel 1 (
        echo El ST-Link ya está adjunto.
        set "already_attached=true"
    )
)

:: Si no está adjunto, hacer attach
if not defined already_attached (
    echo Adjuntando ST-Link con BUSID: %STLINK_ID%
    usbipd attach --wsl --busid %STLINK_ID%
    if errorlevel 1 (
        echo Error al adjuntar el ST-Link.
        exit /b 1
    )
)

echo ST-Link (%STLINK_ID%) está ahora compartido y adjunto correctamente.

endlocal