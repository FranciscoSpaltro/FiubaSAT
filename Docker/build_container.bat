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