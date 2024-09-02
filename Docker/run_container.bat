@echo off

docker build -t frans8/fiubasat . --no-cache
docker run -it --name fiubasat -p 4444:4444 -v "%cd%/app":/usr/src/app --privileged -v /dev/bus/usb:/dev/bus/usb frans8/fiubasat /bin/bash