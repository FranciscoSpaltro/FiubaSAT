## Por qué utilizar Docker

Docker es una plataforma que permite crear, desplegar y gestionar aplicaciones dentro de contenedores, entornos ligeros y portátiles que agrupan lo necesario para ejecutar una aplicación, incluyendo el código, las dependencias, las bibliotecas y las configuraciones del sistema operativo.

El uso de Docker en este proyecto podría fundamentarse utilizando tres aspectos:
1. En cuanto al trabajo individual:
    - Permite aislar el sistema operativo principal en el cual se está trabajando, evitando modificar configuraciones o librerias que causen conflictos, errores o inestabilidad en otras aplicaciones
    - A la inversa, permite mantener un entorno idéntico cada vez que se trabaje, a pesar de las modificaciones que se puedan realizar en el sistema

2. En cuanto al trabajo en grupo:
    - Docker permite crear entornos de desarrollo que pueden ser compartidos entre los miembros de un equipo. De esta forma, cualquier progreso en el proyecto va a poder ejecutarse por parte de todos los miembros del equipo
    - Estandariza las configuraciones mínimas necesarias

3. En cuanto a la continuidad del proyecto en el futuro:
    - Docker tiene la capacidad de capturar un "snapshot" del entorno de desarrollo, incluyendo todas las dependencias, configuraciones y herramientas necesarias para que una aplicación funcione correctamente. Se empaqueta el sistema operativo base, las bibliotecas, frameworks y herramientas, las variables de entorno, archivos de configuración y cualquier otro ajuste necesario. Esto beneficia la reproducibilidad, es decir, la posibilidad de recrear el entorno original aun con el paso del tiempo y las actualizaciones que surjan en ese período
    - Actua como un complemento a la documentación del proyecto

---

## Descripción de los archivos contenidos en la carpeta Docker

### 1. Dockerfile

```
FROM ubuntu:24.04
RUN mkdir -p /usr/src/
WORKDIR /usr/src/
RUN apt-get update && apt-get install -y \
    git \
    python3 \
    usbutils \
    openocd \
    make \
    wget \
    xz-utils \
    nano
```

- Descarga los paquetes necesarios para 
- Descarga la versión 24.04 de Ubuntu
- Crea el directorio /usr/src/ (-p para que cree todos los padres necesarios) y lo establece como directorio de trabajo

```
RUN git clone https://github.com/fran855/FiubaSAT.git
RUN ls -l /usr/src/FiubaSAT/Drivers/stm32f1x.cfg && \
    cp -f /usr/src/FiubaSAT/Drivers/stm32f1x.cfg /usr/share/openocd/scripts/target/stm32f1x.cfg
```

- Clona el repositorio principal
- Verifica la existencia del archivo .cfg y lo copia a la carpeta de openocd. Esto es necesario para trabajar con una Blue Pill genérica

```
RUN mkdir -p /usr/src/Descargas && \
    wget -P /usr/src/Descargas https://developer.arm.com/-/media/Files/downloads/gnu/13.3.rel1/binrel/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi.tar.xz
RUN cd /opt && \
    tar xJf /usr/src/Descargas/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi.tar.xz && \
    mv arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi gcc-arm
RUN echo "export PATH="/opt/gcc-arm/bin:$PATH"" >> ~/.bashrc
```

- Crea el directorio de Descargas (con sus carpetas intermedias si es necesario) y descarga el .tar de la cadena de herramientas ARM
- Se mueve al directorio */opt* y extrae la carpeta
- Renombra el directorio por simplicidad (*gcc-arm*)
- Agrega la ruta de la cadena de herramientas GNU ARM (/opt/gcc-arm/bin) al PATH en el archivo ~/.bashrc, lo que hace que el compilador esté disponible en futuras sesiones de la shell

```
CMD ["bash", "-c", "source ~/.bashrc && /bin/bash"]
```

- CMD define el comando por defecto que se ejecuta cuando se inicia un contenedor a partir de la imagen de Docker, en este caso `bash -c "source ~/.bashrc && /bin/bash"`
- Este comando se divide en `bash -c` (ejecuta lo que le siga en una nueva instancia de bash) y `source ~/.bashrc && /bin/bash` (carga las configuraciones del archivo *~/.bashrc* en la sesión actual de bash e inicia una nueva sesión de bash)


### 2. build_container.bat
- No es necesario pero simplifica la construcción del Docker a partir del archivo Dockerfile, al guardar todos los parámetros necesarios

```
@echo off

set "DIR_PATH=%~dp0..\..\Docker"
for %%i in ("%DIR_PATH%") do set "ABSOLUTE_DIR_PATH=%%~fi"
if not exist "%DIR_PATH%" (
    echo Directorio creado
    mkdir "%DIR_PATH%"
)
```

- Como el Dockerfile se ejecuta desde el repositorio, arma la dirección relativa a una carpeta llamada Docker que se encuentre un nivel por encima del mismo (es decir, afuera)
- Pasa de dirección relativa a absoluta y verifica si el directorio existe. Si no existe, lo crea

```
docker build -t fiubasat . --no-cache
```

- Construye una imagen Docker con el Dockerfile que se encuentra en el directorio actual (.) sin utilizar el caché (construcción limpia)
- Asigna el nombre fiubasat a la imagen creada

```
docker run -it --name fiubasat -p 4444:4444 -v "%ABSOLUTE_DIR_PATH%/app":/usr/src/app --privileged -v /dev/bus/usb:/dev/bus/usb fiubasat /bin/bash
```

- docker run: ejecuta un contenedor a partir de una imagen de Docker
- -it: (i) mantiene el flujo de entrada del contenedor abierto para poder interactuar con él, (t) asigna una pseudo-terminal al contenedor, permitiendo itneractuar con él como si fuera una terminal
- --name fiubasat: da el nombre "fiubasat" al contenedor
- -p 4444:4444: mapea el puerto 4444 del host al puerto 4444 del contenedor. Cualquier cosa que escuche en el puerto dentro del contenedor será accesible desde el host
- -v "%ABSOLUTE_DIR_PATH%/app":/usr/src/app: monta un volumen. Hace que el directorio app dentro del directorio actual se refleje en /usr/src/app, para poder compartir archivos entre el host y el contenedor
- --privileged: da privilegios adicionales al contenedor (por ejemplo, para interactuar con hardware específico)
- -v /dev/bus/usb:/dev/bus/usb: monta el dispositivo USB del host dentro del contenedor
- fiubasat: especifica la imagen de Docker que se va a utilizar para crear en el contenedor. En este caso, utiliza la versión básica del SO Ubuntu
- /bin/bash: es el comando que se ejecutará dentro del contenedor una vez que se inicie. En este caso, se lanza una sesión de Bash para permitir que interactúe con el contenedor a través de la línea de comandos

### run_container.bat
- No es necesario pero simplifica la ejecución del Docker
  
```
@echo off

docker start fiubasat
docker attach fiubasat
```

- Inicia el contenedor Docker llamado fiubasat
  - Si el contenedor está detenido, este comando lo pondrá en marcha
  - Si el contenedor ya está en ejecución, el comando no hará nada.
- Conecta la terminal actual al contenedor en ejecución llamado fiubasat

---
## Instrucciones
### Si aún no se construyó la imagen
1. Si no se cuenta con el paquete *usbipd*, instalarlo desde [acá](https://github.com/dorssel/usbipd-win/releases) o desde la carpeta FiubaSAT/Drivers
2. Ejecutar `build_container.bat`
3. En Windows, abrir una terminal con permisos de administrador
4. Ejecutar `usbipd list` para conocer el BUS ID del ST-LINK V2
5. Si es la primera vez que se utiliza o el dispositivo no figura como "Shared" (porque se cambió de puerto, por ejemplo), ejecutar `usbipd bind --busid <ID>`
6. Ejecutar `usbipd attach --wsl --busid <ID>`

> Observación: si no se va a utilizar el ST-LINK V2 en la sesión, alcanza con realizar solamente el paso 2

### Si ya se construyó la imagen anteriormente
1. Si no se cuenta con el paquete *usbipd*, instalarlo desde [acá](https://github.com/dorssel/usbipd-win/releases) o desde la carpeta FiubaSAT/Drivers
2. Ejecutar `run_container.bat`
3. En Windows, abrir una terminal con permisos de administrador
4. Ejecutar `usbipd list` para conocer el BUS ID del ST-LINK V2
5. Si es la primera vez que se utiliza o el dispositivo no figura como "Shared" (porque se cambió de puerto, por ejemplo), ejecutar `usbipd bind --busid <ID>`
6. Ejecutar `usbipd attach --wsl --busid <ID>`

> Observación: si no se va a utilizar el ST-LINK V2 en la sesión, alcanza con realizar solamente el paso 2
