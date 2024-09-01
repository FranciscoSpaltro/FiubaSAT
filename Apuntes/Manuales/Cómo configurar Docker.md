### 1. Por qué utilizar Docker

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
# UTILIZANDO DOCKERFILE
1. Abrir la terminal de Docker
2. Ir a la carpeta donde está el Dockerfile y los .bat
3. Correr `./run_container.bat` | Obs: hay que clonar libopencm3 y hacer make
4. Descargar el driver de USBIPD del repositorio
5. Abrir PowerShell con permisos de administrador
6. Ubicar el ID del ST-Link con `usbipd list`
7. Ejecutar `usbipd bind --busid <ID>` con el ID correspondiente
8. Ejecutar `usbipd attach --wsl --busid <ID>` con el ID correspondiente
9. (OPC) En WSL, ejecutar `lsusb` para verificar que esté el ST-Link 




---
# UTILIZANDO DOCKER HUB [REVISAR]

1. Iniciar sesión en Docker Hub
2. Crear un repositorio y nombrarlo <nombre-de-usuario>/nombre-de-repo
3. Escribir el archivo Dockerfile
4. Iniciar sesión en Docker Hub desde Docker Desktop o la terminal
5. Construir la imagen Docker ejecutando `docker build -t <nombre-de-usuario>/nombre-de-repo`
6. Para testear localmente la imagen Docker, ejecutar `docker run <nombre-de-usuario>/nombre-de-repo`
7. Pushear la imagen Docker a Docker Hub ejecutando `docker push <nombre-de-usuario>/nombre-de-repo`


---
Observaciones:

    `docker run -it --name ubuntu -p 4444:4444 -v "$(pwd)/app":/usr/src/app --privileged -v /dev/bus/usb:/dev/bus/usb ubuntu /bin/bash`

- docker run: ejecuta un contenedor a partir de una imagen de Docker
- -it: (i) mantiene el flujo de entrada del contenedor abierto para poder interactuar con él, (t) asigna una pseudo-terminal al contenedor, permitiendo itneractuar con él como si fuera una terminal
- --name ubuntu: da el nombre "ubuntu" al contenedor
- -p 4444:4444: mapea el puerto 4444 del host al puerto 4444 del contenedor. Cualquier cosa que escuche en el puerto dentro del contenedor será accesible desde el host
- -v "$(pwd)/app":/usr/src/app: monta un volumen. Hace que el directorio app dentro del directorio actual se refleje en /usr/src/app, para poder compartir archivos entre el host y el contenedor
- --privileged: da privilegios adicionales al contenedor (por ejemplo, para interactuar con hardware específico)
- -v /dev/bus/usb:/dev/bus/usb: monta el dispositivo USB del host dentro del contenedor
- ubuntu: especifica la imagen de Docker que se va a utilizar para crear en el contenedor. En este caso, utiliza la versión básica del SO Ubuntu
- /bin/bash: es el comando que se ejecutará dentro del contenedor una vez que se inicie. En este caso, se lanza una sesión de Bash para permitir que interactúe con el contenedor a través de la línea de comandos

> modifique `"$(pwd)/app"` por `ruta/app`, sin comillas y con la carpeta app creada

