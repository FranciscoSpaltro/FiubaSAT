# Instalación de Docker en Raspberry Pi 5 con Raspberry OS 64-bits
1. Agregar el repositorio oficial de Docker al sistema apt (administrador de paquetes de Debian/Ubuntu). De esta forma, nos aseguramos de tener acceso a las últimas versiones estables de Docker, además de actualizaciones de seguridad y parches que pueden no estar incluidas por defecto en los repositorios de Debian/Ubuntu
    ```
    # Add Docker's official GPG key:
    sudo apt-get update
    sudo apt-get install ca-certificates curl
    sudo install -m 0755 -d /etc/apt/keyrings
    sudo curl -fsSL https://download.docker.com/linux/debian/gpg -o /etc/apt/keyrings/docker.asc
    sudo chmod a+r /etc/apt/keyrings/docker.asc

    # Add the repository to Apt sources:
    echo \
    "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/debian \
    $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
    sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
    sudo apt-get update
    ```

2. Instalar los paquetes de Docker, última versión
    ```
    sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
    ```

    Observación: se puede utilizar el script `install_docker.sh` en la carpeta 'Drivers'. Para eso, descargarlo y hacerlo ejecutable con

    chmod +x install_docker.sh

    Luego, ejecutar `./install_docker.sh`

3. Ejecutar la imagen `hello-world` para verificar que la instalación se realizó de forma correcta. El comando descarga una imagen de prueba y la ejecuta en un container. Cuando esto ocurre, imprime un mensaje de confirmación y sale.
    ```
    sudo docker run hello-world
    ```

4. Crear el grupo `docker` para utilizarlo sin `sudo`
    ```
    sudo groupadd docker
    sudo usermod -aG docker $USER
    # Reiniciar o para que funcione en la terminal actual: newgrp docker
    ```

    Reintentar el `hello-world` sin sudo

4. Descargar el Dockerfile. En nuestro caso, elegimos realizar un `wget` del archivo que está almacenado en nuestro repositorio de Github. Para obtener el URL, ir al archivo dentro del repositorio y hacer click en `Raw`

    ```
    cd ~/Desktop
    mkdir FiubaSAT
    cd FiubaSAT
    wget URL_DOCKERFILE
    ```

5. Construir la imagen
    ```
    sudo docker build -t fiubasat . --no-cache
    ```

    **Opcional**: en lugar de usar URL, se puede hacer un pull de la imagen subida a DockerHub:
    ```
    docker pull frans8/fiubasat:rpi_v1.0
    ```

6. Crear la carpeta app
    ```
    mkdir app
    ```

7. Ejecutar el contenedor por primera vez
    ```
    docker run -it --name fiubasat --privileged -v /dev/bus/usb:/dev/bus/usb frans8/fiubasat:rpi_v1.0 /bin/bash
    ```

8. Para detenerlo:
    ```
    docker stop fiubasat
    # OPCIONAL: docker rm fiubasat
    ```

9. Para ejecutarlo nuevamente
    ```
    docker start -ai fiubasat
    ```