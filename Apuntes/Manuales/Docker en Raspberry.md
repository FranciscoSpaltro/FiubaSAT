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

3. Ejecutar la imagen `hello-world` para verificar que la instalación se realizó de forma correcta. El comando descarga una imagen de prueba y la ejecuta en un container. Cuando esto ocurre, imprime un mensaje de confirmación y sale.

4. Descargar el Dockerfile. En nuestro caso, elegimos realizar un `wget` del archivo que está almacenado en nuestro repositorio de Github. Para obtener el URL, ir al archivo dentro del repositorio y hacer click en `Raw`

    ```
        cd ~/Desktop
        mkdir FiubaSAT
        cd FiubaSAT
        wget https://raw.githubusercontent.com/fran855/FiubaSAT/refs/heads/main/Docker/Dockerfile
    ```

5. Construir la imagen
    ```
        sudo docker build -t fiubasat . --no-cache
    ```

6. Crear la carpeta app
    ```
        mkdir app
    ```

7. [Opcional] Armar un script para correr la imagen
    i. Crear el archivo `run_container.sh`
        ```
            nano run_container.sh
        ```

    ii. Escribir el script
        ```
            #!/bin/bash

            APP_DIR="$HOME/Desktop/FiubaSAT/app"
            IMAGE_NAME="fiubasat"

            # Verifica si el contenedor ya existe
            if [ "$(sudo docker ps -aq -f name=$IMAGE_NAME)" ]; then
                # Verifica si el contenedor está corriendo
                if [ "$(sudo docker ps -q -f name=$IMAGE_NAME)" ]; then
                    echo "El contenedor '$IMAGE_NAME' ya está en ejecución. Accediendo al contenedor..."
                    sudo docker exec -it $IMAGE_NAME /bin/bash
                else
                    echo "El contenedor '$IMAGE_NAME' existe pero está detenido. Iniciándolo..."
                    sudo docker start $IMAGE_NAME
                    sudo docker exec -it $IMAGE_NAME /bin/bash
                fi
            else
                echo "No existe un contenedor con el nombre '$IMAGE_NAME'. Creando y ejecutando uno nuevo..."
                sudo docker run -it --name $IMAGE_NAME -p 4444:4444 -v "$APP_DIR":/usr/src/app --privileged -v /dev/bus/usb:/dev/bus/usb $IMAGE_NAME /bin/bash
            fi
        ```
    
    iii. Hacer el script ejecutable
        ```
            chmod +x run_container.sh
        ```