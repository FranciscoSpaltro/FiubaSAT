# Habilitación del puerto I2C en la Raspberry Pi 5 con Raspberry OS 64-bits
1. En la terminal, ejecutar ```sudo raspi-config```
2. Seleccionar **3. Interface Options**
3. Seleccionar **I5. I2C**
4. Habilitar el puerto
5. Presionar **Esc** para salir del entorno de configuración

# Probar la habilitación
Los pines GPIO 0 y GPIO 1 están asignados por defecto al bus I2C0. Sin embargo, estos pines son normalmente utilizados por el sistema para leer la memoria EEPROM de los HATs (Hardware Attached on Top, módulos de hardware que pueden conectarse a la Raspberry Pi) cuando estos están conectados. Suponiendo que utilizamos I2C1 (GPIO2 SDA, GPIO3 SCL)

1. Conectar un esclavo a los pines
2. En la terminal, ejecutar ```i2cdetect -y 1``` (1 por el I2C1)

    Si el comando no funciona, ejecutar primero ```sudo apt-get install i2c-tools```

3. Corroborar que aparezca la dirección del esclavo. Caso contrario, revisar la conección

# Entornos en Python
1. En la terminal, dirigirse a la carpeta del proyecto (o donde se quiera guardar el entorno)
2. Ejecutar ```python3 -m venv nombre_entorno```
3. Seleccionar el inteprete **nombre_entorno/bin/python**


- Para activar el entorno, ejecutar ```source nombre_entorno/bin/activate```
- Para guardar las dependencias utilizadas, ejecutar ```pip freeze > requirements.txt```
- Para desactivar el entorno, ejecutar ```deactivate``` 