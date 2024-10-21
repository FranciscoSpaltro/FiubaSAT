# Como utilizar el analizador lógico Saleae en RPi 5

1. Instalar las herramientas de sigrok (las de Saleae no estan disponibles para ARM)
    ```
        sudo apt install sigrok sigrok-cli
    ```

2. Para verificar que el dispositivo haya sido reconocido, realizar un escaneo
    ```
        sigrok-cli --scan
    ```

3. Se puede realizar un muestreo continuo en la terminal con
    ```
        sigrok-cli --config samplerate=1M --driver=fx2lafw --continuous -P i2c:sda=D0:scl=D2
    ```

    El nombre fx2lafw:conn=1.4 corresponde al que figura en el escaneo
    
4. Decodificar
    ```
        sigrok-cli -r output.sr --protocols i2c
    ```
    
sigrok-cli -d fx2lafw --config samplerate=1M -o output.sr -P i2c:sda=D0:scl=D2