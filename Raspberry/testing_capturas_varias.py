from gpiozero import OutputDevice
import time
import subprocess
import threading

# Variables globales
SIGNAL_PIN = 17  # Cambia esto segun el pin que estas usando
signal_device = OutputDevice(SIGNAL_PIN)
MAX_TIME = 1.9  # Tiempo maximo de interrupciones
capture_i2c_values = None

# Funcion de captura I2C mediante un analizador lÃ³gico, con canales SCL en D2 y SDA en D0. Guarda los valores en la variable global capture_i2c_values
def capture_i2c(samples):
    global capture_i2c_values
  
    cmd = [
        'sigrok-cli',
        '--driver', 'fx2lafw',
        '--config', 'samplerate=1M',
        '--samples', str(samples),
        '-P', 'i2c:scl=D2:sda=D0'
    ]

    # Ejecuta el comando y captura la salida
    result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

    if result.returncode == 0:
        capture_i2c_values = process_data(result.stdout)  # Procesa la salida capturada
    else:
        print("Error en la captura:", result.stderr)
        capture_i2c_values = None

# Funcion que procesa los datos obtenidos de la captura I2C
def process_data(output):
    lines = output.splitlines()
    capture_active = False
    valores = []
    for line in lines:
        if 'Start' in line:
            valores.append('Start')
            capture_active = True

        elif capture_active and 'Write' in line:
            valores.append('Write')
            capture_active = True

        elif capture_active and 'Address write:' in line:
            data_value = line.split('Address write: ')[1].strip()
            valores.append(data_value)

        elif capture_active and 'ACK' in line:
            valores.append('ACK')

        elif capture_active and 'Data write:' in line:
            data_value = line.split('Data write: ')[1].strip()
            valores.append(data_value)

        elif 'Stop' in line:
            valores.append('Stop')
            capture_active = False

    return valores

# Funcion que envia una seÃ±al de interrupcion al dispositivo compuesta por la cantidad de pulsos indicada
def signal_interrupt(n_pulse=1):
    if n_pulse < 1 or n_pulse * 0.1 > MAX_TIME:
        return
    for _ in range(n_pulse):
        # Enviar pulso alto
        signal_device.on()
        time.sleep(0.05)
        signal_device.off()
        time.sleep(0.05)
    time.sleep(MAX_TIME - n_pulse * 0.1)

# Funcion para capturar los pulsos de echo en el canal D4 del analizador logico
def capture_echo(samples=2000000):
    # Comando para capturar el canal D4 durante 2 segundos
    cmd = [
        'sigrok-cli',
        '--driver', 'fx2lafw',
        '--config', 'samplerate=1000000',
        '--samples', str(samples),
        '--channels', 'D4'
    ]

    # Ejecuta el comando y captura la salida
    result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

    if result.returncode == 0:
        return count_pulses(result.stdout)
    else:
        print("Error en la captura:", result.stderr)
        return 0

# Funcion para contar los pulsos de alto a bajo en la seÃ±al de echo
def count_pulses(output):
    lines = output.splitlines()
    pulse_count = 0
    last_state = None

    for line in lines:
        if 'D4:' in line:
            parts = line.split(':')
            if len(parts) > 1:  # Verifica que haya suficientes elementos
                current_bits = parts[1].replace(' ', '')  # Quita espacios en blanco
   
                for bit in current_bits:
                    current_state = int(bit)  # Extrae el estado (0 o 1)

                    if last_state == 1 and current_state == 0:
                        pulse_count += 1 

                    last_state = current_state
    return pulse_count

# Funcion para codificar un mensaje en I2C, asumiendo protocolo implementado (byte 01 que indica mensaje y direccion 04 del Arduino)
def codificar_i2c(mensaje):
    codificacion = ['Start', 'Write', '04', 'ACK', '01', 'ACK']  # Inicia con el encabezado

    for char in mensaje:
        hex_char = format(ord(char), '02x').upper()  # Convertir a hexadecimal (2 digitos)
        codificacion.append(hex_char)
        codificacion.append('ACK')

    codificacion.append('Stop')
    
    return codificacion

# Funciones de testeo
def testing_1():
    signal_interrupt(1)
    recibidos = capture_echo()
    assert recibidos == 1, f"TEST 1 FALLO: esperado {1} pulso/s, recibido {recibidos} pulso/s"
    print("TEST 1 OK")

def testing_2():
    capture_thread = threading.Thread(target=capture_i2c, args=(4000000,))
    capture_thread.start()
    time.sleep(0.1)
    signal_interrupt(2)
    capture_thread.join()

    expected_output = codificar_i2c('a')
    assert capture_i2c_values == expected_output, f"TEST 2 FALLO: esperado {expected_output}, recibido {capture_i2c_values}"
    print("TEST 2 OK")

def testing_3():
    capture_thread = threading.Thread(target=capture_i2c, args=(6000000,))
    capture_thread.start()
    time.sleep(0.1)
    signal_interrupt(3)
    capture_thread.join()

    expected_output = codificar_i2c('Prueba extensa numero 1') + codificar_i2c('Prueba extensa numero 2')
    assert capture_i2c_values == expected_output, f"TEST 3 FALLO: esperado {expected_output}, recibido {capture_i2c_values}"
    print("TEST 3 OK")

try:
    testing_1()
    testing_2()
    testing_3()

except KeyboardInterrupt:
    print("Terminando el programa.")


