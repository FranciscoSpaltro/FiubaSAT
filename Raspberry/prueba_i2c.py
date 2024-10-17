import smbus2
import time

ARDUINO_I2C_ADDRESS = 0x08

bus = smbus2.SMBus(1)

def request_data_from_arduino():
        try:
            bus.write_byte(ARDUINO_I2C_ADDRESS, 0x01)
            time.sleep(0.1)
            data = bus.read_byte(ARDUINO_I2C_ADDRESS)
            return data
        except Exception as e:
            print(f"Error de comunicacion i2c: {e}")
            return None
        
if __name__ == "__main__":
    data = request_data_from_arduino()
    if data is not None:
        print(f"Respuesta del Arduino: {data}")
    else:
        print("No se recibio respuesta del Arduino")
