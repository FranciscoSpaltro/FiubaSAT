#include <Wire.h>  // Incluimos la librería Wire para I2C

#define HTDU21D_ADDRESS 0x40  // Dirección I2C del sensor HTU21D

#define TRIGGER_TEMP_MEASURE_HOLD  0xE3
#define TRIGGER_HUMD_MEASURE_HOLD  0xE5
#define TRIGGER_TEMP_MEASURE_NOHOLD  0xF3
#define TRIGGER_HUMD_MEASURE_NOHOLD  0xF5
#define WRITE_USER_REG  0xE6
#define READ_USER_REG  0xE7
#define SOFT_RESET  0xFE

byte sensorStatus;

void setup() {
  Serial.begin(115200);
  Wire.begin();  // Iniciamos la comunicación I2C
  Serial.println("HTU21D sensor initialization...");

  resetSensor();  // Realizamos un reset al sensor
  delay(15);  // Esperamos unos milisegundos para que el sensor se reinicie
}

void loop() {
  float temperature = readTemperature();
  float humidity = readHumidity();

  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" C");

  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");

  delay(2000);  // Esperamos 2 segundos antes de la siguiente lectura
}

void resetSensor() {
  Wire.beginTransmission(HTDU21D_ADDRESS);
  Wire.write(SOFT_RESET);
  Wire.endTransmission();
  delay(15);  // El reset toma unos 15 ms
}

float readTemperature() {
  Wire.beginTransmission(HTDU21D_ADDRESS);
  Wire.write(TRIGGER_TEMP_MEASURE_NOHOLD);  // Solicitamos la medición de temperatura
  Wire.endTransmission();
  
  delay(50);  // Esperamos a que la medición esté disponible

  Wire.requestFrom(HTDU21D_ADDRESS, 3);  // Pedimos 3 bytes al sensor (dos para los datos y uno para la CRC)

  if (Wire.available() == 3) {
    uint16_t rawTemperature = (Wire.read() << 8) | Wire.read();  // Leemos los dos primeros bytes
    byte crc = Wire.read();  // Leemos el tercer byte (CRC, que podemos ignorar por simplicidad)

    rawTemperature &= 0xFFFC;  // Aplicamos máscara para quitar los bits de estado
    float temp = -46.85 + (175.72 * rawTemperature / 65536.0);  // Convertimos los datos a temperatura en grados Celsius
    return temp;
  } else {
    return NAN;  // Si no se reciben los 3 bytes, devolvemos "NaN" (Not a Number)
  }
}

float readHumidity() {
  Wire.beginTransmission(HTDU21D_ADDRESS);
  Wire.write(TRIGGER_HUMD_MEASURE_NOHOLD);  // Solicitamos la medición de humedad
  Wire.endTransmission();
  
  delay(50);  // Esperamos a que la medición esté disponible

  Wire.requestFrom(HTDU21D_ADDRESS, 3);  // Pedimos 3 bytes al sensor

  if (Wire.available() == 3) {
    uint16_t rawHumidity = (Wire.read() << 8) | Wire.read();  // Leemos los dos primeros bytes
    byte crc = Wire.read();  // Leemos el tercer byte (CRC)

    rawHumidity &= 0xFFFC;  // Aplicamos máscara para quitar los bits de estado
    float hum = -6.0 + (125.0 * rawHumidity / 65536.0);  // Convertimos los datos a porcentaje de humedad
    return hum;
  } else {
    return NAN;  // Si no se reciben los 3 bytes, devolvemos "NaN"
  }
}
