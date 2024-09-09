#include <Wire.h>  // Biblioteca para comunicación I2C

#define SLAVE_ADDRESS 0x04  // Dirección I2C del esclavo, debe coincidir con la del STM32
#define TRIG_PIN 7
#define ECHO_PIN 6

int data_to_send = 42;  // Dato que el Arduino enviará al maestro

void setup() {
  Wire.begin(SLAVE_ADDRESS);  // Inicia el Arduino como esclavo en la dirección especificada
  Wire.onRequest(requestEvent);  // Define la función que se llama cuando el maestro solicita datos
  Wire.onReceive(receiveEvent);  // Define la función que se llama cuando el maestro envía datos
  Serial.begin(9600);  // Inicializa el puerto serie para depuración

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  // El loop se mantiene vacío porque el Arduino funciona de forma reactiva a las solicitudes del maestro
  delay(100);
}

// Esta función se ejecuta cuando el maestro solicita datos del esclavo
void requestEvent() {
  int distance = getDistance();
  Wire.write(distance);  // Envía el dato al maestro
}

// Esta función se ejecuta cuando el maestro envía datos al esclavo
void receiveEvent(int howMany) {
  while (Wire.available()) {
    int received = Wire.read();  // Lee el dato enviado por el maestro

    Serial.println(received);
  }
}

int getDistance(){
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  int distance = duration * 0.034 / 2;
  return distance;
}
