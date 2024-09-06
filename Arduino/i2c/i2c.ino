#include <Wire.h>  // Biblioteca para comunicación I2C

#define SLAVE_ADDRESS 0x08  // Dirección I2C del esclavo, debe coincidir con la del STM32

int data_to_send = 42;  // Dato que el Arduino enviará al maestro

void setup() {
  Wire.begin(SLAVE_ADDRESS);  // Inicia el Arduino como esclavo en la dirección especificada
  Wire.onRequest(requestEvent);  // Define la función que se llama cuando el maestro solicita datos
  Wire.onReceive(receiveEvent);  // Define la función que se llama cuando el maestro envía datos
  Serial.begin(9600);  // Inicializa el puerto serie para depuración
  Serial.println("Inicio.");
}

void loop() {
  // El loop se mantiene vacío porque el Arduino funciona de forma reactiva a las solicitudes del maestro
  delay(100);
}

// Esta función se ejecuta cuando el maestro solicita datos del esclavo
void requestEvent() {
  Wire.write(data_to_send);  // Envía el dato al maestro
}

// Esta función se ejecuta cuando el maestro envía datos al esclavo
void receiveEvent(int howMany) {
  while (Wire.available()) {
    int received = Wire.read();  // Lee el dato enviado por el maestro

    if(received == 0){
      Serial.println("0x00: enviar estado");
      data_to_send = 0;
    }

    if(received == 1){
      Serial.println("0x01: enviar ticks");
      data_to_send = 1;
    }
    // Aquí puedes agregar lógica para manejar el dato recibido si es necesario
  }
}
