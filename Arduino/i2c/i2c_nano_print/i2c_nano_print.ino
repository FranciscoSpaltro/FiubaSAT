#include <Wire.h>

#define I2C_ADDRESS 8  // Dirección del esclavo I2C
#define IMPRIMIR_MENSAJE 0x01  // Comando para imprimir mensaje

// Variables globales para almacenar el comando y el mensaje recibido
uint8_t comando;
String mensajeRecibido = "";

// Función de configuración
void setup() {
    Wire.begin(I2C_ADDRESS);  // Inicializa el esclavo I2C con la dirección definida
    Wire.onReceive(onReceive); // Registra la función que se llamará al recibir datos
    Wire.onRequest(onRequest); // Registra la función que se llamará al solicitar datos
    Serial.begin(9600);        // Inicializa la comunicación serie para imprimir
}

// Función que se llama cuando se reciben datos
void onReceive(int numBytes) {
    if (numBytes > 0) {
        comando = Wire.read(); // Lee el primer byte como comando
        mensajeRecibido = "";   // Reinicia el mensaje recibido
        while (Wire.available()) { // Lee el resto de los bytes
            char c = Wire.read();
            mensajeRecibido += c;  // Concatenar el carácter al mensaje
        }
        
        // Verifica si el comando es el esperado
        if (comando == IMPRIMIR_MENSAJE) {
            Serial.println(mensajeRecibido); // Imprime el mensaje recibido
        }
    }
}

// Función que se llama cuando se solicita datos del esclavo
void onRequest() {
    // Si necesitas enviar algún dato al maestro, lo puedes hacer aquí
}

void loop() {
    // El código del loop puede estar vacío si solo se espera recibir datos
}
