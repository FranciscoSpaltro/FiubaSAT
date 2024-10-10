// Comprobación del bus I2C con Arduino UNO en dirección 0x08

#include <Wire.h>
#include <avr/wdt.h>  // Librería para el Watchdog Timer

#define I2C_ADDRESS 0x08  // Dirección del esclavo I2C
#define IMPRIMIR_MENSAJE 0x01  // Comando para imprimir mensaje

// Variables globales para almacenar el comando y el mensaje recibido
uint8_t comando;
String mensajeRecibido = "";

// Función de configuración
void setup() {
    Wire.begin(I2C_ADDRESS);  // Inicializa el esclavo I2C con la dirección definida
    Wire.onReceive(onReceive); // Registra la función que se llamará al recibir datos
    Wire.onRequest(onRequest);  // Registra la función que se llamará al solicitar datos
    Serial.begin(9600);         // Inicializa la comunicación serie para imprimir
    Serial.println("Inicializando...");
}

void onReceive(int numBytes) {
    if (numBytes > 0) {
        comando = Wire.read(); // Lee el primer byte como comando
        mensajeRecibido = "";  // Reinicia el mensaje recibido

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
    // Verifica si se recibe algo por el puerto serie
    if (Serial.available()) {
        char letra = Serial.read(); // Lee un carácter de la serie

        // Si la letra recibida es 'a', alterna el estado de simularPerdida
        if (letra == 'a') {
            Serial.println("Simulación de pérdida de comunicación activada. Reseteando...");
            // Habilita el Watchdog Timer para forzar un reinicio en 15ms
            wdt_enable(WDTO_15MS);
            while(1) {
                // Espera a que el Watchdog resete el Arduino
                // Para evitar colgar el programa, puedes agregar una pausa corta.
                delay(100);  // Pequeña pausa para no colgar el sistema.
            }
        }
    }

    // Reinicia el WDT para evitar un reinicio no deseado
    wdt_reset();

    // Otras tareas en el loop (si es necesario)
}
