// RESET EN EL PIN 2 DEL ARDUINO UNO CON EL COMANDO 'a'

void setup() {
    Serial.begin(9600); // Inicia la comunicación serie
    pinMode(2, OUTPUT); // Configura el pin 2 como salida
    digitalWrite(2, HIGH); // Asegúrate de que el pin de reset esté en alto
}

void loop() {
    if (Serial.available()) {
        char letra = Serial.read(); // Lee un carácter de la serie

        // Si la letra recibida es 'a', activa el reset del Arduino Nano
        if (letra == 'a') {
            Serial.println("Reseteando Arduino Nano..."); // Mensaje de depuración
            digitalWrite(2, LOW); // Activa el reset
            delay(100);           // Mantiene el reset por un corto período
            digitalWrite(2, HIGH); // Libera el reset
        }
    }
}
