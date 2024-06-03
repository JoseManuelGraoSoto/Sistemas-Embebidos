#include <Arduino.h>

TaskHandle_t TaskLed;

// Estructura para almacenar los parámetros de la tarea
struct GenericData {
  int pin;
  int delayTime;
};

// Prototipo de la función de la tarea
void tareaLed(void *params);

void setup() {
  Serial.begin(115200);

  // Crear tres instancias de la estructura GenericData con diferentes parámetros
  static GenericData paramsRed = {27, 1000};   // Pin 2 para el LED rojo, delay de 1000ms
  static GenericData paramsBlue = {25, 2000};  // Pin 4 para el LED azul, delay de 2000ms
  static GenericData paramsGreen = {26, 5000}; // Pin 5 para el LED verde, delay de 5000ms

  // Crear las tareas para controlar los LEDs con diferentes parámetros
  xTaskCreate(tareaLed, "Controlador Red", 1024, (void*)&paramsRed, 1, NULL);
  xTaskCreate(tareaLed, "Controlador Green", 1024, (void*)&paramsGreen, 1, NULL);
  xTaskCreate(tareaLed, "Controlador Blue", 1024, (void*)&paramsBlue, 1, NULL);
}

// Tarea que controla los LEDs según los parámetros recibidos
void tareaLed(void *params) {
  GenericData* taskParams = (GenericData *)params;

  for (;;) {
    // Configurar el pin como salida
    pinMode(taskParams->pin, OUTPUT);

    // Encender el LED correspondiente
    digitalWrite(taskParams->pin, HIGH);
    Serial.print("Encendiendo LED en pin ");
    Serial.print(taskParams->pin);
    Serial.println("...");
    // Esperar el tiempo especificado
    vTaskDelay(pdMS_TO_TICKS(taskParams->delayTime));
    
    // Apagar el LED
    digitalWrite(taskParams->pin, LOW);
    Serial.print("Apagando LED en pin ");
    Serial.print(taskParams->pin);
    Serial.println("...");
    // Esperar el tiempo especificado
    vTaskDelay(pdMS_TO_TICKS(taskParams->delayTime)); 
  }
}

void loop() {
  // No hay tarea específica en el loop
}
