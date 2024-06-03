#include <Arduino.h>

TaskHandle_t Task1;
TaskHandle_t Task2;


void setup() {
  Serial.begin(115200);

  // Crear tarea 1 que escribe en el puerto serial cada 100 ms
  xTaskCreatePinnedToCore(
    taskFunction1,     // Función de la tarea
    "Task1",       // Nombre de la tarea
    10000,         // Tamaño de la pila de la tarea
    NULL,          // Parámetro de la tarea
    1,             // Prioridad de la tarea
    &Task1,        // Manejador de la tarea
    0              // Asignar tarea al núcleo 0
  );

  // Crear tarea 2 que enciende y apaga el LED integrado cada 5 segundos
  xTaskCreatePinnedToCore(
    taskFunction2,     // Función de la tarea
    "Task2",       // Nombre de la tarea
    10000,         // Tamaño de la pila de la tarea
    NULL,          // Parámetro de la tarea
    1,             // Prioridad de la tarea
    &Task2,        // Manejador de la tarea
    1              // Asignar tarea al núcleo 1
  );
}

// Tarea 1: escribe en el puerto serial cada 100 ms
void taskFunction1(void * pvParameters) {
  for (;;) {
    Serial.printf("Hola soy la tarea %s, me estoy ejecutando en el core %d\n", pcTaskGetName(NULL), xPortGetCoreID());
    vTaskDelay(pdMS_TO_TICKS(1000)); // Esperar 100 ms
  } 
}

// Tarea 2: enciende y apaga el LED integrado cada 5 segundos
void taskFunction2(void *pvParameters) {
  pinMode(LED_BUILTIN, OUTPUT);
  for (;;) {
    digitalWrite(LED_BUILTIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(5000));
    digitalWrite(LED_BUILTIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

void loop() {
  // No hay tarea específica en el loop
}

