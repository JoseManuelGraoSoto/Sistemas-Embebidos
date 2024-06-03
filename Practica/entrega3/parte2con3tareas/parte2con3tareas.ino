#include <Arduino.h>

TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;

// Define los pines para los LEDs RGB
const int RED_LED_PIN = 27;
const int BLUE_LED_PIN = 25;
const int GREEN_LED_PIN = 26;

void setup() {
  Serial.begin(115200);

  // Crear tarea 1 para el LED rojo
  xTaskCreatePinnedToCore(
    taskRedLED,     // Función de la tarea
    "RedLED",       // Nombre de la tarea
    10000,          // Tamaño de la pila de la tarea
    NULL,           // Parámetro de la tarea
    1,              // Prioridad de la tarea
    &Task1,         // Manejador de la tarea
    0               // Asignar tarea al núcleo 0
  );

  // Crear tarea 2 para el LED azul
  xTaskCreatePinnedToCore(
    taskBlueLED,    // Función de la tarea
    "BlueLED",      // Nombre de la tarea
    10000,          // Tamaño de la pila de la tarea
    NULL,           // Parámetro de la tarea
    1,              // Prioridad de la tarea
    &Task2,         // Manejador de la tarea
    0               // Asignar tarea al núcleo 0
  );

  // Crear tarea 3 para el LED verde
  xTaskCreatePinnedToCore(
    taskGreenLED,   // Función de la tarea
    "GreenLED",     // Nombre de la tarea
    10000,          // Tamaño de la pila de la tarea
    NULL,           // Parámetro de la tarea
    1,              // Prioridad de la tarea
    &Task3,         // Manejador de la tarea
    0               // Asignar tarea al núcleo 0
  );
}

// Tarea para el LED rojo
void taskRedLED(void * pvParameters) {
  pinMode(RED_LED_PIN, OUTPUT);
  
  for (;;) {
    digitalWrite(RED_LED_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(1000));
    digitalWrite(RED_LED_PIN, LOW);
    Serial.println("Enchufando LED rojo... cada 1 segundo");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// Tarea para el LED azul
void taskBlueLED(void * pvParameters) {
  pinMode(BLUE_LED_PIN, OUTPUT);
  
  for (;;) {
    digitalWrite(BLUE_LED_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(2000));
    digitalWrite(BLUE_LED_PIN, LOW);
    Serial.println("Enchufando LED azul...cada 2 segundos");
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

// Tarea para el LED verde
void taskGreenLED(void * pvParameters) {
  pinMode(GREEN_LED_PIN, OUTPUT);
  
  for (;;) {
    digitalWrite(GREEN_LED_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(5000));
    digitalWrite(GREEN_LED_PIN, LOW);
    Serial.println("Enchufando LED verde... cada 5 segundos");
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

void loop() {
  // No hay tarea específica en el loop
}
