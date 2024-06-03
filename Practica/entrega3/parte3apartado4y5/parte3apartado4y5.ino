#include <Arduino.h>
#include <esp_sleep.h>
#include <driver/rtc_io.h>
#include <ESP32Time.h> // Asegúrate de instalar esta biblioteca si no la tienes


gpio_num_t redPin = (gpio_num_t) 27;
#define greenPin 26
#define bluePin 25
gpio_num_t wakeUpPin = (gpio_num_t) 4;
gpio_num_t unusedPin = (gpio_num_t) 14;



RTC_DATA_ATTR int contador_reinicios;
RTC_DATA_ATTR uint64_t tiempo_despierto = 0;
RTC_DATA_ATTR uint64_t tiempo_despierto = 0;


void setup() {
  Serial.begin(9600);
  while (!Serial) {}

  if(contador_reinicios != 0){
    obtenerTiempoDespierto();
  }

  contador_reinicios++;
  
  pinMode(redPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(unusedPin, OUTPUT); // Configura el pin como salida
  digitalWrite(unusedPin, HIGH); // Establece el pin en HIGH
  pinMode(wakeUpPin, INPUT); // Configura el pin como salida
  // Obtén el tiempo despierto al iniciar
  tiempo_despierto = esp_timer_get_time();

}

void loop() {

  digitalWrite(greenPin, HIGH);
  for (int i = 0; i< 10; i++){
      Serial.println("Estoy haciendo trabajo ");
      Serial.println(contador_reinicios);
      delay(1000);
  }
  digitalWrite(greenPin, LOW);
  digitalWrite(unusedPin, HIGH); // Mantener el pin en HIGH
  digitalWrite(redPin, HIGH);

  gpio_deep_sleep_hold_en();
  // Mantengo el pin en alto incluso en modo de sueño
  gpio_hold_en(unusedPin);
  gpio_hold_en(redPin);

  esp_sleep_enable_ext0_wakeup(wakeUpPin, HIGH);
  
  esp_deep_sleep(15000000);
}

void obtenerTiempoDespierto() {
  uint64_t tiempo_dormido = esp_timer_get_time() - tiempo_despierto;
  Serial.print("El microcontrolador estuvo dormido durante: ");
  Serial.print(tiempo_dormido);
  Serial.println(" microsegundos");
  
  // Obtén la causa del despertar
  esp_sleep_wakeup_cause_t causa = esp_sleep_get_wakeup_cause();
  switch(causa) {
    case ESP_SLEEP_WAKEUP_EXT1:
      Serial.println("El microcontrolador fue despertado por un evento externo en el pin");
      break;
    default:
      Serial.println("El microcontrolador fue despertado por una causa desconocida");
      break;
  }
}
