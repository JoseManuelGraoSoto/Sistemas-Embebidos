#include <Arduino.h>
#include <esp_sleep.h>
#include <driver/rtc_io.h>

gpio_num_t redPin = (gpio_num_t) 27;
#define greenPin 26
#define bluePin 25
RTC_DATA_ATTR int contador_reinicios;

void setup() {
  Serial.begin(9600);
  while(!Serial){}
  contador_reinicios++;

  pinMode(redPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(greenPin, OUTPUT);
}
void loop(){
  digitalWrite(greenPin, HIGH);
  for (int i = 0; i< 10; i++){
      Serial.println("Estoy haciendo trabajo ");
      Serial.println(contador_reinicios);
      delay(1000);
  }
    
  digitalWrite(greenPin, LOW);
  digitalWrite(redPin, HIGH);
  esp_sleep_enable_timer_wakeup(15000000);
  gpio_hold_en(redPin);
  esp_deep_sleep_start();
  //Cuando se duerme el led se apaga.
}