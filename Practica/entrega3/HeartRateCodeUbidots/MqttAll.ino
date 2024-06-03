/*
  Optical heart rate Detection using the MAX30102 sensor
  By: Jose Manuel Grao Soto 
  Date: April 10th, 2024

  This demo shows heart rate

  This example is based on the example8_SPO2.ino from SparkFun Electronics. 

  Hardware Connections (Breakoutboard to Arduino):
  -5V = 5V (3.3V is allowed)
  -GND = GND
  -SDA = A4 (or SDA)
  -SCL = A5 (or SCL)

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
 
*/

#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "heartRate.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFiNINA.h>
#include "secrets.h"
#include <PubSubClient.h>

MAX30105 particleSensor;

#define OLED_RESET     -1
#define SCREEN_ADDRESS 0x3C
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

int counter = 0;
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;
bool fingerDetected = false;

char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;
const char* ubidotsToken = "BBUS-Ft2l3y26TWqaojstKkclXwLREQe9sU";
const char *mqttTopic  = "/v1.6/devices/nano-iot-test/ubidots-input/lv";

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

unsigned long previousMillis = 0;
const long interval = 5000; // Intervalo de tiempo entre envío de datos (en milisegundos)
int suspendido = 0; // Inicialmente NO esta suspendido

int lastReceivedValue = 0;

void setup() {

  // Iniciar pantalla OLED
    Serial.begin(115200);
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("Error al inicializar la pantalla OLED"));
        for (;;);
    }
    display.setTextColor(SSD1306_WHITE);
    display.display();
    Serial.println("Inicializando...");

    // Iniciar conexión WiFi
    connectWiFi();

    // Iniciar conexión MQTT
    connectMQTT();

    // Iniciar Sensor
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
        Serial.println("MAX30105 no encontrado. Por favor, revisa la conexión/suministro de energía.");
        while (1);
    }
    Serial.println("Coloque su dedo índice en el sensor con presión constante.");

    particleSensor.setup();
    particleSensor.setPulseAmplitudeRed(8);
    particleSensor.setPulseAmplitudeGreen(0);
}

void loop() {
    mqttClient.loop();
    unsigned long currentMillis = millis(); // Obtener el tiempo actual en milisegundos

    // Verificar si ha pasado el intervalo de tiempo establecido en la practica
    if (suspendido == 0) {
        if (currentMillis - previousMillis >= interval) {
            previousMillis = currentMillis; // Actualizar el tiempo anterior
            // Enviar datos a Ubidots
            sendDataToUbidots(beatAvg);
        }

        long irValue = particleSensor.getIR();

        if (irValue < 50000) {
            fingerDetected = false;
            display.clearDisplay();
            display.setTextSize(2);
            display.setCursor(0, 0);
            display.print("Situe dedo");
            display.display();
        } else {
            if (!fingerDetected) {
                fingerDetected = true;
                Serial.println("Dedo detectado. Analizando ritmo...");
            }

            if (checkForBeat(irValue)) {
                long delta = millis() - lastBeat;
                lastBeat = millis();

                beatsPerMinute = 60 / (delta / 1000.0);

                if (beatsPerMinute < 255 && beatsPerMinute > 20) {
                    rates[rateSpot++] = (byte)beatsPerMinute;
                    rateSpot %= RATE_SIZE;

                    beatAvg = 0;
                    for (byte x = 0; x < RATE_SIZE; x++)
                        beatAvg += rates[x];
                    beatAvg /= RATE_SIZE;
                }
            }
        }

        if (fingerDetected) {
            Serial.print("IR=");
            Serial.print(irValue);
            Serial.print(", Avg BPM=");
            Serial.print(beatAvg);

            display.clearDisplay();
            display.setTextSize(4);
            display.setCursor(0, 0);
            display.println(beatAvg);
            display.display();

            Serial.println();
        }
    } else {
        display.clearDisplay();
        display.setTextSize(2);
        display.setCursor(0, 0);
        display.print("Suspendido");
        display.display();
    }
}

void connectMQTT() {
    Serial.println("Conectando al broker MQTT...");

    mqttClient.setServer("industrial.api.ubidots.com", 1883);
     // set the message receive callback
    mqttClient.setCallback(onMqttMessage);

    while (!mqttClient.connected()) {
        Serial.println("Intentando conexión MQTT...");
        if (mqttClient.connect("nano-iot-test", ubidotsToken, "")) {
            Serial.println("Conectado al broker MQTT!");
            // subscribe to a topic
            mqttClient.subscribe(mqttTopic);
            // topics can be unsubscribed
        } else {
            Serial.print("Error al conectar al broker MQTT, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" Intentando de nuevo en 5 segundos...");
            delay(5000);
        }
    }
}

void connectWiFi() {
    int status = WL_IDLE_STATUS;

    //Antes de conectarnos comprobaremos que nos podemos comunicar con el chip wifi
    if (status == WL_NO_MODULE) {
        Serial.println("Communication with WiFi module failed!");
        while (true);
    }

    //Comprobar la version firmware es correcta
    String fv = WiFi.firmwareVersion();
    if (fv != "1.0.0") {
        Serial.println("Please upgrade the firmware");
    }

    //Realizaremos la conexion mediante la funcion begin que recibe como parametros nombe y clave de la wifi
    while (status != WL_CONNECTED) {
        Serial.println("Conectando a la red WiFi de: ");
        Serial.println(ssid);
        status = WiFi.begin(ssid, pass);
        delay(3000);
    }
    printWifiStatus();
}

void printWifiStatus() {
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);

    long rssi = WiFi.RSSI();
    Serial.print("signal strength (RSSI):");
    Serial.print(rssi);
    Serial.println(" dBm");
}

void sendDataToUbidots(int pulsaciones) {
    String topic = "/v1.6/devices/nano-iot-test/pulsaciones";
    String payload = String(pulsaciones);
    mqttClient.publish(topic.c_str(), payload.c_str());
}

void onMqttMessage(char* topic, byte* payload, unsigned int length) {

  int receivedValue = atoi((char *)payload);
  if(lastReceivedValue != receivedValue){
    Serial.print("Mensaje recibido: ");
    Serial.println(topic);
    Serial.print("Contenido:");
    Serial.println(receivedValue);
    lastReceivedValue = receivedValue;
    suspendido = 1;
  }else{
    suspendido = 0;
  }
}
