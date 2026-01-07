#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>

const char* ssid = "Nguyễn Hải Trung";
const char* password = "12345678910";

WiFiUDP udp;
const int UDP_PORT = 23456;

// Mapping motor pins
int motorPins[6] = {16, 32, 19, 27, 13, 26};

char packetBuffer[256];

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 6; i++) {
    pinMode(motorPins[i], OUTPUT);
    digitalWrite(motorPins[i], LOW);
  }

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }

  Serial.println("\nESP32 connected!");
  Serial.println(WiFi.localIP());

  udp.begin(UDP_PORT);
}

void stopAllMotors() {
  for (int i = 0; i < 6; i++) {
    digitalWrite(motorPins[i], LOW);
  }
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1);
    if (len > 0) packetBuffer[len] = 0;

    Serial.print("UDP raw: ");
    Serial.println(packetBuffer);

    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, packetBuffer);
    if (err) {
      Serial.print("JSON error: ");
      Serial.println(err.c_str());
      return;
    }

    int mode = doc["mode"];
    JsonArray motors = doc["motors"];

    Serial.print("Mode = ");
    Serial.println(mode);

    Serial.print("Motors = ");
    for (int i = 0; i < 6; i++) {
      Serial.print((int)motors[i]);
      Serial.print(" ");
    }
    Serial.println();

    stopAllMotors();

    for (int i = 0; i < 6; i++) {
      if (motors[i] == 1) {
        digitalWrite(motorPins[i], HIGH);
      }
    }
  }
}

