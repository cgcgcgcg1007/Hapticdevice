#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <ArduinoJson.h>

const char* WIFI_SSID = "Oooo";
const char* WIFI_PASS = "12345678";
const char* PC_IP = "26.252.42.15";
const uint16_t PC_PORT  = 12345;
const uint16_t ESP_PORT = 12346;
WiFiUDP udp;

#define LED_PIN 4

int motorPins[6] = {16, 32, 19, 27, 13, 26};

int directions[12][2] = {
  {16, -1}, {16, 32}, {32, -1}, {32, 19},
  {19, -1}, {19, 27}, {27, -1}, {27, 13},
  {13, -1}, {13, 26}, {26, -1}, {26, 16}
};

int brightnessIndex = 2;    // không còn dùng cho LED

void stopMotors() {
  for (int i = 0; i < 6; ++i) digitalWrite(motorPins[i], LOW);
}

void startMotorsForDir(int dir) {
  stopMotors();
  if (dir < 0 || dir >= 12) return;
  int p1 = directions[dir][0];
  int p2 = directions[dir][1];
  if (p1 != -1) digitalWrite(p1, HIGH);
  if (p2 != -1) digitalWrite(p2, HIGH);
}

void vibrateAll(bool on) {
  for (int i = 0; i < 6; ++i) digitalWrite(motorPins[i], on ? HIGH : LOW);
}

void setup() {
  Serial.begin(115200);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) delay(200);

  udp.begin(ESP_PORT);

  // Motor setup
  for (int i = 0; i < 6; i++) {
    pinMode(motorPins[i], OUTPUT);
    digitalWrite(motorPins[i], LOW);
  }

  // Replace LED PWM with normal digital output
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
}

void handleIncomingUDP() {
  int packetSize = udp.parsePacket();
  if (!packetSize) return;

  char inBuf[256];
  int len = udp.read(inBuf, sizeof(inBuf) - 1);
  if (len <= 0) return;
  inBuf[len] = 0;

  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, inBuf)) return;

  const char* cmd = doc["cmd"];
  if (!cmd) return;

  if (strcmp(cmd, "direction") == 0) {
    int d = doc["dir"];
    if (d == -1) vibrateAll(true);
    else startMotorsForDir(d);
  }

  else if (strcmp(cmd, "stop") == 0) {
    stopMotors();
  }

  else if (strcmp(cmd, "refuse") == 0) {
    stopMotors();
  }

  // NEW — simple LED on/off
  else if (strcmp(cmd, "danger_on") == 0) {
    digitalWrite(LED_PIN, HIGH);
  }
  else if (strcmp(cmd, "danger_off") == 0) {
    digitalWrite(LED_PIN, LOW);
  }
}

void loop() {
  handleIncomingUDP();
  delay(5);
}
