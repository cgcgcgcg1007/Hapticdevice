#include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "Nguyễn Hải Trung";
const char* password = "12345678910";

WiFiUDP udp;
const int UDP_PORT = 23456;

char packetBuffer[256];

void setup() {
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }

  Serial.println("\nESP32 connected!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  udp.begin(UDP_PORT);
  Serial.print("Listening UDP on port ");
  Serial.println(UDP_PORT);
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1);
    if (len > 0) packetBuffer[len] = 0;

    Serial.print("UDP RECEIVED: ");
    Serial.println(packetBuffer);
  }
}
