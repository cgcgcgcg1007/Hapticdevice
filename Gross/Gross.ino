#include <WiFi.h>
#include <WiFiUdp.h>

// ----------------------
// WiFi config
// ----------------------
const char* ssid     = "VIETTEL-Hung Tu";
const char* password = "19731973";

// ----------------------
// UDP config
// ----------------------
WiFiUDP udp;
const char* pythonIP = "192.168.1.9";
const int pythonPort = 12345;

// ----------------------
// Pin config
// ----------------------
#define PIN_REJECT 33
#define PIN_PLUS   17
#define PIN_MINUS  25
#define PIN_OUTPUT 4

// ----------------------
int vibration_level = 3;
unsigned long lastSend = 0;

// Debounce
unsigned long debounceTime = 150;
unsigned long lastPlus = 0;
unsigned long lastMinus = 0;
unsigned long lastReject = 0;

// ----------------------
// Reject state timer
// ----------------------
bool reject_active = false;
unsigned long reject_start_time = 0;

// ----------------------
int level_to_pwm(int lv) {
  switch (lv) {
    case 1: return 50;
    case 2: return 100;
    case 3: return 150;
    case 4: return 200;
    case 5: return 255;
  }
  return 0;
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_REJECT, INPUT_PULLUP);
  pinMode(PIN_PLUS, INPUT_PULLUP);
  pinMode(PIN_MINUS, INPUT_PULLUP);
  pinMode(PIN_OUTPUT, OUTPUT);

  analogWrite(PIN_OUTPUT, level_to_pwm(vibration_level));

  WiFi.begin(ssid, password);
  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");
  Serial.println(WiFi.localIP());
}

void loop() {
  unsigned long now = millis();

  // ==========================
  // NÚT TĂNG (tăng 1 lần mỗi lần bấm)
  // ==========================
  if (digitalRead(PIN_PLUS) == LOW && now - lastPlus > debounceTime) {
    lastPlus = now;  // chống lặp khi giữ nút

    if (vibration_level < 5) {
      vibration_level++;
      Serial.print("[PLUS] → Level tăng lên: ");
      Serial.println(vibration_level);
      analogWrite(PIN_OUTPUT, level_to_pwm(vibration_level));
    } else {
      Serial.println("[PLUS] Đã đạt mức tối đa!");
    }
  }

  // ==========================
  // NÚT GIẢM (giảm 1 lần mỗi lần bấm)
  // ==========================
  if (digitalRead(PIN_MINUS) == LOW && now - lastMinus > debounceTime) {
    lastMinus = now;

    if (vibration_level > 1) {
      vibration_level--;
      Serial.print("[MINUS] → Level giảm xuống: ");
      Serial.println(vibration_level);
      analogWrite(PIN_OUTPUT, level_to_pwm(vibration_level));
    } else {
      Serial.println("[MINUS] Đã đạt mức thấp nhất!");
    }
  }

  // ==========================
  // NÚT TỪ CHỐI → giữ state = 1 trong 0.5s
  // ==========================
  if (digitalRead(PIN_REJECT) == LOW && now - lastReject > debounceTime) {
    lastReject = now;

    reject_active = true;
    reject_start_time = now;

    Serial.println("[REJECT] Nhấn! → Gửi state = 1 trong 0.5s");
  }

  // Tự tắt reject sau 500ms
  if (reject_active && now - reject_start_time >= 500) {
    reject_active = false;
    Serial.println("[REJECT] Hết 0.5s → state = 0");
  }

  int state = reject_active ? 1 : 0;

  // ==========================
  // Gửi UDP mỗi 50ms
  // ==========================
  if (now - lastSend >= 50) {
    lastSend = now;

    String msg = "{\"pitch\":0, \"roll\":0, \"state\":" + String(state) +
                 ", \"ts\":" + String(now) + "}";

    udp.beginPacket(pythonIP, pythonPort);
    udp.print(msg);
    udp.endPacket();

    Serial.println("UDP → " + msg);
  }
}
