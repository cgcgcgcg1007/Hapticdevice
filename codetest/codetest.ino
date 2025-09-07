#include <Arduino.h>

// Motor pins
const int MOTOR_TOP    = 16;
const int MOTOR_BOTTOM = 25;
const int MOTOR_LEFT   = 27;
const int MOTOR_RIGHT  = 17;

void setupMotors() {
  pinMode(MOTOR_TOP, OUTPUT);
  pinMode(MOTOR_BOTTOM, OUTPUT);
  pinMode(MOTOR_LEFT, OUTPUT);
  pinMode(MOTOR_RIGHT, OUTPUT);
  vibrate(0,0,0,0);
}

void vibrate(int top, int bottom, int left, int right) {
  digitalWrite(MOTOR_TOP,    top);
  digitalWrite(MOTOR_BOTTOM, bottom);
  digitalWrite(MOTOR_LEFT,   left);
  digitalWrite(MOTOR_RIGHT,  right);
}

// =========================
unsigned long startPeriod = 0;
unsigned long lastChange  = 0;
bool isVibrating = false;
int currentDir = 0;

// Chu kỳ ban đầu (s)
float t_rung0 = 0.4;
float n_rest0 = 0.4;

// Thời gian chu kỳ tổng (9s)
const unsigned long totalCycle = 9000;

void setDirection(int dir) {
  switch (dir) {
    case 0: vibrate(0,1,0,0); Serial.println("Rung: BOTTOM"); break;
    case 1: vibrate(0,1,1,0); Serial.println("Rung: BOTTOM + LEFT"); break;
    case 2: vibrate(0,0,1,0); Serial.println("Rung: LEFT"); break;
    case 3: vibrate(1,0,1,0); Serial.println("Rung: TOP + LEFT"); break;
    case 4: vibrate(1,0,0,0); Serial.println("Rung: TOP"); break;
    case 5: vibrate(1,0,0,1); Serial.println("Rung: TOP + RIGHT"); break;
    case 6: vibrate(0,0,0,1); Serial.println("Rung: RIGHT"); break;
    case 7: vibrate(0,1,0,1); Serial.println("Rung: BOTTOM + RIGHT"); break;
  }
}

void setup() {
  Serial.begin(115200);
  randomSeed(analogRead(0));
  setupMotors();
  startPeriod = millis();
  lastChange = startPeriod;
  currentDir = random(0,8);
  Serial.printf("Start dir %d\n", currentDir);
}

void loop() {
  unsigned long now = millis();
  unsigned long elapsed = now - startPeriod;

  // Tỉ lệ 0->1 trong 9s
  float ratio = constrain((float)elapsed / totalCycle, 0.0f, 1.0f);

  // Giảm theo hàm mũ
  float t_rung_now = t_rung0 * pow(0.2, ratio);
  float n_rest_now = n_rest0 * pow(0.2, ratio);

  unsigned long duration = isVibrating ? (unsigned long)(t_rung_now * 1000)
                                       : (unsigned long)(n_rest_now * 1000);

  if (now - lastChange >= duration) {
    isVibrating = !isVibrating;
    lastChange = now;

    if (isVibrating) {
      setDirection(currentDir);
    } else {
      vibrate(0,0,0,0);
      Serial.println("Ngừng rung");
    }
  }

  // Sau 9s, chọn hướng mới và reset thời gian
  if (elapsed >= totalCycle) {
    startPeriod = now;
    currentDir = random(0,8);
    Serial.printf("New dir %d\n", currentDir);
  }
}
