#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <ArduinoJson.h>

// WiFi cấu hình
const char* WIFI_SSID = "Oooo";
const char* WIFI_PASS = "12345678";
const char* PC_IP = "172.24.198.128";  

// UDP ports
const uint16_t PC_PORT  = 12345; // PC lắng nghe ESP
const uint16_t ESP_PORT = 12346; // ESP lắng nghe PC
WiFiUDP udp;

// MPU6050
const int MPU_ADDR = 0x68;
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float pitch = 0.0f, roll = 0.0f;
unsigned long lastTime = 0;
const float alpha = 0.98f;

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
  digitalWrite(MOTOR_TOP, LOW);
  digitalWrite(MOTOR_BOTTOM, LOW);
  digitalWrite(MOTOR_LEFT, LOW);
  digitalWrite(MOTOR_RIGHT, LOW);
}

void vibrate(int top, int bottom, int left, int right) {
  digitalWrite(MOTOR_TOP,    top);
  digitalWrite(MOTOR_BOTTOM, bottom);
  digitalWrite(MOTOR_LEFT,   left);
  digitalWrite(MOTOR_RIGHT,  right);
}

void setupMPU() {
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0); Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); Wire.write(0x00); Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); Wire.write(0x00); Wire.endTransmission(true);
}

void readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // temp
  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();

  accelX = ax / 16384.0;
  accelY = ay / 16384.0;
  accelZ = az / 16384.0;
  gyroX = gx / 131.0;
  gyroY = gy / 131.0;
  gyroZ = gz / 131.0;
}

// =================== CONTROL ===================
int currentDir = -1;
bool isVibrating = false;
unsigned long lastVibrate = 0;
int curOn = 200, curOff = 100;

void setDirection(int dir) {
  switch (dir) {
    case 0: vibrate(0,1,0,0); Serial.println("➡ Bottom"); break;
    case 1: vibrate(0,1,1,0); Serial.println("↘ Bottom-Left"); break;
    case 2: vibrate(0,0,1,0); Serial.println("⬅ Left"); break;
    case 3: vibrate(1,0,1,0); Serial.println("↖ Top-Left"); break;
    case 4: vibrate(1,0,0,0); Serial.println("⬆ Top"); break;
    case 5: vibrate(1,0,0,1); Serial.println("↗ Top-Right"); break;
    case 6: vibrate(0,0,0,1); Serial.println("➡ Right"); break;
    case 7: vibrate(0,1,0,1); Serial.println("↙ Bottom-Right"); break;
  }
}

void processCoords(float x, float y, float z) {
  float r = sqrt(x*x + y*y);

  // Ở giữa
  if (r < 0.13) {
    vibrate(1,1,1,1);
    Serial.println("🔔 Center - all motors ON");
    return;
  }

  // xác định góc → hướng
  float angle = atan2(y, x) * 180.0 / PI;
  if (angle < 0) angle += 360;

  if ((angle >= 337.5 || angle < 22.5)) currentDir = 0;       // bottom
  else if (angle >= 22.5 && angle < 67.5)  currentDir = 1;
  else if (angle >= 67.5 && angle < 112.5) currentDir = 2;    // left
  else if (angle >= 112.5 && angle < 157.5) currentDir = 3;
  else if (angle >= 157.5 && angle < 202.5) currentDir = 4;   // top
  else if (angle >= 202.5 && angle < 247.5) currentDir = 5;
  else if (angle >= 247.5 && angle < 292.5) currentDir = 6;   // right
  else if (angle >= 292.5 && angle < 337.5) currentDir = 7;

  // vùng xa: rung liên tục
  if (r >= 0.56) {
    setDirection(currentDir);
    isVibrating = false;
    Serial.println("🔁 Continuous vibration");
  }
  // vùng gần: rung theo chu kỳ
  else {
    float ratio = (r - 0.13f) / (0.56f - 0.13f); // chuẩn hóa 0..1
    curOn  = 200 + (int)(600 * ratio);  // 200 -> 800 ms
    curOff =  50 + (int)(150 * ratio);  //  50 -> 200 ms
    Serial.printf("⏱ Pulse vibration: ON=%d ms, OFF=%d ms\n", curOn, curOff);
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) { delay(300); Serial.print("."); }
  Serial.println();
  Serial.print("ESP IP: "); Serial.println(WiFi.localIP());

  udp.begin(ESP_PORT);
  setupMPU();
  setupMotors();
  lastTime = millis();
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0f;
  if (dt <= 0) dt = 0.01;
  lastTime = now;

  // Đọc MPU
  readMPU();
  float accelRoll  = atan2(accelY, accelZ) * 180.0 / PI;
  float accelPitch = atan2(-accelX, sqrt(accelY*accelY + accelZ*accelZ)) * 180.0 / PI;
  roll  = alpha * (roll  + gyroX * dt) + (1 - alpha) * accelRoll;
  pitch = alpha * (pitch + gyroY * dt) + (1 - alpha) * accelPitch;

  // Gửi pitch/roll tới PC
  char buf[128];
  int len = snprintf(buf, sizeof(buf),
                     "{\"pitch\":%.3f,\"roll\":%.3f,\"ts\":%lu}",
                     pitch, roll, millis());
  udp.beginPacket(PC_IP, PC_PORT);
  udp.write((uint8_t*)buf, len);
  udp.endPacket();

  // Nhận toạ độ từ PC
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char recvBuf[128];
    int rlen = udp.read(recvBuf, sizeof(recvBuf)-1);
    if (rlen > 0) {
      recvBuf[rlen] = 0;
      StaticJsonDocument<128> doc;
      if (!deserializeJson(doc, recvBuf)) {
        float x = doc["x"];
        float y = doc["y"];
        float z = doc["z"];
        processCoords(x, y, z);
      }
    }
  }

  // Điều khiển rung theo chu kỳ
  if (currentDir >= 0 && (now - lastVibrate) >= (isVibrating ? curOn : curOff)) {
    lastVibrate = now;
    if (isVibrating) {
      vibrate(0,0,0,0); // nghỉ
      isVibrating = false;
    } else {
      setDirection(currentDir); // rung hướng
      isVibrating = true;
    }
  }

  delay(50); // tần số cập nhật ~20 Hz
}
