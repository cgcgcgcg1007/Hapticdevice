#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <ArduinoJson.h>

// ================== WIFI ==================
const char* WIFI_SSID = "Oooo";
const char* WIFI_PASS = "12345678";
const char* PC_IP = "172.24.198.128";  // ⚠️ đổi theo IP máy tính

const uint16_t PC_PORT  = 12345; // PC lắng nghe ESP
const uint16_t ESP_PORT = 12346; // ESP lắng nghe PC
WiFiUDP udp;

// ================== MPU ==================
const int MPU_ADDR = 0x68;
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float pitch = 0.0f, roll = 0.0f;
unsigned long lastTime = 0;
const float alpha = 0.98f;

// ================== MOTOR ==================
const int MOTOR_TOP    = 16;//trong lòng
const int MOTOR_BOTTOM = 25;//đối diện
const int MOTOR_LEFT   = 27;//bên trái(khi tay ngửa)
const int MOTOR_RIGHT  = 17;//bên phải(khi tay ngửa)

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

void vibrateAll(bool on) {
  vibrate(on, on, on, on);
}

// ================== MPU FUNC ==================
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

// ================== RUNG LOGIC ==================
// Tham số biên độ rung
const float ON_MAX  = 0.8f;   // xa nhất: rung lâu
const float ON_MIN  = 0.2f;   // gần nhất: rung ngắn
const float OFF_MAX = 0.2f;   // xa nhất: nghỉ dài
const float OFF_MIN = 0.05f;  // gần nhất: nghỉ ngắn

// Trạng thái coords
unsigned long lastVibrate = 0;
bool isVibrating = false;
float curOn = 0.5, curOff = 0.2;
int currentDir = -1;

// Set hướng rung
void setDirection(int dir) {
  switch (dir) {
    case 0: vibrate(0,1,0,0); break; // bottom
    case 1: vibrate(0,1,1,0); break; // bottom + left
    case 2: vibrate(0,0,1,0); break; // left
    case 3: vibrate(1,0,1,0); break; // top + left
    case 4: vibrate(1,0,0,0); break; // top
    case 5: vibrate(1,0,0,1); break; // top + right
    case 6: vibrate(0,0,0,1); break; // right
    case 7: vibrate(0,1,0,1); break; // bottom + right
  }
}

// Tính hướng + tần số rung từ (x,y,z)
void processCoords(float x, float y, float z) {
  float r = sqrt(x*x + y*y);

  if (r < 0.13) {
    currentDir = -1;   // ở giữa
    curOn  = ON_MIN;
    curOff = OFF_MIN;
    return;
  }

  float ratio = constrain((r - 0.13) / 1.0f, 0.0f, 1.0f);
  curOn  = ON_MAX  - ratio * (ON_MAX  - ON_MIN);
  curOff = OFF_MAX - ratio * (OFF_MAX - OFF_MIN);

  float angle = atan2(y, x) * 180.0 / PI;
  if (angle < 0) angle += 360;

  if ((angle >= 337.5 && angle < 360) || (angle >= 0 && angle < 22.5)) currentDir = 0;
  else if (angle >= 22.5 && angle < 67.5) currentDir = 1;
  else if (angle >= 67.5 && angle < 112.5) currentDir = 2;
  else if (angle >= 112.5 && angle < 157.5) currentDir = 3;
  else if (angle >= 157.5 && angle < 202.5) currentDir = 4;
  else if (angle >= 202.5 && angle < 247.5) currentDir = 5;
  else if (angle >= 247.5 && angle < 292.5) currentDir = 6;
  else if (angle >= 292.5 && angle < 337.5) currentDir = 7;
}

// ================== LABEL RUNG ==================
const int VIB_DURATION = 1000; // ms
const int PAUSE_TIME   = 500;  // ms

void processLabel(int value) {
  for (int i = 0; i < value; i++) {
    vibrateAll(true);
    delay(VIB_DURATION);
    vibrateAll(false);
    if (i < value - 1) delay(PAUSE_TIME);
  }
}

// ================== SETUP ==================
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

// ================== LOOP ==================
void loop() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0f;
  if (dt <= 0) dt = 0.01;
  lastTime = now;

  // đọc MPU
  readMPU();
  float accelRoll  = atan2(accelY, accelZ) * 180.0 / PI;
  float accelPitch = atan2(-accelX, sqrt(accelY*accelY + accelZ*accelZ)) * 180.0 / PI;
  roll  = alpha * (roll  + gyroX * dt) + (1 - alpha) * accelRoll;
  pitch = alpha * (pitch + gyroY * dt) + (1 - alpha) * accelPitch;

  // gửi tới PC
  char buf[128];
  int len = snprintf(buf, sizeof(buf),
                     "{\"pitch\":%.3f,\"roll\":%.3f,\"ts\":%lu}",
                     pitch, roll, millis());
  udp.beginPacket(PC_IP, PC_PORT);
  udp.write((uint8_t*)buf, len);
  udp.endPacket();

  // nhận phản hồi từ PC
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char recvBuf[256];
    int rlen = udp.read(recvBuf, sizeof(recvBuf)-1);
    if (rlen > 0) {
      recvBuf[rlen] = 0;
      Serial.printf("Recv from PC: %s\n", recvBuf);

      StaticJsonDocument<256> doc;
      if (!deserializeJson(doc, recvBuf)) {
        const char* type = doc["type"];
        if (type && strcmp(type, "coords") == 0) {
          float x = doc["x"];
          float y = doc["y"];
          float z = doc["z"];
          processCoords(x, y, z);
        } 
        else if (type && strcmp(type, "label") == 0) {
          int value = doc["value"];
          processLabel(value);
        }
      }
    }
  }

  // điều khiển rung theo coords
  if (isVibrating) {
    if (now - lastVibrate >= (unsigned long)(curOn*1000)) {
      isVibrating = false;
      lastVibrate = now;
      vibrate(0,0,0,0);
    }
  } else {
    if (now - lastVibrate >= (unsigned long)(curOff*1000)) {
      isVibrating = true;
      lastVibrate = now;
      if (currentDir == -1) vibrateAll(true);
      else setDirection(currentDir);
    }
  }

  delay(20);
}
