#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

// =======================
// BUTTON PINS
// =======================
#define PIN_PLUS   17
#define PIN_MINUS  25

// =======================
// MOTOR PINS
// =======================
int motorPins[6] = {16, 32, 19, 27, 13, 26};

// =======================
int vibration_level = 3;
int currentMotor = 0;

// debounce
unsigned long debounceTime = 150;
unsigned long lastPlus = 0;
unsigned long lastMinus = 0;

// timing
unsigned long motorStart = 0;

// =======================
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

// =======================
void vibrateOne(int index) {
  int pwm = level_to_pwm(vibration_level);
  for (int i = 0; i < 6; i++) {
    analogWrite(motorPins[i], (i == index) ? pwm : 0);
  }
}

// =======================
void setup() {
  Serial.begin(115200);

  pinMode(PIN_PLUS, INPUT_PULLUP);
  pinMode(PIN_MINUS, INPUT_PULLUP);

  for (int i = 0; i < 6; i++) {
    pinMode(motorPins[i], OUTPUT);
    analogWrite(motorPins[i], 0);
  }

  Wire.begin(21, 22);
  if (!mpu.begin()) {
    Serial.println("❌ MPU6050 not found");
    while (1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  motorStart = millis();
  vibrateOne(currentMotor);
}

// =======================
void loop() {
  unsigned long now = millis();

  // ===== NÚT TĂNG =====
  if (digitalRead(PIN_PLUS) == LOW && now - lastPlus > debounceTime) {
    lastPlus = now;
    if (vibration_level < 5) vibration_level++;
    Serial.print("Level + : ");
    Serial.println(vibration_level);
    vibrateOne(currentMotor);
  }

  // ===== NÚT GIẢM =====
  if (digitalRead(PIN_MINUS) == LOW && now - lastMinus > debounceTime) {
    lastMinus = now;
    if (vibration_level > 1) vibration_level--;
    Serial.print("Level - : ");
    Serial.println(vibration_level);
    vibrateOne(currentMotor);
  }

  // ===== ĐỔI MOTOR SAU 4 GIÂY =====
  if (now - motorStart >= 4000) {
    motorStart = now;
    currentMotor = (currentMotor + 1) % 6;
    vibrateOne(currentMotor);

    Serial.print("→ Motor: ");
    Serial.println(currentMotor);
  }

  // ===== ĐỌC MPU6050 =====
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float pitch = atan2(
    a.acceleration.x,
    sqrt(a.acceleration.y * a.acceleration.y +
         a.acceleration.z * a.acceleration.z)
  ) * 180.0 / PI;

  float roll = atan2(
    a.acceleration.y,
    sqrt(a.acceleration.x * a.acceleration.x +
         a.acceleration.z * a.acceleration.z)
  ) * 180.0 / PI;

  Serial.print("Pitch: ");
  Serial.print(pitch, 2);
  Serial.print(" | Roll: ");
  Serial.print(roll, 2);
  Serial.print(" | Gyro Z: ");
  Serial.println(g.gyro.z, 2);

  delay(50);
}
