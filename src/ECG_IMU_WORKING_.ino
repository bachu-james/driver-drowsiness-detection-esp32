#include <Wire.h>

// ---------------- MPU6050 addresses ----------------
#define MPU_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define ACCEL_XOUT_H 0x3B

// ---------------- Pin setup ----------------
const int LED_TILT = 2;
const int LED_ECG  = 4;
const int BUZZER   = 5;
const int ECG_PIN = 34;
const int LO_PLUS_PIN = 25;
const int LO_MINUS_PIN = 26;
const bool USE_LO_PINS = true;
const int MOTOR1_PIN = 18;
const int MOTOR2_PIN = 19;

// ---------------- Thresholds ----------------
const float ANGLE_THRESHOLD_DEG = 20.0;
const int ECG_MIN_AMP = 60;
const unsigned long ECG_WINDOW_MS = 300;

// ---------------- Variables ----------------
unsigned long ecgWindowStart = 0;
int ecgMin = 4095, ecgMax = 0;
float baseX = 0.0, baseY = 0.0, baseZ = 1.0;
bool mpuConnected = false;

// ---------------- Helper functions ----------------
void i2cWriteByte(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

bool checkMPUConnection() {
  Wire.beginTransmission(MPU_ADDR);
  return (Wire.endTransmission() == 0);
}

bool readMPUAccelRaw(int16_t &ax, int16_t &ay, int16_t &az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) {
    return false; // communication failed
  }

  if (Wire.requestFrom(MPU_ADDR, 6, true) != 6) {
    return false; // didn’t get 6 bytes
  }

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  return true;
}

void normalizeVec(float &x, float &y, float &z) {
  float mag = sqrt(x*x + y*y + z*z);
  if (mag > 0.0001) { x /= mag; y /= mag; z /= mag; }
}

void calibrateBaseline(unsigned int samples = 200, unsigned int delayMs = 5) {
  long sumX = 0, sumY = 0, sumZ = 0;
  for (unsigned int i = 0; i < samples; ++i) {
    int16_t ax, ay, az;
    if (!readMPUAccelRaw(ax, ay, az)) {
      Serial.println("⚠️ Calibration failed: MPU not responding");
      return;
    }
    sumX += ax; sumY += ay; sumZ += az;
    delay(delayMs);
  }
  const float accelScale = 16384.0;
  float avgX = (float)sumX / samples / accelScale;
  float avgY = (float)sumY / samples / accelScale;
  float avgZ = (float)sumZ / samples / accelScale;
  normalizeVec(avgX, avgY, avgZ);
  baseX = avgX; baseY = avgY; baseZ = avgZ;
  Serial.printf("✅ Calibrated baseline: bx=%.3f by=%.3f bz=%.3f\n", baseX, baseY, baseZ);
}

float angleFromBaseline(float ax, float ay, float az) {
  normalizeVec(ax, ay, az);
  float dot = baseX*ax + baseY*ay + baseZ*az;
  dot = constrain(dot, -1.0, 1.0);
  return acos(dot) * 180.0 / PI;
}

// Reconnect MPU if disconnected
void reconnectMPU() {
  Serial.println("🔌 Reconnecting to MPU6050...");
  for (int i = 0; i < 5; i++) {
    if (checkMPUConnection()) {
      i2cWriteByte(PWR_MGMT_1, 0x00);
      Serial.println("✅ MPU reconnected, recalibrating...");
      calibrateBaseline();
      mpuConnected = true;
      return;
    }
    delay(500);
  }
  Serial.println("❌ MPU not found after retries!");
  mpuConnected = false;
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  pinMode(LED_TILT, OUTPUT);
  pinMode(LED_ECG, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(MOTOR1_PIN, OUTPUT);
  pinMode(MOTOR2_PIN, OUTPUT);

  if (USE_LO_PINS) {
    pinMode(LO_PLUS_PIN, INPUT_PULLUP);
    pinMode(LO_MINUS_PIN, INPUT_PULLUP);
  }

  // Try connecting to MPU
  if (checkMPUConnection()) {
    Serial.println("✅ MPU6050 connected");
    i2cWriteByte(PWR_MGMT_1, 0x00); // wake up
    calibrateBaseline();
    mpuConnected = true;
  } else {
    Serial.println("❌ MPU6050 not found, will retry automatically.");
    mpuConnected = false;
  }

  ecgWindowStart = millis();
}

// ---------------- Main Loop ----------------
void loop() {
  bool tiltAlert = false;
  bool ecgAlert = false;

  // ---- MPU Connection Check ----
  if (!checkMPUConnection()) {
    if (mpuConnected) {
      Serial.println("⚠️ MPU disconnected!");
      mpuConnected = false;
    }
    reconnectMPU();
  }

  // ---- Read MPU6050 acceleration ----
  if (mpuConnected) {
    int16_t axRaw, ayRaw, azRaw;
    if (readMPUAccelRaw(axRaw, ayRaw, azRaw)) {
      const float accelScale = 16384.0;
      float Ax = axRaw / accelScale;
      float Ay = ayRaw / accelScale;
      float Az = azRaw / accelScale;
      float angle = angleFromBaseline(Ax, Ay, Az);
      if (angle > ANGLE_THRESHOLD_DEG) tiltAlert = true;
    } else {
      Serial.println("⚠️ Failed to read MPU, retrying...");
      reconnectMPU();
    }
  }

  // =======================================
  // ✅ ECG CHECK — ONLY ELECTRODE REMOVAL TRIGGERS ALERT
  // =======================================
  if (USE_LO_PINS) {
    int loPlus = digitalRead(LO_PLUS_PIN);
    int loMinus = digitalRead(LO_MINUS_PIN);

    if (loPlus == HIGH || loMinus == HIGH) {
      ecgAlert = true;
      Serial.println("⚠️ ECG LED OFF - Electrodes Removed!");
      digitalWrite(LED_ECG, LOW);
    } else {
      ecgAlert = false;
      digitalWrite(LED_ECG, HIGH);
    }
  }

  // ====================================
  // 🚨 ALERT MANAGEMENT SECTION (NORMAL BUZZER)
  // ====================================
  if (tiltAlert && ecgAlert) {
    Serial.println("🚨 ALERT: Head Slanted + ECG Problem!");
    digitalWrite(LED_TILT, HIGH);
    digitalWrite(LED_ECG, HIGH);
    digitalWrite(MOTOR1_PIN, LOW);
    digitalWrite(MOTOR2_PIN, LOW);
    digitalWrite(BUZZER, HIGH); // steady buzzer ON
  } 
  else if (tiltAlert) {
    Serial.println("⚠️ Head Slanted!");
    digitalWrite(LED_TILT, HIGH);
    digitalWrite(LED_ECG, LOW);
    digitalWrite(MOTOR1_PIN, LOW);
    digitalWrite(MOTOR2_PIN, LOW);
    digitalWrite(BUZZER, HIGH); // steady buzzer ON
  } 
  else if (ecgAlert) {
    Serial.println("⚠️ ECG Lead Off!");
    digitalWrite(LED_ECG, LOW);
    digitalWrite(LED_TILT, HIGH);
    digitalWrite(MOTOR1_PIN, LOW);
    digitalWrite(MOTOR2_PIN, LOW);
    digitalWrite(BUZZER, HIGH); // steady buzzer ON
  } 
  else {
    digitalWrite(LED_TILT, LOW);
    digitalWrite(LED_ECG, HIGH); // ECG connected → LED ON
    digitalWrite(MOTOR1_PIN, HIGH);
    digitalWrite(MOTOR2_PIN, HIGH);
    digitalWrite(BUZZER, LOW); // buzzer OFF
  }

  delay(100);
}
