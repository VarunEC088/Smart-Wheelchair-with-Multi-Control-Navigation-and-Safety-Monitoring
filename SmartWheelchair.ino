#include <Wire.h>
#include <MPU6050.h>
#include <TinyGPSPlus.h>
#include "BluetoothSerial.h"

// ===== MODULES =====
BluetoothSerial SerialBT;
TinyGPSPlus gps;
HardwareSerial SerialGPS(2);
MPU6050 mpu;

// ===== PINS =====
const int RPWM_R = 32, LPWM_R = 33, RPWM_L = 25, LPWM_L = 26;
const int JOY_X = 34, JOY_Y = 35, JOY_SW = 14;
const int TRIG_PIN = 18, ECHO_PIN = 19;
const int BUZZER_PIN = 4, LED_PIN = 2;

// ===== SPEED SETTINGS =====
int btSpeed = 45;          // Normal Bluetooth speed
int maxSpeed = 120;        // Max speed for joystick
const int BOOST_SPEED = 70;
const int PRECISION_SPEED = 30;

// ===== JOYSTICK CONFIG =====
int centerX = 2048, centerY = 2048;
const int deadzone = 600;

// ===== STATE =====
bool btActive = false, joystickActive = false;
bool alarmActive = false, buttonAlarmActive = false, fallDetected = false;
bool blinkState = false;
unsigned long lastBlink = 0;
unsigned long lastBTTime = 0, lastJoyTime = 0;
const unsigned long BT_TIMEOUT = 3000, JOY_TIMEOUT = 800;
unsigned long lastGPSread = 0, lastGPSsend = 0, lastFallCheck = 0, lastObstacleCheck = 0;
bool lastButtonState = HIGH;

// ===== SETUP =====
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  mpu.initialize();
  SerialBT.begin("ESP32_WHEELCHAIR");
  SerialGPS.begin(9600, SERIAL_8N1, 16, 17);

  pinMode(RPWM_R, OUTPUT); pinMode(LPWM_R, OUTPUT);
  pinMode(RPWM_L, OUTPUT); pinMode(LPWM_L, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT); pinMode(BUZZER_PIN, OUTPUT);
  pinMode(JOY_SW, INPUT_PULLUP);

  stopAll();

  long sumX = 0, sumY = 0;
  for (int i = 0; i < 30; i++) {
    sumX += analogRead(JOY_X);
    sumY += analogRead(JOY_Y);
    delay(10);
  }
  centerX = sumX / 30; centerY = sumY / 30;

  Serial.printf("🧭 Joystick centered: %d,%d\n", centerX, centerY);
  Serial.println("✅ System ready!");
}

// ===== LOOP =====
void loop() {
  handleBluetooth();
  handleJoystick();
  handleButton();
  handleGPS();

  if (millis() - lastFallCheck > 500) { detectFall(); lastFallCheck = millis(); }
  if (millis() - lastObstacleCheck > 200) { obstacleCheck(); lastObstacleCheck = millis(); }

  if (btActive && millis() - lastBTTime > BT_TIMEOUT) { btActive = false; stopAll(); }
  if (joystickActive && millis() - lastJoyTime > JOY_TIMEOUT) { joystickActive = false; stopAll(); }

  if (alarmActive) emergencySirenPattern();

  handleBlinkActivity();
}

// ===== LED BLINK INDICATOR =====
void handleBlinkActivity() {
  bool anyTask =
      btActive ||
      joystickActive ||
      alarmActive ||
      fallDetected ||
      buttonAlarmActive;

  if (anyTask) {
    if (millis() - lastBlink > 400) {
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);
      lastBlink = millis();
    }
  } else {
    digitalWrite(LED_PIN, LOW);
  }
}

// ===== MOTOR CONTROL =====
void setMotorPWM(int leftPWM, int rightPWM) {
  if (alarmActive) { stopAll(); return; }

  leftPWM = constrain(leftPWM, -maxSpeed, maxSpeed);
  rightPWM = constrain(rightPWM, -maxSpeed, maxSpeed);

  if (leftPWM >= 0) { analogWrite(RPWM_L, leftPWM); analogWrite(LPWM_L, 0); }
  else { analogWrite(RPWM_L, 0); analogWrite(LPWM_L, abs(leftPWM)); }

  if (rightPWM >= 0) { analogWrite(RPWM_R, rightPWM); analogWrite(LPWM_R, 0); }
  else { analogWrite(RPWM_R, 0); analogWrite(LPWM_R, abs(rightPWM)); }
}

void stopAll() {
  analogWrite(RPWM_R, 0); analogWrite(LPWM_R, 0);
  analogWrite(RPWM_L, 0); analogWrite(LPWM_L, 0);
}

// ===== JOYSTICK =====
void handleJoystick() {
  if (btActive || alarmActive) return;

  int x = analogRead(JOY_X) - centerX;
  int y = centerY - analogRead(JOY_Y);

  if (abs(x) < deadzone) x = 0;
  if (abs(y) < deadzone) y = 0;
  if (x == 0 && y == 0) { stopAll(); joystickActive = false; return; }

  joystickActive = true; lastJoyTime = millis();

  int pwmLeft = constrain(map(y - x, -2048, 2048, -maxSpeed, maxSpeed), -maxSpeed, maxSpeed);
  int pwmRight = constrain(map(y + x, -2048, 2048, -maxSpeed, maxSpeed), -maxSpeed, maxSpeed);

  setMotorPWM(pwmLeft, pwmRight);
}

// ===== BLUETOOTH CONTROL =====
void handleBluetooth() {
  while (SerialBT.available()) {
    char c = toupper(SerialBT.read());
    if (c == '\r' || c == '\n') continue;
    btActive = true; lastBTTime = millis(); joystickActive = false;

    if (alarmActive) { stopAll(); return; }

    switch (c) {
      case 'U': setMotorPWM(btSpeed, btSpeed); break;
      case 'D': setMotorPWM(-btSpeed, -btSpeed); break;
      case 'L': setMotorPWM(-btSpeed, btSpeed); break;
      case 'R': setMotorPWM(btSpeed, -btSpeed); break;
      case 'S': stopAll(); break;
      case 'G': sendGPSLocation(); break;

      case 'Q': setMotorPWM(btSpeed * 0.5, btSpeed); break;
      case 'E': setMotorPWM(btSpeed, btSpeed * 0.5); break;
      case 'Z': setMotorPWM(-btSpeed * 0.5, -btSpeed); break;
      case 'C': setMotorPWM(-btSpeed, -btSpeed * 0.5); break;

      case 'B': btSpeed = BOOST_SPEED;
                Serial.println("🚀 Boost Mode ON (70)");
                SerialBT.println("🚀 Boost Mode ON (70)"); break;
      case 'P': btSpeed = PRECISION_SPEED;
                Serial.println("🐢 Precision Mode ON (30)");
                SerialBT.println("🐢 Precision Mode ON (30)"); break;

      default:
        Serial.println("❌ Invalid BT command");
        SerialBT.println("❌ Invalid BT command");
        break;
    }
  }
}

// ===== FALL DETECTION =====
void detectFall() {
  if (buttonAlarmActive) return;

  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  float axg = ax / 16384.0, ayg = ay / 16384.0, azg = az / 16384.0;
  float totalAccel = sqrt(axg * axg + ayg * ayg + azg * azg);
  float tiltAngle = acos(azg / totalAccel) * 57.3;

  if ((tiltAngle > 50 || totalAccel < 0.5) && !alarmActive) {
    fallDetected = true; alarmActive = true;
    Serial.println("⚠ FALL DETECTED!");
    sendGPSLocation();
  }

  if (!buttonAlarmActive && alarmActive && tiltAngle < 20 && totalAccel > 0.8) {
    alarmOff(); Serial.println("✅ Upright restored");
  }
}

// ===== BUTTON =====
void handleButton() {
  bool btnState = digitalRead(JOY_SW);
  if (btnState == LOW && lastButtonState == HIGH) {
    buttonAlarmActive = !buttonAlarmActive;
    alarmActive = buttonAlarmActive;
    if (buttonAlarmActive) {
      Serial.println("🚨 EMERGENCY ALARM TRIGGERED");
      sendGPSLocation();
    } else {
      Serial.println("🔕 Emergency Alarm OFF");
      alarmOff();
    }
    delay(300);
  }
  lastButtonState = btnState;
}

// ===== OBSTACLE DETECTION =====
void obstacleCheck() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 25000);
  float distance = duration * 0.034 / 2;

  if (distance <= 0 || distance > 20) { noTone(BUZZER_PIN); return; }

  Serial.printf("📏 Distance: %.1f cm\n", distance);

  if (distance < 7) { stopAll(); tone(BUZZER_PIN, 3000); delay(100); }
  else if (distance < 15) { tone(BUZZER_PIN, 2000); delay(150); }
  else if (distance < 20) { tone(BUZZER_PIN, 1500); delay(250); }
  else noTone(BUZZER_PIN);
}

// ===== SIREN =====
void emergencySirenPattern() {
  static unsigned long lastChange = 0;
  static int toneFreq = 1500;
  static bool rising = true;

  unsigned long now = millis();
  if (now - lastChange > 50) {
    lastChange = now;
    toneFreq += rising ? 100 : -100;
    if (toneFreq >= 3000) rising = false;
    if (toneFreq <= 1500) rising = true;
    tone(BUZZER_PIN, toneFreq);
  }
}

void alarmOff() {
  alarmActive = false;
  fallDetected = false;
  noTone(BUZZER_PIN);
}

// ===== GPS =====
void handleGPS() {
  if (millis() - lastGPSread > 200) {
    while (SerialGPS.available()) gps.encode(SerialGPS.read());
    lastGPSread = millis();
  }
  if (millis() - lastGPSsend > 5000) {
    sendGPSLocation();
    lastGPSsend = millis();
  }
}

void sendGPSLocation() {
  if (gps.location.isValid()) {
    String loc = String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
    Serial.println("📡 GPS: " + loc);
    SerialBT.println("GPS:" + loc);
  } else {
    Serial.println("📡 GPS: No fix");
    SerialBT.println("GPS: No fix");
  }
}