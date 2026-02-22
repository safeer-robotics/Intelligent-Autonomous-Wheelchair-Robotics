finalcode..
/************************************************************
  Smart Wheelchair Prototype
  Manual mode (joystick) + Auto obstacle avoidance
  E-Brake works in any mode (button + safety latch)
  Crash/tilt detection using MPU6050
************************************************************/

#define BLYNK_PRINT Serial

#define BLYNK_TEMPLATE_ID   "YOUR_BLYNK_TEMPLATE_ID"
#define BLYNK_TEMPLATE_NAME "YOUR_BLYNK_TEMPLATE_NAME"
#define BLYNK_AUTH_TOKEN    "YOUR_BLYNK_AUTH_TOKEN"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <math.h>

// WiFi credentials
char ssid[] = "YOUR_WIFI_SSID";
char pass[] = "YOUR_WIFI_PASSWORD";

// Motor pins (L298N)
#define ENA 25
#define IN1 26
#define IN2 27
#define ENB 33
#define IN3 14
#define IN4 12

// Ultrasonic sensors
#define TRIG_F 5
#define ECHO_F 34
#define TRIG_L 18
#define ECHO_L 35
#define TRIG_R 19
#define ECHO_R 32

// MPU6050 I2C
#define SDA_PIN 21
#define SCL_PIN 22
#define MPU_ADDR 0x68

// Blynk virtual pins
#define VP_MODE       V0
#define VP_JOY_X      V1
#define VP_JOY_Y      V2
#define VP_AUTO_LED   V3
#define VP_EBRAKE     V4
#define VP_MANUAL_LED V5
#define VP_ALERTS     V20

// PWM settings
const int PWM_FREQ = 1000;
const int PWM_RES  = 8;

// Control variables
volatile int modeAuto = 0;
volatile int joyX = 0;
volatile int joyY = 0;
volatile int ebrakeCmd = 0;
bool accidentLatched = false;

// Manual mode settings
const int DEADZONE = 15;
const int MANUAL_SPEED = 180;

// Motor smoothing
int currentLeftSpeed = 0;
int currentRightSpeed = 0;
const int ACCEL_STEP = 18;

// Auto mode parameters
int SPEED_FWD   = 140;
int SPEED_TURN  = 140;
int SPEED_REV   = 120;

int FRONT_SLOW_CM = 55;
int FRONT_STOP_CM = 25;
int SIDE_CLEAR_CM = 45;
int SIDE_HARD_CM  = 30;

int REVERSE_MS = 350;
int TURN_MS    = 420;

// MPU detection thresholds
const float MANUAL_FLIP_TILT_DEG = 68.0;
const int   MANUAL_FLIP_HITS     = 12;
int manualFlipHits = 0;

float prevAccelMag = 1.0f;

// Alert handling
unsigned long lastAlertMs = 0;
bool alertActive = false;

// Event cooldowns (prevent spam)
unsigned long lastAccidentEventMs = 0;
unsigned long lastEbrakeEventMs   = 0;
const unsigned long EVENT_COOLDOWN_MS = 4000;

enum AccidentType : uint8_t { ACC_NONE = 0, ACC_TILT = 1, ACC_CRASH = 2 };
AccidentType lastAccidentType = ACC_NONE;

// Motor control functions
void stopMotors() {
  for (int i = 0; i < 3; i++) {
    int left  = currentLeftSpeed  * (2 - i) / 3;
    int right = currentRightSpeed * (2 - i) / 3;

    digitalWrite(IN1, left >= 0);
    digitalWrite(IN2, left < 0);
    digitalWrite(IN3, right >= 0);
    digitalWrite(IN4, right < 0);

    ledcWrite(ENA, abs(left));
    ledcWrite(ENB, abs(right));
    delay(15);
  }

  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); 
  digitalWrite(IN4, LOW);
  ledcWrite(ENA, 0);
  ledcWrite(ENB, 0);

  currentLeftSpeed = 0;
  currentRightSpeed = 0;
}

void setMotorSmooth(int targetLeft, int targetRight) {
  targetLeft  = constrain(targetLeft,  -255, 255);
  targetRight = constrain(targetRight, -255, 255);

  if (currentLeftSpeed < targetLeft)  
    currentLeftSpeed = min(currentLeftSpeed + ACCEL_STEP, targetLeft);
  if (currentLeftSpeed > targetLeft)  
    currentLeftSpeed = max(currentLeftSpeed - ACCEL_STEP, targetLeft);

  if (currentRightSpeed < targetRight) 
    currentRightSpeed = min(currentRightSpeed + ACCEL_STEP, targetRight);
  if (currentRightSpeed > targetRight) 
    currentRightSpeed = max(currentRightSpeed - ACCEL_STEP, targetRight);

  digitalWrite(IN1, currentLeftSpeed >= 0);
  digitalWrite(IN2, currentLeftSpeed < 0);
  digitalWrite(IN3, currentRightSpeed >= 0);
  digitalWrite(IN4, currentRightSpeed < 0);

  ledcWrite(ENA, abs(currentLeftSpeed));
  ledcWrite(ENB, abs(currentRightSpeed));
}

void setMotorDirect(int left, int right) {
  left  = constrain(left,  -255, 255);
  right = constrain(right, -255, 255);

  digitalWrite(IN1, left >= 0);
  digitalWrite(IN2, left < 0);
  digitalWrite(IN3, right >= 0);
  digitalWrite(IN4, right < 0);

  ledcWrite(ENA, abs(left));
  ledcWrite(ENB, abs(right));

  currentLeftSpeed = left;
  currentRightSpeed = right;
}

// Ultrasonic distance measurement
long readCM(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long duration = pulseIn(echo, HIGH, 25000);
  if (duration == 0) return 400;

  long cm = (long)(duration * 0.0343 / 2.0);
  if (cm < 2) cm = 400;
  if (cm > 400) cm = 400;
  
  return cm;
}

long readCMAverage(int trig, int echo, int samples = 2) {
  long total = 0;
  for (int i = 0; i < samples; i++) {
    total += readCM(trig, echo);
    if (i < samples - 1) delay(15);
  }
  return total / samples;
}

// Alert management
void showAlert(const String &msg) {
  Blynk.virtualWrite(VP_ALERTS, msg);
  lastAlertMs = millis();
  alertActive = (msg.length() > 0);
}

void clearAlert() {
  Blynk.virtualWrite(VP_ALERTS, "");
  alertActive = false;
  lastAlertMs = 0;
}

void sendAccidentEvent(const String &msg) {
  unsigned long now = millis();
  if (now - lastAccidentEventMs >= EVENT_COOLDOWN_MS) {
    Blynk.logEvent("accident", msg);
    lastAccidentEventMs = now;
    Serial.println("[EVENT] " + msg);
  }
}

void sendEbrakeEvent(const String &msg) {
  unsigned long now = millis();
  if (now - lastEbrakeEventMs >= EVENT_COOLDOWN_MS) {
    Blynk.logEvent("ebrake", msg);
    lastEbrakeEventMs = now;
    Serial.println("[EVENT] " + msg);
  }
}

// Read acceleration from MPU6050
void readAccelG(float &AX, float &AY, float &AZ) {
  int16_t ax, ay, az;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();

  AX = ax / 16384.0f;
  AY = ay / 16384.0f;
  AZ = az / 16384.0f;
}

// Manual mode: detect only severe tilts
bool detectManualFlip() {
  float AX, AY, AZ;
  readAccelG(AX, AY, AZ);

  float amag = sqrt(AX * AX + AY * AY + AZ * AZ);
  if (amag < 0.10f) amag = 0.10f;

  float tiltDeg = acos(constrain(AZ / amag, -1.0f, 1.0f)) * 57.2958f;

  if (tiltDeg >= MANUAL_FLIP_TILT_DEG) 
    manualFlipHits++;
  else if (manualFlipHits > 0) 
    manualFlipHits--;

  if (manualFlipHits >= MANUAL_FLIP_HITS) {
    lastAccidentType = ACC_TILT;
    return true;
  }
  return false;
}

// Auto mode: detect crashes and tilts
bool detectAutoAccident() {
  int16_t ax, ay, az, gx, gy, gz;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();

  float AX = ax / 16384.0f;
  float AY = ay / 16384.0f;
  float AZ = az / 16384.0f;

  float accelMag = sqrt(AX * AX + AY * AY + AZ * AZ);
  if (accelMag < 0.05f) accelMag = 0.05f;

  float tilt = acos(constrain(AZ / accelMag, -1.0f, 1.0f)) * 57.3f;

  float gxs = fabs(gx / 131.0f);
  float gys = fabs(gy / 131.0f);
  float gzs = fabs(gz / 131.0f);
  float gyroMax = max(gxs, max(gys, gzs));

  float accelChange = fabs(accelMag - prevAccelMag);
  prevAccelMag = accelMag;

  bool tipped      = (tilt > 52.0f);
  bool impact      = (accelMag > 3.4f);
  bool gyroSpike   = (gyroMax > 340.0f);
  bool suddenCrash = (accelChange > 1.8f) && (accelMag > 1.6f);

  if (tipped) { 
    lastAccidentType = ACC_TILT; 
    return true; 
  }
  if (impact || gyroSpike || suddenCrash) { 
    lastAccidentType = ACC_CRASH; 
    return true; 
  }

  lastAccidentType = ACC_NONE;
  return false;
}

// Update mode LEDs
void updateModeLEDs() {
  if (modeAuto == 0) {
    Blynk.virtualWrite(VP_MANUAL_LED, 1);
    Blynk.virtualWrite(VP_AUTO_LED, 0);
  } else {
    Blynk.virtualWrite(VP_MANUAL_LED, 0);
    Blynk.virtualWrite(VP_AUTO_LED, 1);
  }
}

// Blynk callbacks
BLYNK_WRITE(VP_MODE) {
  modeAuto = param.asInt();
  updateModeLEDs();
  manualFlipHits = 0;
  lastAccidentType = ACC_NONE;
}

BLYNK_WRITE(VP_JOY_X) { 
  joyX = param.asInt(); 
}

BLYNK_WRITE(VP_JOY_Y) { 
  joyY = param.asInt(); 
}

BLYNK_WRITE(VP_EBRAKE) {
  ebrakeCmd = param.asInt();

  if (ebrakeCmd == 0) {
    accidentLatched = false;
    manualFlipHits = 0;
    lastAccidentType = ACC_NONE;
    clearAlert();
    Serial.println("E-Brake released");
  } else {
    showAlert("E-BRAKE ENGAGED");
    sendEbrakeEvent("E-Brake engaged manually");
  }
}

BLYNK_CONNECTED() {
  Blynk.syncVirtual(VP_MODE, VP_EBRAKE);
  updateModeLEDs();
}

// Manual driving mode
void runManualMode() {
  int x = (abs(joyX) < DEADZONE) ? 0 : joyX;
  int y = (abs(joyY) < DEADZONE) ? 0 : joyY;

  if (x == 0 && y == 0) {
    stopMotors();
    return;
  }

  int forward = map(y, -100, 100, -MANUAL_SPEED, MANUAL_SPEED);
  int turn    = map(x, -100, 100, -MANUAL_SPEED, MANUAL_SPEED);

  setMotorSmooth(forward + turn, forward - turn);
}

// Autonomous navigation
void runAutoMode() {
  long dF = readCM(TRIG_F, ECHO_F);
  delay(25);
  long dL = readCMAverage(TRIG_L, ECHO_L);
  delay(25);
  long dR = readCMAverage(TRIG_R, ECHO_R);

  // Front obstacle - stop and maneuver
  if (dF <= FRONT_STOP_CM) {
    stopMotors();
    delay(80);

    setMotorDirect(-SPEED_REV, -SPEED_REV);
    delay(REVERSE_MS);
    stopMotors();
    delay(80);

    long dF2 = readCM(TRIG_F, ECHO_F);
    delay(25);
    long dL2 = readCMAverage(TRIG_L, ECHO_L);
    delay(25);
    long dR2 = readCMAverage(TRIG_R, ECHO_R);

    if (dL2 > dR2) 
      setMotorDirect(-SPEED_TURN, SPEED_TURN);
    else           
      setMotorDirect(SPEED_TURN, -SPEED_TURN);

    delay(TURN_MS);
    stopMotors();

    setMotorSmooth(SPEED_FWD, SPEED_FWD);
    delay(120);
    return;
  }

  // Slow down when approaching obstacle
  if (dF <= FRONT_SLOW_CM) {
    setMotorSmooth(SPEED_FWD - 25, SPEED_FWD - 25);
    delay(30);
    return;
  }

  // Side obstacle avoidance
  int leftPWM  = SPEED_FWD;
  int rightPWM = SPEED_FWD;

  if (dL <= SIDE_HARD_CM) {
    leftPWM  = SPEED_FWD;
    rightPWM = SPEED_FWD - 70;
  } else if (dR <= SIDE_HARD_CM) {
    leftPWM  = SPEED_FWD - 70;
    rightPWM = SPEED_FWD;
  } else if (dL <= SIDE_CLEAR_CM) {
    leftPWM  = SPEED_FWD;
    rightPWM = SPEED_FWD - 40;
  } else if (dR <= SIDE_CLEAR_CM) {
    leftPWM  = SPEED_FWD - 40;
    rightPWM = SPEED_FWD;
  }

  setMotorSmooth(leftPWM, rightPWM);
  delay(25);
}

void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  pinMode(TRIG_F, OUTPUT); pinMode(ECHO_F, INPUT);
  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);

  ledcAttach(ENA, PWM_FREQ, PWM_RES);
  ledcAttach(ENB, PWM_FREQ, PWM_RES);

  stopMotors();

  // Initialize MPU6050
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  delay(100);

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  clearAlert();
}

void loop() {
  Blynk.run();

  // Check for accidents
  bool accidentNow = false;
  if (modeAuto == 0) {
    accidentNow = detectManualFlip();
  } else {
    accidentNow = detectAutoAccident();
  }

  // Handle accident detection
  if (!accidentLatched && accidentNow) {
    accidentLatched = true;
    ebrakeCmd = 1;
    stopMotors();
    Blynk.virtualWrite(VP_EBRAKE, 1);
    
    if (lastAccidentType == ACC_TILT) {
      showAlert("TILT DETECTED - RESET E-BRAKE");
      sendAccidentEvent("Wheelchair tilt detected");
    } else {
      showAlert("CRASH DETECTED - RESET E-BRAKE");
      sendAccidentEvent("Crash detected");
    }
    
    sendEbrakeEvent("E-Brake auto engaged");
  }

  // E-brake overrides all movement
  if (ebrakeCmd || accidentLatched) {
    stopMotors();
    return;
  }

  // Normal operation
  if (modeAuto == 0) {
    runManualMode();
  } else {
    runAutoMode();
  }
}
