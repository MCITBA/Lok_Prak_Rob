/*
  Self-Balancing Robot - ESP32
  MPU6500 (I2C) + PID + L298N (Ohne Webserver)

  -------------------------------------------------------
  MPU6500 Wiring
  -------------------------------------------------------
  VCC  -> 3V3
  GND  -> GND
  SDA  -> GPIO23
  SCL  -> GPIO18
  NCS  -> 3V3   (aktiviert I2C Modus!)
  AD0  -> GND   (I2C Adresse 0x68)

  -------------------------------------------------------
  L298N Wiring
  -------------------------------------------------------
  ENA  -> GPIO26
  IN1  -> GPIO25
  IN2  -> GPIO27
  IN3  -> GPIO14
  IN4  -> GPIO33
  ENB  -> GPIO32
*/

#include <Wire.h>

// =======================================================
// Motor Pins
// =======================================================

const int ENA = 26;
const int IN1 = 25;
const int IN2 = 27;

const int IN3 = 14;
const int IN4 = 33;
const int ENB = 32;

// --- PROFIL 1: Niedrige Frequenz (Hohes Drehmoment) ---
const int PWM_FREQ       = 255;    
const int PWM_RESOLUTION = 8;       // 0-255
const int MAX_PWM        = 255;
const int MIN_START_PWM  = 75;     // Deine ermittelten 80 PWM auf dem Boden
const int MIN_HOLD_PWM   = 40;     // Deine ermittelte Mindest-Halte-PWM

// Motor-Trimm-Faktor
const float TRIM_LEFT  = 1.00f;  // 100% Leistung
const float TRIM_RIGHT = 1.00f;  // 100% Leistung

// =======================================================
// KICKSTART & RICHTUNGSWECHSEL LOGIK
// =======================================================
const int KICKSTART_CYCLES = 7;    // 10 Zyklen (ca. 20ms bei 200Hz)

int kickCounterLeft = 0;
int kickCounterRight = 0;

int lastSignLeft = 0; 
int lastSignRight = 0;

int actualPwmLeft = 0;
int actualPwmRight = 0;

// =======================================================
// MPU6500 Parameters
// =======================================================

#define MPU_ADDR  0x68
#define LOOP_HZ   200
#define LOOP_US   (1000000 / LOOP_HZ)  // 5000 us

const float COMPLEMENTARY_ALPHA = 0.98f;
const float GYRO_LP_ALPHA       = 0.8f;
const float ACCEL_MAG_HIGH      = 13.0f;
const float ACCEL_MAG_LOW       =  9.0f;
const int   CALIB_SAMPLES       = 500;

float pitchDeg         = 0.0f;
float gyroOffsetY      = 0.0f;
float gyroRateFiltered = 0.0f;

// Mechanischer Offset, um den physikalischen Schwerpunkt zu trimmen
float angleOffset      = -2.0f; 

// =======================================================
// PID Tunings (Hier direkt im Code anpassen!)
// =======================================================

float Kp = 25.0f;
float Ki = 0.2f;  
float Kd = 0.9f;

//float Kp = 27.0f;
//float Ki = 0.8f;  
//float Kd = 0.7f;

float setpointDeg   = 0.0f; // Da kein Joystick vorhanden, balanciert er fest auf 0°
float integral      = 0.0f;
float previousError = 0.0f;

const float INTEGRAL_LIMIT   = 300.0f;
const float FALL_ANGLE_LIMIT = 30.0f;

// =======================================================
// Timing
// =======================================================

unsigned long lastLoopMicros   = 0;
unsigned long lastSerialMillis = 0;

// =======================================================
// Motor Funktionen
// =======================================================

void setMotor(int ena, int in1, int in2, int command, int &kickCounter, int &lastSign, int &actualPwmStore) {
  command = constrain(command, -255, 255);
  int pwm = abs(command);

  // Aktuelles Vorzeichen bestimmen: +1 (vorwärts), -1 (rückwärts), 0 (stopp)
  int currentSign = (command > 0) ? 1 : ((command < 0) ? -1 : 0);

  // Wenn der Motor laut Regler komplett stehen soll
  if (pwm == 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(ena, 0);
    kickCounter = 0; 
    lastSign = 0;
    actualPwmStore = 0;
    return;
  }

  // --- KICKSTART BEI VORZEICHENWECHSEL / RICHTUNGSWECHSEL ---
  if (currentSign != lastSign && lastSign != 0) {
    kickCounter = 0; 
  }
  lastSign = currentSign; 

  // Deine Kickstart-Regelung
  if (pwm < MIN_START_PWM) {
    if (kickCounter < KICKSTART_CYCLES) {
      pwm = MIN_START_PWM; 
      kickCounter++; 
    } else {
      if (pwm < MIN_HOLD_PWM) {
        pwm = MIN_HOLD_PWM;
      }
    }
  } else {
    kickCounter = 0; 
  }

  pwm = constrain(pwm, 0, MAX_PWM);
  actualPwmStore = pwm;

  // Drehrichtung an die Hardware-Pins ausgeben
  if (command > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (command < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }

  ledcWrite(ena, pwm);
}

void setMotors(int command) {
  int commandLeft  = (int)(command * TRIM_LEFT);
  int commandRight = (int)(command * TRIM_RIGHT);

  setMotor(ENA, IN1, IN2, commandLeft, kickCounterLeft, lastSignLeft, actualPwmLeft);
  setMotor(ENB, IN3, IN4, commandRight, kickCounterRight, lastSignRight, actualPwmRight);
}

void stopMotors() {
  setMotors(0);
}

// =======================================================
// I2C Hilfsfunktionen
// =======================================================

void writeMPU(byte reg, byte val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void readAll(float &ax, float &ay, float &az,
             float &gx, float &gy, float &gz) {

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14);

  int16_t rawAx = (Wire.read() << 8) | Wire.read();
  int16_t rawAy = (Wire.read() << 8) | Wire.read();
  int16_t rawAz = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();
  int16_t rawGx = (Wire.read() << 8) | Wire.read();
  int16_t rawGy = (Wire.read() << 8) | Wire.read();
  int16_t rawGz = (Wire.read() << 8) | Wire.read();

  ax = (rawAx / 16384.0f) * 9.81f;
  ay = (rawAy / 16384.0f) * 9.81f;
  az = (rawAz / 16384.0f) * 9.81f;

  gx = rawGx / 65.5f;
  gy = rawGy / 65.5f;
  gz = rawGz / 65.5f;
}

// =======================================================
// Kalibrierung
// =======================================================

void calibrateGyro() {
  Serial.println("Kalibriere Gyro - Roboter stillhalten...");
  float sumY = 0.0f;

  for (int i = 0; i < CALIB_SAMPLES; i++) {
    float ax, ay, az, gx, gy, gz;
    readAll(ax, ay, az, gx, gy, gz);
    sumY += gy;
    delay(2);
  }

  gyroOffsetY = sumY / CALIB_SAMPLES;
  Serial.print("Gyro Offset Y: ");
  Serial.println(gyroOffsetY, 4);
  Serial.println("Kalibrierung fertig.");
}

// =======================================================
// Pitch berechnen
// =======================================================

void updatePitch(float dt,
                 float ax, float ay, float az,
                 float gy) {

  float accelPitch = atan2(-ax, az) * 180.0f / PI;
  float gyroRate   = gy - gyroOffsetY;

  if (isnan(accelPitch) || isnan(gyroRate)) return;

  gyroRate = constrain(gyroRate, -500.0f, 500.0f);

  gyroRateFiltered = GYRO_LP_ALPHA * gyroRateFiltered
                    + (1.0f - GYRO_LP_ALPHA) * gyroRate;

  float accelMag = sqrt(ax*ax + ay*ay + az*az);
  float alpha    = (accelMag > ACCEL_MAG_HIGH ||
                    accelMag < ACCEL_MAG_LOW) ? 1.0f : COMPLEMENTARY_ALPHA;

  float rawPitch = alpha * (pitchDeg + angleOffset + gyroRateFiltered * dt)
                   + (1.0f - alpha) * accelPitch;

  pitchDeg = rawPitch - angleOffset;
  pitchDeg = constrain(pitchDeg, -90.0f, 90.0f);
}

// =======================================================
// PID
// =======================================================

float computePID(float dt) {
  //setpointDeg = joystickY * JOYSTICK_ANGLE_TRIM_MAX; // Falls vorhanden

  float error = setpointDeg - pitchDeg;

  // --- ANTI-WIND-UP-SCHUTZ ---
  // Das Integral darf NUR arbeiten, wenn der Roboter fast gerade steht (unter 2 Grad).
  // Wenn er weiter kippt, frieren wir das Integral ein, um das Aufschaukeln zu verhindern!
  if (abs(error) < 2.0f) {
    integral += error * dt;
    integral  = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  } else {
    integral = 0.0f; // Setzt den Speicher zurück, wenn er zu weit kippt
  }

  const float FIXED_DT = 0.005f;
  float derivative  = (error - previousError) / FIXED_DT;
  previousError     = error;

  // Ursprünglicher linearer P-Anteil
  float P = Kp * error;

  // Deine progressive Erweiterung
  if (abs(error) > 6.0f) {
    P = Kp * (error * abs(error) * 0.3f); 
  }

  float I = Ki * integral;
  float D = Kd * derivative;

  float output = P + I + D;
  return constrain(output, -255.0f, 255.0f);
}

// =======================================================
// Setup
// =======================================================

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("ESP32 startet im Balance-Modus...");

  // Motor Pins
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  ledcAttach(ENA, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(ENB, PWM_FREQ, PWM_RESOLUTION);

  stopMotors();

  // I2C
  Wire.begin(23, 18);
  Wire.setClock(400000);
  delay(100);

  // MPU6500 initialisieren
  writeMPU(0x6B, 0x80);   // Reset
  delay(100);
  writeMPU(0x6B, 0x01);   // Wake up
  delay(100);
  writeMPU(0x68, 0x07);   // Signal Path Reset
  delay(100);
  writeMPU(0x1B, 0x08);   // Gyro:  +-500 deg/s
  writeMPU(0x1C, 0x00);   // Accel: +-2g
  writeMPU(0x1A, 0x01);   // DLPF:  184 Hz

  calibrateGyro();

  // Startwinkel bestimmen
  float ax, ay, az, gx, gy, gz;
  readAll(ax, ay, az, gx, gy, gz);
  pitchDeg = atan2(-ax, az) * 180.0f / PI;

  Serial.print("Start-Pitch: ");
  Serial.println(pitchDeg, 2);
  Serial.println("SYSTEM READY - Balancierung aktiv.");

  lastLoopMicros = micros();
}

// =======================================================
// Loop - Stabile 200 Hz ohne Netzwerk-Overhead
// =======================================================

void loop() {
  unsigned long now = micros();
  if (now - lastLoopMicros < LOOP_US) return;

  float dt = (now - lastLoopMicros) / 1000000.0f;
  lastLoopMicros = now;

  // Nach langer Pause (z.B. durch serielles Blockieren): neu initialisieren
  if (dt > 0.05f) {
    float ax, ay, az, gx, gy, gz;
    readAll(ax, ay, az, gx, gy, gz);
    pitchDeg      = atan2(-ax, az) * 180.0f / PI;
    integral      = 0.0f;
    previousError = 0.0f;
    stopMotors();
    return;
  }

  float ax, ay, az, gx, gy, gz;
  readAll(ax, ay, az, gx, gy, gz);

  updatePitch(dt, ax, ay, az, gy);

  int motorCommand = 0; 

  if (abs(pitchDeg) > FALL_ANGLE_LIMIT) {
    stopMotors();
    integral      = 0.0f;
    previousError = 0.0f;
  } else {
    motorCommand = (int)computePID(dt);
    setMotors(motorCommand);
  }

  // Datenausgabe für den IDE Seriellen Plotter (10 Hz)
  if (millis() - lastSerialMillis >= 100) {
    lastSerialMillis = millis();
    
    // Formatierte Ausgabe für das Plotter-Diagramm
    Serial.print("Ist-Pitch:"); Serial.print(pitchDeg, 2);
    Serial.print(" ");
    Serial.print("Soll-Winkel:"); Serial.print(setpointDeg, 2);
    Serial.print(" ");
    Serial.print("PID_skaliert:"); Serial.print((computePID(0.005f) / 10.0f), 1);
    Serial.print(" ");

    int realPwmAverage = (actualPwmLeft + actualPwmRight) / 2;
    Serial.print("PWM_Echt_skaliert:"); Serial.print((realPwmAverage / 10.0f), 1);
    
    Serial.println(); 
  }
}