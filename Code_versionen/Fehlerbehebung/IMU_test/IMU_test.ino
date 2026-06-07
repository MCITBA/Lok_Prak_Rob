#include <Wire.h>
#include <Kalman.h>

// ==========================================
// 1. HARDWARE PINS DEFINIEREN
// ==========================================
// IMU Pins (Deine funktionierenden I2C-Pins)
const int MPU_ADDR = 0x68;
const int SDA_PIN  = 23;
const int SCL_PIN  = 18;

// ==========================================
// 2. SCHWERPUNKT-OFFSET (Schritt 3)
// ==========================================
// Trage hier später den Winkel ein, bei dem dein Roboter perfekt balanciert!
const float ANGLE_OFFSET = 2.5f; 

// ==========================================
// VARIANBLEN FÜR KALMAN & IMU
// ==========================================
Kalman kalmanPitch;

int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;

double kalmanPitchDeg; 
double accPitchDeg;
unsigned long lastMicros;

// ==========================================
// HILFSFUNKTION: IMU REWRITE
// ==========================================
void readMPUData() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true); 

  accX  = Wire.read() << 8 | Wire.read();  
  accY  = Wire.read() << 8 | Wire.read();  
  accZ  = Wire.read() << 8 | Wire.read();  
  Wire.read() << 8 | Wire.read(); // Temp überspringen
  gyroX = Wire.read() << 8 | Wire.read();  
  gyroY = Wire.read() << 8 | Wire.read();  
  gyroZ = Wire.read() << 8 | Wire.read();  
}


// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  
  // I2C mit deinen Pins starten
  Wire.begin(SDA_PIN, SCL_PIN); 

  // MPU6500 aufwecken
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); 
  Wire.write(0);    
  Wire.endTransmission(true);
  delay(50);

  // Gyroskop-Messbereich auf +-250 Grad/Sekunde setzen
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); 
  Wire.write(0x00); 
  Wire.endTransmission(true);
  delay(50);

  // Ersten Winkel messen für den Kalman-Startwert
  readMPUData();
  accPitchDeg = atan2(accX, accZ) * RAD_TO_DEG;
  kalmanPitch.setAngle(accPitchDeg);
  
  lastMicros = micros();
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
  // Zeitdifferenz (dt) berechnen
  unsigned long now = micros();
  double dt = (double)(now - lastMicros) / 1000000.0;
  lastMicros = now;

  // IMU-Daten auslesen
  readMPUData();

  // 1. Echten Pitch-Winkel berechnen
  accPitchDeg = atan2(accX, accZ) * RAD_TO_DEG;

  // 2. Gyro-Drehgeschwindigkeit für Pitch (Y-Achse)
  double gyroRatePitch = (double)gyroY / 131.0;

  // 3. Kalman-Filter berechnen & Schwerpunkt abziehen
  kalmanPitchDeg = kalmanPitch.getAngle(accPitchDeg, gyroRatePitch, dt) - ANGLE_OFFSET;

  // 4. Datenausgabe für den Seriellen Plotter (20 Hz)
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 50) { 
    lastPrint = millis();
    Serial.print("Winkel_Ist:");
    Serial.println(kalmanPitchDeg);
  }

  delay(5); // Schleife läuft mit ca. 200 Hz
}

