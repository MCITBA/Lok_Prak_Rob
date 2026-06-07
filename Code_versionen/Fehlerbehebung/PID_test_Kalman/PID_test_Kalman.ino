#include <Wire.h>
#include <Kalman.h>

// ==========================================
// 1. HARDWARE PINS DEFINIEREN
// ==========================================
// IMU Pins
const int MPU_ADDR = 0x68;
const int SDA_PIN  = 23;
const int SCL_PIN  = 18;

// L298N Pins
const int ENA = 26;
const int IN1 = 25;
const int IN2 = 27;

const int IN3 = 14;
const int IN4 = 33;
const int ENB = 32;

// Motor-Trimm-Faktoren für den perfekten Geradeauslauf
const float TRIM_LEFT  = 1.00f;  
const float TRIM_RIGHT = 1.00f;  

// ==========================================
// 2. STEUERUNGSPARAMETER & FREQUENZPROFILE
// ==========================================

// --- PROFIL 1: Niedrige Frequenz (Hohes Drehmoment) ---
const int PWM_FREQ       = 255;
const int PWM_RESOLUTION = 8;       // 0-255
const int MIN_START_PWM  = 80;      // minimaler PWM-Wert zum Überwinden der Trägheit / Haftreibung des Getriebes
const int MIN_HOLD_PWM   = 40;
const int MAX_PWM        = 255;

/*
// --- PROFIL 2: Hohe Frequenz (Kein Fiepen, evtl. mehr Start-PWM nötig) ---
const int PWM_FREQ       = 1000;    
const int PWM_RESOLUTION = 8;       // 0-255
int MIN_START_PWM        = 100;     // Bei 1000Hz muss der Wert evtl. erhöht werden
const int MIN_HOLD_PWM   = 40;      // Bei 1000Hz muss der Wert evtl. erhöht werden
const int MAX_PWM        = 255;
*/

// ==========================================
// 3. KICKSTART & RICHTUNGSWECHSEL LOGIK
// ==========================================
// ermöglicht es auch geringere PWM-Werte zu verwenden (Haftreibung > Gleitreibung)
const int KICKSTART_CYCLES = 10;    // 10 Zyklen (ca. 20ms bei delay(2))

int kickCounterLeft = 0;
int kickCounterRight = 0;

int lastSignLeft = 0; 
int lastSignRight = 0;

// ==========================================
// 4. SCHWERPUNKT-OFFSET & PID-TUNING
// ==========================================
// Der Winkel, bei dem der Roboter mechanisch im Schwerpunkt steht
const float ANGLE_OFFSET = 2.5f; 

// Startwerte für das Tuning auf dem Boden (mit delay(2))
float Kp = 10.0;  
float Ki = 0.0;   
float Kd = 0.0;

float errorSum = 0;
float lastError = 0;

int actualPwmLeft = 0;
int actualPwmRight = 0;

// ==========================================
// VARIABLEN FÜR KALMAN & IMU
// ==========================================
Kalman kalmanPitch;

int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;

double kalmanPitchDeg; 
double accPitchDeg;
unsigned long lastMicros;

// ==========================================
// MOTOR-FUNKTIONEN MIT RICHTUNGSWECHSEL-KICKSTART
// ==========================================
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
  // Wenn sich das Vorzeichen zur vorherigen Schleife geändert hat (und wir nicht aus dem Stopp kamen)
  if (currentSign != lastSign && lastSign != 0) {
    kickCounter = 0; // Erzwinge einen brandneuen Kickstart in die entgegengesetzte Richtung!
  }
  lastSign = currentSign; // Vorzeichen für den nächsten Durchlauf merken
  // -----------------------------------------------------------

  // Deine Kickstart-Regelung
  if (pwm < MIN_START_PWM) {
    if (kickCounter < KICKSTART_CYCLES) {
      // Stufe 1: Der Kickstart-Tritt (80 PWM)
      pwm = MIN_START_PWM; 
      kickCounter++; 
    } else {
      // Stufe 2: Kickstart vorbei! Aber wir halten den Motor bei mindestens 40 PWM auf Zug
      if (pwm < MIN_HOLD_PWM) {
        pwm = MIN_HOLD_PWM;
      }
    }
  } else {
    // Stufe 3: Der PID-Wert ist ohnehin größer als 80. Normalfahrt
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

  // Beide Motoren separat mit ihren eigenen Zählern und Vorzeichen-Speichern füttern
  setMotor(ENA, IN1, IN2, commandLeft, kickCounterLeft, lastSignLeft, actualPwmLeft);
  setMotor(ENB, IN3, IN4, commandRight, kickCounterRight, lastSignRight, actualPwmRight);
}

void stopMotors() {
  setMotors(0);
}



// ==========================================
// HILFSFUNKTION: IMU DATA READ
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
  delay(500);
  
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

  // Motor-Pins deklarieren
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // PWM starten
  ledcAttach(ENA, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(ENB, PWM_FREQ, PWM_RESOLUTION);

  // Startwinkel für Kalman initialisieren (mit deiner Achsengeometrie)
  readMPUData();
  accPitchDeg = (atan2(accX, accZ) * RAD_TO_DEG);
  kalmanPitch.setAngle(accPitchDeg);
  
  stopMotors();
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

  // 1. Echten Pitch-Winkel berechnen (Deine Geometrie-Formel)
  accPitchDeg = (atan2(accX, accZ) * RAD_TO_DEG);

  // 2. Gyro-Drehgeschwindigkeit für Pitch (Y-Achse)
  double gyroRatePitch = (double)gyroY / 131.0;

  // 3. Kalman-Filter berechnen & mechanischen Schwerpunkt abziehen
  kalmanPitchDeg = kalmanPitch.getAngle(accPitchDeg, gyroRatePitch, dt) - ANGLE_OFFSET;

  // HIER GEÄNDERT: Variable vorab deklarieren, damit sie in der ganzen loop() bekannt ist!
  float pidOutput = 0.0;

  // 4. NOT-AUS: Wenn der Roboter flach liegt (> 45 Grad), Motoren komplett aus!
  if (abs(kalmanPitchDeg) > 45.0) {
    stopMotors();
    errorSum = 0; // Integralspeicher leeren
    pidOutput = 0; // Im Not-Aus ist der PWM-Wert logischerweise 0
  } 
  else {
    // 5. DER PID-ALGORITHMUS
    float error = kalmanPitchDeg - 0.0; // Soll-Winkel (0.0) minus Ist-Winkel
    
    // P-Anteil
    float P = Kp * error;
    
    // I-Anteil
    errorSum += error * dt;
    float I = Ki * errorSum;
    
    // D-Anteil
    float D = Kd * ((error - lastError) / dt);
    lastError = error;

    // Gesamtes Steuersignal berechnen
    pidOutput = P + I + D;

    // 6. Motoren ansteuern
    setMotors((int)pidOutput);
  }

  // 7. Datenausgabe für den Seriellen Plotter (alle 10ms)
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 10) { 
    lastPrint = millis();
    
    // Ausgabe des aktuellen Winkels
    Serial.print("Winkel_Ist:");
    Serial.print(kalmanPitchDeg);
    Serial.print(","); 
    
    // Jetzt klappt es: Ausgabe des aktuellen PWM-Werts
    Serial.print("PWM_Wert:");
    Serial.println(abs((int)pidOutput)); 

    // Der echte, an die Motoren ausgegebene PWM-Mittelwert
    int realPwmAverage = (actualPwmLeft + actualPwmRight) / 2;
    Serial.print("PWM_Echt:");
    Serial.println(realPwmAverage);
  }

  delay(2); // Deine schnelle 2ms Loop!
}