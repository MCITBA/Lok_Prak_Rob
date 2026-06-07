/*
  Self-Balancing Robot - ESP32
  MPU6500 (I2C) + PID + L298N + Live Webserver Tuning + PID Plotting

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
#include <WiFi.h>
#include <WebServer.h>

// =======================================================
// Webserver Konfiguration
// =======================================================
const char* ssid = "BalancingRobot_Tuning";
const char* password = "password123"; // Mindestens 8 Zeichen
WebServer server(80);

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
int PWM_FREQ             = 255;    
const int PWM_RESOLUTION = 8;       // 0-255
const int MAX_PWM        = 255;
int MIN_START_PWM        = 75;     
const int MIN_HOLD_PWM   = 40;     

// Motor-Trimm-Faktor
const float TRIM_LEFT  = 1.00f;  
const float TRIM_RIGHT = 1.00f;  

// =======================================================
// KICKSTART & RICHTUNGSWECHSEL LOGIK
// =======================================================
const int KICKSTART_CYCLES = 7;    

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
float angleOffset      = 0.0f; 

// =======================================================
// PID Tunings & Globale Plot-Variablen
// =======================================================

float Kp = 15.0f;
float Ki = 0.2f;  
float Kd = 0.3f;

float setpointDeg   = 0.0f; 
float integral      = 0.0f;
float previousError = 0.0f;

const float INTEGRAL_LIMIT   = 300.0f;
const float FALL_ANGLE_LIMIT = 30.0f;

// Globale Variablen, damit der serielle Plotter die Einzelanteile mitschreiben kann
float last_P_out = 0.0f;
float last_I_out = 0.0f;
float last_D_out = 0.0f;

// =======================================================
// Timing
// =======================================================

unsigned long lastLoopMicros   = 0;
unsigned long lastSerialMillis = 0;

// =======================================================
// HTML Dashboard Template
// =======================================================

void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<title>Robot PID Tuning</title>";
  html += "<style>body{font-family:Arial,sans-serif;background:#f4f4f9;padding:20px;color:#333;}";
  html += ".card{background:white;padding:20px;border-radius:10px;box-shadow:0 4px 8px rgba(0,0,0,0.1);max-width:400px;margin:0 auto;}";
  html += "h2{text-align:center;color:#007BFF;} .form-group{margin-bottom:15px;}";
  html += "label{display:block;margin-bottom:5px;font-weight:bold;}";
  html += "input[type='number']{width:100%;padding:8px;box-sizing:border-box;border:1px solid #ccc;border-radius:5px;}";
  html += "button{width:100%;padding:10px;background:#007BFF;border:none;color:white;font-size:16px;border-radius:5px;cursor:pointer;margin-top:10px;}";
  html += "button:hover{background:#0056b3;}";
  html += ".angle-box{text-align:center;font-size:24px;font-weight:bold;margin:15px 0;padding:10px;background:#e9ecef;border-radius:5px;color:#495057;}</style>";
  
  // JavaScript für das automatische Live-Update des Winkels im Webserver (AJAX)
  html += "<script>setInterval(function(){";
  html += "  var xhttp = new XMLHttpRequest();";
  html += "  xhttp.onreadystatechange = function() {";
  html += "    if (this.readyState == 4 && this.status == 200) {";
  html += "      document.getElementById('live_angle').innerHTML = this.responseText + ' °';";
  // Dynamische Farbe: Rot bei Sturz/großem Fehler, Grün bei Balance
  html += "      var angle = Math.abs(parseFloat(this.responseText));";
  html += "      if(angle > 5.0){ document.getElementById('live_angle').style.color = '#dc3545'; }";
  html += "      else { document.getElementById('live_angle').style.color = '#28a745'; }";
  html += "    }";
  html += "  };";
  html += "  xhttp.open('GET', '/getAngle', true);";
  html += "  xhttp.send();";
  html += "}, 50);</script>"; // Aktualisiert alle 200ms im Browser
  
  html += "</head><body>";
  
  html += "<div class='card'><h2>PID & Motor Live Tuning</h2>";
  
  // Live Winkel Anzeige Box
  html += "<label style='text-align:center;'>Aktueller Winkel:</label>";
  html += "<div class='angle-box' id='live_angle'>0.00 °</div>";
  
  html += "<form action='/save' method='POST'>";
  html += "<div class='form-group'><label>Kp:</label><input type='number' step='0.1' name='kp' value='" + String(Kp) + "'></div>";
  html += "<div class='form-group'><label>Ki:</label><input type='number' step='0.01' name='ki' value='" + String(Ki) + "'></div>";
  html += "<div class='form-group'><label>Kd:</label><input type='number' step='0.01' name='kd' value='" + String(Kd) + "'></div>";
  html += "<div class='form-group'><label>Angle Offset (°):</label><input type='number' step='0.1' name='offset' value='" + String(angleOffset) + "'></div>";
  html += "<div class='form-group'><label>MIN START PWM:</label><input type='number' step='1' name='min_start' value='" + String(MIN_START_PWM) + "'></div>";
  html += "<div class='form-group'><label>PWM Frequenz (Hz):</label><input type='number' step='1' name='freq' value='" + String(PWM_FREQ) + "'></div>";
  
  html += "<button type='submit'>Speichern & Übernehmen</button>";
  html += "</form></div></body></html>";
  
  server.send(200, "text/html", html);
}

void handleSave() {
  if (server.hasArg("kp")) Kp = server.arg("kp").toFloat();
  if (server.hasArg("ki")) Ki = server.arg("ki").toFloat();
  if (server.hasArg("kd")) Kd = server.arg("kd").toFloat();
  if (server.hasArg("offset")) angleOffset = server.arg("offset").toFloat();
  if (server.hasArg("min_start")) MIN_START_PWM = server.arg("min_start").toInt();
  
  if (server.hasArg("freq")) {
    int newFreq = server.arg("freq").toInt();
    if (newFreq != PWM_FREQ) {
      PWM_FREQ = newFreq;
      ledcAttach(ENA, PWM_FREQ, PWM_RESOLUTION);
      ledcAttach(ENB, PWM_FREQ, PWM_RESOLUTION);
    }
  }
  
  server.sendHeader("Location", "/");
  server.send(333, "text/plain", "Bereit");
}

// Endpoint für das JavaScript, um den Winkel abzufragen
void handleGetAngle() {
  server.send(200, "text/plain", String(pitchDeg, 2));
}

// =======================================================
// Motor Funktionen
// =======================================================

void setMotor(int ena, int in1, int in2, int command, int &kickCounter, int &lastSign, int &actualPwmStore) {
  command = constrain(command, -255, 255);
  int pwm = abs(command);

  int currentSign = (command > 0) ? 1 : ((command < 0) ? -1 : 0);

  if (pwm == 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(ena, 0);
    kickCounter = 0; 
    lastSign = 0;
    actualPwmStore = 0;
    return;
  }

  if (currentSign != lastSign && lastSign != 0) {
    kickCounter = 0; 
  }
  lastSign = currentSign; 

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
// PID (Mit Einzelwert-Speicherung für den Plotter)
// =======================================================

float computePID(float dt) {
  float error = setpointDeg - pitchDeg;

  // --- ANTI-WIND-UP-SCHUTZ ---
  if (abs(error) < 3.0f) {
    integral += error * dt;
    integral  = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  } else {
    integral = 0.0f; 
  }

  const float FIXED_DT = 0.005f;
  float derivative  = (error - previousError) / FIXED_DT;
  previousError     = error;

  // P-Anteil mit progressiver Kennlinie berechnen
  float P = Kp * error;
  if (abs(error) > 4.0f) {
    P = Kp * (error * abs(error) * 0.4f); 
  }

  float I = Ki * integral;
  float D = Kd * derivative;

  // Werte für die serielle Ausgabe sichern
  last_P_out = P;
  last_I_out = I;
  last_D_out = D;

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

  // Access Point aufmachen
  WiFi.softAP(ssid, password);
  Serial.println("WLAN Access Point gestartet!");
  Serial.print("IP-Adresse: ");
  Serial.println(WiFi.softAPIP());

  // Webserver URLs verknüpfen (start() durch korrekte Funktion begin() ersetzt)
  server.on("/", handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/getAngle", HTTP_GET, handleGetAngle); // Neuer Live-Winkel-Endpoint
  server.begin();
  Serial.println("Webserver gestartet.");

  Serial.println("SYSTEM READY - Balancierung aktiv.");
  lastLoopMicros = micros();
}

// =======================================================
// Loop
// =======================================================

// =======================================================
// Loop
// =======================================================

void loop() {
  server.handleClient();

  unsigned long now = micros();
  if (now - lastLoopMicros < LOOP_US) return;

  float dt = (now - lastLoopMicros) / 1000000.0f;
  lastLoopMicros = now;

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

  int motorCommand = 0; // Hier wird sie das erste und einzige Mal deklariert!

  if (abs(pitchDeg) > FALL_ANGLE_LIMIT) {
    stopMotors();
    integral      = 0.0f;
    previousError = 0.0f;
    // Auch die Plot-Werte im Sturz nullen
    last_P_out = 0.0f; last_I_out = 0.0f; last_D_out = 0.0f;
  } else {
    motorCommand = (int)computePID(dt);
    setMotors(motorCommand);
  }

  // Datenausgabe für den IDE Seriellen Plotter / MATLAB mit vollen 200 Hz
  if (millis() - lastSerialMillis >= 5) {
    lastSerialMillis = millis();
    
    // 1. Winkeldaten
    Serial.print("Ist-Pitch:"); Serial.print(pitchDeg, 2);
    Serial.print(" ");
    
    // 2. Die einzelnen PID-Kanal-Anteile unskaliert (direkter PWM-Einfluss von -255 bis 255)
    Serial.print("P-Anteil:"); Serial.print(last_P_out, 1);
    Serial.print(" ");
    Serial.print("I-Anteil:"); Serial.print(last_I_out, 1);
    Serial.print(" ");
    Serial.print("D-Anteil:"); Serial.print(last_D_out, 1);
    Serial.print(" ");

    // 3. Reales Gesamtsignal am Motor
    int realPwmAverage = (actualPwmLeft + actualPwmRight) / 2;
    // Wir nutzen hier direkt das oben berechnete motorCommand für das Vorzeichen (OHNE neues 'int'!)
    int signedRealPwm = (motorCommand >= 0) ? realPwmAverage : -realPwmAverage;
    Serial.print("PWM_Effektiv:"); Serial.print(signedRealPwm);
    
    Serial.println(); 
  }
}