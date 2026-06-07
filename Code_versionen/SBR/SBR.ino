/*
  Self-Balancing Robot - ESP32
  MPU6500 (I2C) + Webserver + PID + L298N

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

  -------------------------------------------------------
  Webserver
  -------------------------------------------------------
  1. Mit dem Netzwerk "ESP32-Control" verbinden (Passwort: 12345678)
  2. Browser: http://192.168.4.1
*/

#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <Wire.h>

// =======================================================
// WiFi
// =======================================================

const char* ssid     = "ESP32-Control";
const char* password = "12345678";

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

const int PWM_FREQ       = 1000;    // Niedrige Frequenz für besseres Drehmoment am L298N
const int PWM_RESOLUTION = 8;       // 0-255
const int MAX_PWM        = 255;
const int MIN_START_PWM  = 100;      // Mindestwert damit Motoren anlaufen

// Motor-Trimm-Faktor
const float TRIM_LEFT  = 1.00f;  // 100% Leistung
const float TRIM_RIGHT = 1.00f;  // 100% Leistung

// =======================================================
// MPU6500
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
float angleOffset      = -2.8f; 

// =======================================================
// PID
// =======================================================

float Kp = 20.0f;
float Ki = 50.0f;
float Kd = 0.5f;

float setpointDeg   = 0.0f;
float integral      = 0.0f;
float previousError = 0.0f;

const float INTEGRAL_LIMIT   = 300.0f;
const float FALL_ANGLE_LIMIT = 35.0f;

// =======================================================
// Joystick
// =======================================================

float joystickX = 0.0f;
float joystickY = 0.0f;

const float JOYSTICK_ANGLE_TRIM_MAX = 4.0f;

// =======================================================
// Timing
// =======================================================

unsigned long lastLoopMicros   = 0;
unsigned long lastSerialMillis = 0;

// =======================================================
// Website (Aktualisiert mit Offset-Buttons)
// =======================================================

const char MAIN_page[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Robot Controller</title>
  <style>
    body {
      background: #202020;
      color: white;
      text-align: center;
      font-family: Arial;
      user-select: none;
    }
    canvas {
      display: block;
      margin: 20px auto;
      touch-action: none;
    }
    input {
      font-size: 20px;
      width: 120px;
      padding: 8px;
      margin: 5px;
    }
    label {
      font-size: 20px;
    }
    .pid-row, .offset-row {
      display: flex;
      justify-content: center;
      gap: 20px;
      flex-wrap: wrap;
      margin: 10px 0;
    }
    .pid-field {
      display: flex;
      flex-direction: column;
      align-items: center;
    }
    button {
      font-size: 20px;
      padding: 10px 30px;
      margin: 10px;
      background: #505050;
      color: white;
      border: none;
      border-radius: 8px;
      cursor: pointer;
    }
    button:active {
      background: #707070;
    }
    .btn-offset {
      font-size: 24px;
      padding: 15px 25px;
      background: #2c5282;
    }
    .btn-offset:active {
      background: #4299e1;
    }
    #status     { font-size: 16px; color: #aaa; margin-top: 8px; }
    #pid-status { font-size: 16px; color: #aaa; margin-top: 8px; }
    #offset-val { font-size: 24px; font-weight: bold; color: #63b3ed; margin: 10px 0; }
  </style>
</head>
<body>

  <h1>ESP32 Robot Controller</h1>

  <h2>Joystick</h2>
  <canvas id="joystick" width="250" height="250"></canvas>
  <p id="status"></p>

  <h2>Balance Nullpunkt-Trimmung</h2>
  <div id="offset-val">0.0°</div>
  <div class="offset-row">
    <button class="btn-offset" onclick="adjustOffset(-0.1)">- 0.1°</button>
    <button class="btn-offset" onclick="adjustOffset(0.1)">+ 0.1°</button>
  </div>

  <h2>PID-Parameter</h2>
  <div class="pid-row">
    <div class="pid-field">
      <label>Kp</label>
      <input type="number" id="kp" value="20.0" step="0.01"> </div>
    <div class="pid-field">
      <label>Ki</label>
      <input type="number" id="ki" value="5.00" step="0.01">
    </div>
    <div class="pid-field">
      <label>Kd</label>
      <input type="number" id="kd" value="0.5" step="0.01"> </div>
  </div>

  <button onclick="sendPID()">Speichern</button>
  <p id="pid-status"></p>

  <script>
    const canvas = document.getElementById("joystick");
    const ctx = canvas.getContext("2d");

    const cx = canvas.width / 2;
    const cy = canvas.height / 2;
    const radius = 100;
    const knobRadius = 30;

    let knobX = cx;
    let knobY = cy;
    let active = false;
    let joyX = 0;
    let joyY = 0;
    let currentOffset = 0.0;

    function drawJoystick() {
      ctx.clearRect(0, 0, canvas.width, canvas.height);

      ctx.beginPath();
      ctx.arc(cx, cy, radius, 0, Math.PI * 2);
      ctx.fillStyle = "#404040";
      ctx.fill();
      ctx.strokeStyle = "#606060";
      ctx.lineWidth = 2;
      ctx.stroke();

      ctx.strokeStyle = "#606060";
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(cx - radius, cy);
      ctx.lineTo(cx + radius, cy);
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(cx, cy - radius);
      ctx.lineTo(cx, cy + radius);
      ctx.stroke();

      ctx.beginPath();
      ctx.arc(knobX, knobY, knobRadius, 0, Math.PI * 2);
      ctx.fillStyle = active ? "#909090" : "#707070";
      ctx.fill();
    }

    function updateKnob(x, y) {
      const dx = x - cx;
      const dy = y - cy;
      const dist = Math.sqrt(dx * dx + dy * dy);
      const maxDist = radius - knobRadius;

      if (dist < maxDist) {
        knobX = x;
        knobY = y;
      } else {
        knobX = cx + (dx / dist) * maxDist;
        knobY = cy + (dy / dist) * maxDist;
      }

      joyX =  (knobX - cx) / maxDist;
      joyY = -(knobY - cy) / maxDist;

      drawJoystick();
      sendJoystick();
    }

    function resetKnob() {
      knobX = cx;
      knobY = cy;
      joyX = 0;
      joyY = 0;
      active = false;
      drawJoystick();
      sendJoystick();
    }

    function getPos(e) {
      const rect = canvas.getBoundingClientRect();
      const touch = e.touches ? e.touches[0] : e;
      return { x: touch.clientX - rect.left, y: touch.clientY - rect.top };
    }

    canvas.addEventListener("touchstart",  function(e) { e.preventDefault(); active = true; updateKnob(getPos(e).x, getPos(e).y); });
    canvas.addEventListener("touchmove",   function(e) { e.preventDefault(); updateKnob(getPos(e).x, getPos(e).y); });
    canvas.addEventListener("touchend",    function(e) { e.preventDefault(); resetKnob(); });
    canvas.addEventListener("mousedown",   function(e) { active = true; updateKnob(getPos(e).x, getPos(e).y); });
    canvas.addEventListener("mousemove",   function(e) { if (!active) return; updateKnob(getPos(e).x, getPos(e).y); });
    canvas.addEventListener("mouseup",     resetKnob);
    canvas.addEventListener("mouseleave",  function() { if (active) resetKnob(); });

    function sendJoystick() {
      fetch("/joystick", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ x: joyX.toFixed(3), y: joyY.toFixed(3) })
      });
      document.getElementById("status").innerText =
        "X: " + joyX.toFixed(2) + "  Y: " + joyY.toFixed(2);
    }

    function adjustOffset(val) {
      currentOffset += val;
      // Rundung auf 1 Nachkommastelle, um JS-Fließkommafehler zu vermeiden
      currentOffset = Math.round(currentOffset * 10) / 10;
      document.getElementById("offset-val").innerText = (currentOffset > 0 ? "+" : "") + currentOffset.toFixed(1) + "°";
      
      fetch("/offset", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ offset: currentOffset })
      });
    }

    function sendPID() {
      let kp = parseFloat(document.getElementById("kp").value);
      let ki = parseFloat(document.getElementById("ki").value);
      let kd = parseFloat(document.getElementById("kd").value);

      fetch("/pid", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ kp: kp, ki: ki, kd: kd })
      })
      .then(() => {
        document.getElementById("pid-status").innerText =
          "✓ Gespeichert  Kp:" + kp + "  Ki:" + ki + "  Kd:" + kd;
      })
      .catch(() => {
        document.getElementById("pid-status").innerText = "✗ Fehler";
      });
    }

    drawJoystick();
  </script>
</body>
</html>
)=====";

// =======================================================
// Motor Funktionen
// =======================================================

void setMotor(int ena, int in1, int in2, int command) {
  command = constrain(command, -255, 255);
  int pwm = abs(command);

  if (pwm > 0 && pwm < MIN_START_PWM) {
    pwm = MIN_START_PWM;
  }

  pwm = constrain(pwm, 0, MAX_PWM);

  if (command > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (command < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    pwm = 0;
  }

  ledcWrite(ena, pwm);
}

void setMotors(int command) {
  // Berechne getrennte Commands für beide Motoren
  int commandLeft  = (int)(command * TRIM_LEFT);
  int commandRight = (int)(command * TRIM_RIGHT);

  // Steuer die Motoren getrennt an
  setMotor(ENA, IN1, IN2, commandLeft);
  setMotor(ENB, IN3, IN4, commandRight);
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

  // Mathematisch berechneter Roh-Pitch
  float rawPitch = alpha * (pitchDeg + angleOffset + gyroRateFiltered * dt)
                   + (1.0f - alpha) * accelPitch;

  // Ziehe hier den Trimm-Offset ab, um den physikalischen Nullpunkt zu verschieben
  pitchDeg = rawPitch - angleOffset;

  pitchDeg = constrain(pitchDeg, -90.0f, 90.0f);
}

// =======================================================
// PID
// =======================================================

float computePID(float dt) {
  setpointDeg = joystickY * JOYSTICK_ANGLE_TRIM_MAX;

  float error = setpointDeg - pitchDeg;

  integral += error * dt;
  integral  = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  // Nutzen des stabilen, festen DT-Kehrwerts (0.005s für 200Hz) für saubere D-Anteile
  const float FIXED_DT = 0.005f;
  float derivative  = (error - previousError) / FIXED_DT;
  previousError     = error;

  float output = Kp * error
               + Ki * integral
               + Kd * derivative;

  return constrain(output, -255.0f, 255.0f);
}

// =======================================================
// Webserver Handler
// =======================================================

void handleRoot() {
  server.send(200, "text/html", MAIN_page);
}

void handleJoystick() {
  if (server.hasArg("plain")) {
    String body = server.arg("plain");
    StaticJsonDocument<64> doc;
    if (deserializeJson(doc, body)) {
      server.send(400, "text/plain", "JSON Error");
      return;
    }
    joystickX = constrain((float)doc["x"], -1.0f, 1.0f);
    joystickY = constrain((float)doc["y"], -1.0f, 1.0f);
  }
  server.send(200, "text/plain", "OK");
}

// NEU: Empfängt den Justierungswert von der Website
void handleOffset() {
  if (server.hasArg("plain")) {
    String body = server.arg("plain");
    StaticJsonDocument<64> doc;
    if (deserializeJson(doc, body)) {
      server.send(400, "text/plain", "JSON Error");
      return;
    }
    angleOffset = doc["offset"];
    Serial.print("Neuer Balance-Offset eingestellt: ");
    Serial.print(angleOffset, 1);
    Serial.println("°");
  }
  server.send(200, "text/plain", "OK");
}

void handlePID() {
  if (server.hasArg("plain")) {
    String body = server.arg("plain");
    StaticJsonDocument<96> doc;
    if (deserializeJson(doc, body)) {
      server.send(400, "text/plain", "JSON Error");
      return;
    }
    Kp = doc["kp"];
    Ki = doc["ki"];
    Kd = doc["kd"];

    integral      = 0.0f;
    previousError = 0.0f;

    Serial.println("--- PID gespeichert ---");
    Serial.print("Kp: "); Serial.println(Kp, 3);
    Serial.print("Ki: "); Serial.println(Ki, 3);
    Serial.print("Kd: "); Serial.println(Kd, 3);
    Serial.println("-----------------------");
  }
  server.send(200, "text/plain", "OK");
}

// =======================================================
// Setup
// =======================================================

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32 startet...");

  // Motor Pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  ledcAttach(ENA, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(ENB, PWM_FREQ, PWM_RESOLUTION);

  stopMotors();

  // WiFi
  WiFi.softAP(ssid, password);
  Serial.print("IP Adresse: ");
  Serial.println(WiFi.softAPIP());

  server.on("/",         handleRoot);
  server.on("/joystick", HTTP_POST, handleJoystick);
  server.on("/pid",      HTTP_POST, handlePID);
  server.on("/offset",   HTTP_POST, handleOffset); // NEU: Endpunkt registrieren
  server.begin();
  Serial.println("Webserver gestartet");

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

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 1);
  Serial.print("WHO_AM_I: 0x");
  Serial.println(Wire.read(), HEX);

  calibrateGyro();

  // Startwinkel
  float ax, ay, az, gx, gy, gz;
  readAll(ax, ay, az, gx, gy, gz);
  pitchDeg = atan2(-ax, az) * 180.0f / PI;

  Serial.print("Start-Pitch: ");
  Serial.println(pitchDeg, 2);
  Serial.println("READY");

  lastLoopMicros = micros();
}

// =======================================================
// Loop - 200 Hz
// =======================================================

void loop() {
  server.handleClient();

  unsigned long now = micros();
  if (now - lastLoopMicros < LOOP_US) return;

  float dt = (now - lastLoopMicros) / 1000000.0f;
  lastLoopMicros = now;

  // Nach langer Pause: neu initialisieren
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

  if (abs(pitchDeg) > FALL_ANGLE_LIMIT) {
    // Roboter umgefallen
    stopMotors();
    integral      = 0.0f;
    previousError = 0.0f;
  } else {
    int motorCommand = (int)computePID(dt);
    setMotors(motorCommand);
  }

  // Ausgabe mit 10 Hz
  if (millis() - lastSerialMillis >= 100) {
    lastSerialMillis = millis();
    Serial.print("pitch=");  Serial.print(pitchDeg, 2);
    Serial.print("  soll="); Serial.print(setpointDeg, 2);
    Serial.print("  err=");  Serial.print(setpointDeg - pitchDeg, 2);
    // Festes dt für die serielle PID-Kontrollanzeige nutzen, um Fehlanzeigen im Log zu vermeiden
    Serial.print("  pid=");  Serial.print((int)computePID(0.005f)); 
    Serial.print("  Kp=");   Serial.print(Kp, 1);
    Serial.print("  Ki=");   Serial.print(Ki, 2);
    Serial.print("  Kd=");   Serial.println(Kd, 2);
    
    // Format für den Arduino IDE Seriellen Plotter
    Serial.print("Ist-Pitch:"); Serial.print(pitchDeg, 2);
    Serial.print(" ");
    Serial.print("Soll-Winkel:"); Serial.print(setpointDeg, 2);
    Serial.print(" ");
    
    // Da PID bis 255 geht, teilen wir es durch 10 (skalieren), 
    // damit es im selben Diagramm wie die Gradzahlen (-35 bis +35) gut lesbar ist.
    Serial.print("PID_skaliert:"); Serial.print((computePID(0.005f) / 10.0f), 1);
    
    Serial.println(); // Wichtig: Zeilenumbruch beendet den aktuellen Datenpunkt
  }
}