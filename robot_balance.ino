/*
  Self-balancing robot with ESP32, L298N and MPU6500

  Important:
  The Reefwing MPU6x00 library uses SPI, not I2C.
  Therefore the MPU6500 must be wired with SCLK, SDI/MOSI, SDO/MISO and NCS/CS.

  Recommended corrected MPU6500 wiring:
  VCC       -> 3V3
  GND       -> GND
  SCL/SCLK  -> GPIO18
  SDA/SDI   -> GPIO23
  AD0/SDO   -> GPIO19
  NCS       -> GPIO5

  Recommended corrected L298N wiring:
  ENA -> GPIO32
  IN1 -> GPIO33
  IN2 -> GPIO14
  IN3 -> GPIO25
  ENB -> GPIO26
  IN4 -> GPIO27
*/

#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <ReefwingMPU6x00.h>

// =======================================================
// WiFi access point
// =======================================================

const char* ssid = "ESP32-Control";
const char* password = "12345678";

WebServer server(80);

// =======================================================
// Pin configuration
// =======================================================

// L298N, corrected output-capable pins
const int LEFT_PWM_PIN  = 32;
const int LEFT_IN1_PIN  = 33;
const int LEFT_IN2_PIN  = 14;

const int RIGHT_PWM_PIN = 26;
const int RIGHT_IN1_PIN = 25;
const int RIGHT_IN2_PIN = 27;

// MPU6500 SPI pins
const int MPU_SCK_PIN  = 18;
const int MPU_MISO_PIN = 19;
const int MPU_MOSI_PIN = 23;
const int MPU_CS_PIN   = 5;

// PWM settings
const int PWM_FREQ = 20000;
const int PWM_RESOLUTION = 8;
const int LEFT_PWM_CHANNEL = 0;
const int RIGHT_PWM_CHANNEL = 1;

const int MAX_PWM = 220;
const int MIN_START_PWM = 35;

// =======================================================
// IMU
// =======================================================

static MPU6500 imu = MPU6500(SPI, MPU_CS_PIN);

float ax = 0.0f;
float ay = 0.0f;
float az = 0.0f;
float gx = 0.0f;
float gy = 0.0f;
float gz = 0.0f;

float angleDeg = 0.0f;
float gyroOffsetY = 0.0f;

// Choose the axis that corresponds to forward and backward tilt.
// Depending on how your MPU6500 is mounted, you may need to change this.
const bool INVERT_ANGLE = false;
const bool INVERT_MOTORS = false;

// Complementary filter
const float COMPLEMENTARY_ALPHA = 0.98f;

// =======================================================
// PID
// =======================================================

float Kp = 0.0f;
float Ki = 0.0f;
float Kd = 0.0f;

float setpointDeg = 0.0f;
float integral = 0.0f;
float previousError = 0.0f;

const float INTEGRAL_LIMIT = 300.0f;
const float FALL_ANGLE_LIMIT = 35.0f;

// =======================================================
// Timing
// =======================================================

unsigned long lastControlMicros = 0;
unsigned long lastSerialMillis = 0;

// =======================================================
// Web variables
// =======================================================

float joystickX = 0.0f;
float joystickY = 0.0f;

// Joystick can be used as small trim or manual forward command.
const float JOYSTICK_ANGLE_TRIM_MAX = 4.0f;

// =======================================================
// Web page
// =======================================================

const char MAIN_page[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ESP32 Self-Balancing Robot</title>
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
    .pid-row {
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
      margin-top: 15px;
      background: #505050;
      color: white;
      border: none;
      border-radius: 8px;
    }
    #status, #pid-status {
      font-size: 16px;
      color: #aaa;
      margin-top: 8px;
    }
  </style>
</head>
<body>

  <h1>ESP32 Self-Balancing Robot</h1>

  <h2>Joystick</h2>
  <canvas id="joystick" width="250" height="250"></canvas>
  <p id="status"></p>

  <h2>PID Parameters</h2>
  <div class="pid-row">
    <div class="pid-field">
      <label>Kp</label>
      <input type="number" id="kp" value="0.00" step="0.01">
    </div>
    <div class="pid-field">
      <label>Ki</label>
      <input type="number" id="ki" value="0.00" step="0.01">
    </div>
    <div class="pid-field">
      <label>Kd</label>
      <input type="number" id="kd" value="0.00" step="0.01">
    </div>
  </div>

  <button onclick="sendPID()">Save PID</button>
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
      return {
        x: touch.clientX - rect.left,
        y: touch.clientY - rect.top
      };
    }

    canvas.addEventListener("touchstart", function(e) {
      e.preventDefault();
      active = true;
      updateKnob(getPos(e).x, getPos(e).y);
    });

    canvas.addEventListener("touchmove", function(e) {
      e.preventDefault();
      updateKnob(getPos(e).x, getPos(e).y);
    });

    canvas.addEventListener("touchend", function(e) {
      e.preventDefault();
      resetKnob();
    });

    canvas.addEventListener("mousedown", function(e) {
      active = true;
      updateKnob(getPos(e).x, getPos(e).y);
    });

    canvas.addEventListener("mousemove", function(e) {
      if (!active) return;
      updateKnob(getPos(e).x, getPos(e).y);
    });

    canvas.addEventListener("mouseup", resetKnob);

    canvas.addEventListener("mouseleave", function() {
      if (active) resetKnob();
    });

    function sendJoystick() {
      fetch("/joystick", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          x: joyX,
          y: joyY
        })
      });

      document.getElementById("status").innerText =
        "X: " + joyX.toFixed(2) + "  Y: " + joyY.toFixed(2);
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
          "Saved: Kp=" + kp + "  Ki=" + ki + "  Kd=" + kd;
      })
      .catch(() => {
        document.getElementById("pid-status").innerText = "Error";
      });
    }

    drawJoystick();
  </script>

</body>
</html>
)=====";

// =======================================================
// Motor functions
// =======================================================

void setMotorRaw(int pwmPin, int channel, int in1Pin, int in2Pin, int command) {
  command = constrain(command, -255, 255);

  if (INVERT_MOTORS) {
    command = -command;
  }

  int pwm = abs(command);

  if (pwm > 0 && pwm < MIN_START_PWM) {
    pwm = MIN_START_PWM;
  }

  pwm = constrain(pwm, 0, MAX_PWM);

  if (command > 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
  } else if (command < 0) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
  } else {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    pwm = 0;
  }

  ledcWrite(channel, pwm);
}

void setMotors(int command) {
  setMotorRaw(LEFT_PWM_PIN, LEFT_PWM_CHANNEL, LEFT_IN1_PIN, LEFT_IN2_PIN, command);
  setMotorRaw(RIGHT_PWM_PIN, RIGHT_PWM_CHANNEL, RIGHT_IN1_PIN, RIGHT_IN2_PIN, command);
}

void stopMotors() {
  setMotors(0);
}

// =======================================================
// IMU functions
// =======================================================

void calibrateGyroY() {
  const int samples = 500;
  float sum = 0.0f;

  Serial.println("Gyro calibration. Keep robot still.");

  for (int i = 0; i < samples; i++) {
    while (!imu.dataAvailable()) {
      server.handleClient();
      delay(1);
    }

    imu.readSensor();
    imu.getCalibratedGyro(gx, gy, gz);
    sum += gy;

    delay(2);
  }

  gyroOffsetY = sum / samples;

  Serial.print("Gyro Y offset: ");
  Serial.println(gyroOffsetY, 6);
}

float getAccelAngleDeg() {
  imu.getCalibratedAccel(ax, ay, az);

  float angle = atan2(ax, az) * 180.0f / PI;

  if (INVERT_ANGLE) {
    angle = -angle;
  }

  return angle;
}

void updateAngle(float dt) {
  imu.getCalibratedGyro(gx, gy, gz);
  imu.getCalibratedAccel(ax, ay, az);

  float accelAngle = atan2(ax, az) * 180.0f / PI;

  float gyroRate = gy - gyroOffsetY;

  if (INVERT_ANGLE) {
    accelAngle = -accelAngle;
    gyroRate = -gyroRate;
  }

  angleDeg = COMPLEMENTARY_ALPHA * (angleDeg + gyroRate * dt)
           + (1.0f - COMPLEMENTARY_ALPHA) * accelAngle;
}

// =======================================================
// PID control
// =======================================================

int computePID(float dt) {
  float joystickTrim = joystickY * JOYSTICK_ANGLE_TRIM_MAX;
  setpointDeg = joystickTrim;

  float error = setpointDeg - angleDeg;

  integral += error * dt;
  integral = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  float derivative = (error - previousError) / dt;
  previousError = error;

  float output = Kp * error + Ki * integral + Kd * derivative;

  output = constrain(output, -255.0f, 255.0f);

  return (int)output;
}

// =======================================================
// Web handlers
// =======================================================

void handleRoot() {
  server.send(200, "text/html", MAIN_page);
}

void handleJoystick() {
  if (server.hasArg("plain")) {
    String body = server.arg("plain");

    StaticJsonDocument<96> doc;
    DeserializationError error = deserializeJson(doc, body);

    if (error) {
      server.send(400, "text/plain", "JSON Error");
      return;
    }

    joystickX = doc["x"] | 0.0f;
    joystickY = doc["y"] | 0.0f;

    joystickX = constrain(joystickX, -1.0f, 1.0f);
    joystickY = constrain(joystickY, -1.0f, 1.0f);
  }

  server.send(200, "text/plain", "OK");
}

void handlePID() {
  if (server.hasArg("plain")) {
    String body = server.arg("plain");

    StaticJsonDocument<128> doc;
    DeserializationError error = deserializeJson(doc, body);

    if (error) {
      server.send(400, "text/plain", "JSON Error");
      return;
    }

    Kp = doc["kp"] | Kp;
    Ki = doc["ki"] | Ki;
    Kd = doc["kd"] | Kd;

    integral = 0.0f;
    previousError = 0.0f;

    Serial.println("PID updated:");
    Serial.print("Kp = "); Serial.println(Kp, 4);
    Serial.print("Ki = "); Serial.println(Ki, 4);
    Serial.print("Kd = "); Serial.println(Kd, 4);
  }

  server.send(200, "text/plain", "OK");
}

// =======================================================
// Setup
// =======================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("ESP32 self-balancing robot starting.");

  pinMode(LEFT_IN1_PIN, OUTPUT);
  pinMode(LEFT_IN2_PIN, OUTPUT);
  pinMode(RIGHT_IN1_PIN, OUTPUT);
  pinMode(RIGHT_IN2_PIN, OUTPUT);

  ledcSetup(LEFT_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(LEFT_PWM_PIN, LEFT_PWM_CHANNEL);

  ledcSetup(RIGHT_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(RIGHT_PWM_PIN, RIGHT_PWM_CHANNEL);

  stopMotors();

  WiFi.softAP(ssid, password);

  Serial.print("WiFi AP started. Connect to: ");
  Serial.println(ssid);
  Serial.print("Open browser at: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/joystick", HTTP_POST, handleJoystick);
  server.on("/pid", HTTP_POST, handlePID);
  server.begin();

  Serial.println("Web server started.");

  SPI.begin(MPU_SCK_PIN, MPU_MISO_PIN, MPU_MOSI_PIN, MPU_CS_PIN);

  if (!imu.begin()) {
    Serial.println("MPU6500 initialization failed.");
    Serial.println("Check SPI wiring: SCLK, SDI/MOSI, SDO/MISO and NCS/CS.");
    while (true) {
      stopMotors();
      server.handleClient();
      delay(100);
    }
  }

  Serial.println("MPU6500 connected.");

  imu.calibrateAccelGyro();
  calibrateGyroY();

  while (!imu.dataAvailable()) {
    server.handleClient();
    delay(1);
  }

  imu.readSensor();
  angleDeg = getAccelAngleDeg();

  lastControlMicros = micros();

  Serial.println("Robot ready.");
}

// =======================================================
// Main loop
// =======================================================

void loop() {
  server.handleClient();

  unsigned long now = micros();
  float dt = (now - lastControlMicros) / 1000000.0f;

  if (dt < 0.005f) {
    return;
  }

  lastControlMicros = now;

  if (imu.dataAvailable()) {
    imu.readSensor();
    updateAngle(dt);

    if (abs(angleDeg) > FALL_ANGLE_LIMIT) {
      stopMotors();
      integral = 0.0f;
      previousError = 0.0f;
    } else {
      int motorCommand = computePID(dt);

      // For many two-wheel balancing robots this sign must be tested.
      // If the robot accelerates into the fall instead of catching itself,
      // change this to setMotors(-motorCommand).
      setMotors(motorCommand);
    }
  }

  if (millis() - lastSerialMillis >= 100) {
    lastSerialMillis = millis();

    Serial.print("Angle: ");
    Serial.print(angleDeg, 2);
    Serial.print(" deg | Kp: ");
    Serial.print(Kp, 3);
    Serial.print(" Ki: ");
    Serial.print(Ki, 3);
    Serial.print(" Kd: ");
    Serial.print(Kd, 3);
    Serial.print(" | JoyY: ");
    Serial.println(joystickY, 2);
  }
}