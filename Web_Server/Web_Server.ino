// Anleitung zum Webserver
// 1. Mit dem Netzwerk: "ESP32-Controll" vebinden und mit dem Passwort "12345678" anmelden
// 2. im Broswer folgende Adresse aufrufen: http://192.168.4.1

// Bibliotheken laden
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>

// Globale Variablen definieren
const char* ssid = "ESP32-Control";
const char* password = "12345678";
WebServer server(80);   // erzeugt eine Instanz des Webservers -> erreichbar unter: http://192.168.4.1
float joystickX = 0;
float joystickY = 0;
float Kp = 0;
float Ki = 0;
float Kd = 0;

// definiert die Website
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
    #status {
      font-size: 16px;
      color: #aaa;
      margin-top: 8px;
    }
    #pid-status {
      font-size: 16px;
      color: #aaa;
      margin-top: 8px;
    }
  </style>
</head>
<body>

  <h1>ESP32 Robot Controller</h1>

  <h2>Joystick</h2>
  <canvas id="joystick" width="250" height="250"></canvas>
  <p id="status"></p>

  <h2>PID-Parameter</h2>
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

    // Touch Events
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

    // Maus Events
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

    // Nur Joystick senden
    function sendJoystick() {
      fetch("/joystick", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          x: joyX.toFixed(3),
          y: joyY.toFixed(3)
        })
      });

      document.getElementById("status").innerText =
        "X: " + joyX.toFixed(2) + "  Y: " + joyY.toFixed(2);
    }

    // Nur PID senden
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

// Wird aufgerufen wenn jemand 192.168.4.1/ aufruft. Sendet die HTML-Seite zurück
// wird nur einmal ausgeführt -> danach läuft alles über handleJoystick() und handlePID()
// die HTML-Seite liegt ab dann komplett im Browser
void handleRoot() {
   // allgemeiner Aufbau von server.send
  server.send(200,            // HTTP-Statuscode: 200 = OK, 404 = Not Found, 400 = Bad Request
              "text/html",    // Content-Type: was für Daten werden gesendet
              MAIN_page);     // der eigentliche Inhalt
}

// Joystick
// Wird automatisch aufgerufen wenn POST /joystick ankommt
void handleJoystick() {
  if (server.hasArg("plain")) {   // Prüft ob überhaupt ein Body mitgeschickt wurde
    String body = server.arg("plain");  // Liest den rohen HTTP-Body als Text

    StaticJsonDocument<64> doc;  // Erstellt einen JSON-Container im Arbeitsspeicher
    DeserializationError error = deserializeJson(doc, body);  // Wandelt den JSON-Text in ein C++ Objekt um das du abfragen kannst. Speichert einen möglichen Fehler in error

    if (error) {  // Falls ein Fehler auftritt word die Hunktion beendet
      server.send(400, "text/plain", "JSON Error");
      return;
    }

    // Liest die Werte aus dem JSON-Objekt und schreibt sie in die globalen Variablen
    joystickX = doc["x"];
    joystickY = doc["y"];

    // Gibt die Werte im Serial Monitor aus
    Serial.print("X: "); Serial.print(joystickX, 3);
    Serial.print("  Y: "); Serial.println(joystickY, 3);
  }
  // Das HTTP-Protokoll funktioniert nach dem Request-Response-Prinzip – auf jede Anfrage muss eine Antwort folgen
  // ohne Antwort wartet der Browser endlos und blockiert die nächste Anfrage -->  die Website friert ein
  server.send(200, "text/plain", "OK");
}

// PID
// Wird automatisch aufgerufen wenn POST /pid ankommt
void handlePID() {
  if (server.hasArg("plain")) {   // Prüft ob überhaupt ein Body mitgeschickt wurde
    String body = server.arg("plain");  // Liest den rohen HTTP-Body als Text

    StaticJsonDocument<96> doc;  // Erstellt einen JSON-Container im Arbeitsspeicher
    DeserializationError error = deserializeJson(doc, body);  // Wandelt den JSON-Text in ein C++ Objekt um das du abfragen kannst. Speichert einen möglichen Fehler in error

    if (error) {  // Falls ein Fehler auftritt wird die Funktion beendet
      server.send(400, "text/plain", "JSON Error");
      return;
    }

    // Liest die Werte aus dem JSON-Objekt und schreibt sie in die globalen Variablen
    Kp = doc["kp"];
    Ki = doc["ki"];
    Kd = doc["kd"];

    // Gibt die Werte im Serial Monitor aus
    Serial.println("--- PID gespeichert ---");
    Serial.print("Kp: "); Serial.println(Kp, 3);
    Serial.print("Ki: "); Serial.println(Ki, 3);
    Serial.print("Kd: "); Serial.println(Kd, 3);
    Serial.println("-----------------------");
  }
  // Das HTTP-Protokoll funktioniert nach dem Request-Response-Prinzip – auf jede Anfrage muss eine Antwort folgen
  // ohne Antwort wartet der Browser endlos und blockiert die nächste Anfrage -->  die Website friert ein
  server.send(200, "text/plain", "OK");
}


void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("ESP32 startet...");

  // WLAN starten
  WiFi.softAP(ssid, password);
  Serial.print("IP Adresse: ");
  Serial.println(WiFi.softAPIP());

  // Verknüpft die URL "/" mit der Funktion handleRoot()
  // Wenn das iPad "192.168.4.1/" aufruft → handleRoot() wird aufgerufen
  server.on("/", handleRoot);

  // Verknüpft die URL "/joystick" mit der Funktion handleJoystick()
  // Aber nur wenn es ein POST ist – ein versehentlicher GET auf "/joystick" wird ignoriert
  // Wenn das iPad "192.168.4.1/joystick" per POST aufruft → handleJoystick() wird aufgerufen
  server.on("/joystick", HTTP_POST, handleJoystick);

  // Verknüpft die URL "/pid" mit der Funktion handlePID()
  // Aber nur wenn es ein POST ist – ein versehentlicher GET auf "/pid" wird ignoriert
  // Wenn das iPad "192.168.4.1/pid" per POST aufruft → handlePID() wird aufgerufen
  server.on("/pid", HTTP_POST, handlePID);

  // Startet den Webserver – ab jetzt werden eingehende Anfragen entgegengenommen
  server.begin();

  Serial.println("Webserver gestartet");
}

void loop() {
  server.handleClient();  // prüft ob eine Anfrage da ist – wenn nicht, macht er nichts
}