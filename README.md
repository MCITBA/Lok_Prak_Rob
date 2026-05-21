## Anleitung für den Code des selbstbalancierenden Roboters

Der Code steuert einen zweirädrigen Roboter mit ESP32, L298N Motortreiber und MPU6500. Der Roboter misst seinen Neigungswinkel mit dem MPU6500. Wenn er nach vorne oder hinten kippt, berechnet ein PID-Regler eine Motorleistung. Die Räder fahren dann in Kipprichtung, damit der Roboter wieder unter seinen Schwerpunkt fährt und nicht umfällt.

---

## 1. Wichtiger Hinweis zur Schaltung

In deinem Schaltplan ist der MPU6500 wie ein I2C-Sensor angeschlossen, also nur mit `SCL/SCLK` und `SDA/SDI`. Die verwendete Reefwing `MPU6x00` Library arbeitet aber über **SPI**. Dafür müssen zusätzlich `AD0/SDO` und `NCS` verbunden werden. Im Schaltplan sind diese Pins nicht verbunden. 

Für den Code sollte der MPU6500 so angeschlossen werden:

```text
MPU6500 VCC       -> ESP32 3V3
MPU6500 GND       -> ESP32 GND
MPU6500 SCL/SCLK  -> ESP32 GPIO18
MPU6500 SDA/SDI   -> ESP32 GPIO23
MPU6500 AD0/SDO   -> ESP32 GPIO19
MPU6500 NCS       -> ESP32 GPIO5
```

Außerdem sind im Schaltplan `GPIO34` und `GPIO35` mit dem L298N verbunden. Diese Pins sind beim ESP32 **nur Eingänge** und können deshalb keine Motorsteuersignale ausgeben. Für den Code werden stattdessen Ausgangspins verwendet:

```text
ENA -> GPIO32
IN1 -> GPIO33
IN2 -> GPIO14

IN3 -> GPIO25
ENB -> GPIO26
IN4 -> GPIO27
```

---

## 2. Benötigte Bibliotheken

In der Arduino IDE müssen folgende Libraries installiert sein:

```cpp
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <ReefwingMPU6x00.h>
```

Wichtig sind vor allem:

```cpp
#include <ReefwingMPU6x00.h>
```

für den MPU6500 und

```cpp
#include <ArduinoJson.h>
```

für die Übertragung der PID-Werte vom Webserver.

---

## 3. WLAN und Webserver

Der ESP32 erstellt ein eigenes WLAN:

```cpp
const char* ssid = "ESP32-Control";
const char* password = "12345678";
```

Nach dem Hochladen des Codes kannst du dich mit dem Handy oder Laptop mit diesem WLAN verbinden.

Danach öffnest du im Browser die IP-Adresse, die im seriellen Monitor angezeigt wird. Meist ist das:

```text
192.168.4.1
```

Auf dieser Webseite kannst du später die PID-Werte ändern:

```text
Kp
Ki
Kd
```

Diese Werte werden dann über den Webserver an den ESP32 gesendet.

---

## 4. Pinbelegung im Code

Die Motorpins sind hier definiert:

```cpp
const int LEFT_PWM_PIN  = 32;
const int LEFT_IN1_PIN  = 33;
const int LEFT_IN2_PIN  = 14;

const int RIGHT_PWM_PIN = 26;
const int RIGHT_IN1_PIN = 25;
const int RIGHT_IN2_PIN = 27;
```

Die MPU6500 SPI-Pins sind hier definiert:

```cpp
const int MPU_SCK_PIN  = 18;
const int MPU_MISO_PIN = 19;
const int MPU_MOSI_PIN = 23;
const int MPU_CS_PIN   = 5;
```

Falls du andere Pins verwendest, musst du diese Werte im Code anpassen.

---

## 5. Motorsteuerung

Die Motoren werden über PWM angesteuert:

```cpp
const int PWM_FREQ = 20000;
const int PWM_RESOLUTION = 8;
```

Die Auflösung beträgt 8 Bit. Das bedeutet, der PWM-Wert kann zwischen `0` und `255` liegen.

Die maximale Motorleistung wird begrenzt durch:

```cpp
const int MAX_PWM = 220;
```

Damit fahren die Motoren nicht direkt mit voller Leistung.

Zusätzlich gibt es:

```cpp
const int MIN_START_PWM = 35;
```

Viele kleine Gleichstrommotoren drehen bei sehr kleinen PWM-Werten noch nicht an. Wenn der Regler also eine kleine Bewegung fordert, wird mindestens `35` ausgegeben, damit der Motor überhaupt losläuft.

---

## 6. MPU6500 und Winkelmessung

Der MPU6500 liefert Beschleunigungsdaten und Gyroskopdaten.

Der Roboter braucht vor allem den Neigungswinkel nach vorne oder hinten. Dieser Winkel wird im Code in dieser Variable gespeichert:

```cpp
float angleDeg = 0.0f;
```

Der Winkel wird mit einem Komplementärfilter berechnet:

```cpp
angleDeg = COMPLEMENTARY_ALPHA * (angleDeg + gyroRate * dt)
         + (1.0f - COMPLEMENTARY_ALPHA) * accelAngle;
```

Dabei gilt:

```cpp
const float COMPLEMENTARY_ALPHA = 0.98f;
```

Das bedeutet:

```text
98 % Gyroskop
2 % Beschleunigungssensor
```

Das Gyroskop reagiert schnell, driftet aber mit der Zeit. Der Beschleunigungssensor driftet nicht, ist aber stärker verrauscht. Der Komplementärfilter kombiniert beide Vorteile.

---

## 7. Gyro-Kalibrierung

Beim Start wird das Gyroskop kalibriert:

```cpp
calibrateGyroY();
```

Währenddessen muss der Roboter ruhig stehen. Der Code misst mehrere Werte und berechnet daraus einen Offset:

```cpp
gyroOffsetY = sum / samples;
```

Dieser Offset wird später von der Gyro-Messung abgezogen.

Wichtig: Beim Einschalten darf der Roboter nicht bewegt werden.

---

## 8. PID-Regler

Der PID-Regler berechnet aus dem aktuellen Neigungswinkel eine Motorleistung.

Die drei Parameter sind:

```cpp
float Kp = 0.0f;
float Ki = 0.0f;
float Kd = 0.0f;
```

Der Sollwinkel ist normalerweise:

```cpp
float setpointDeg = 0.0f;
```

Das bedeutet, der Roboter soll senkrecht stehen.

Der Fehler wird so berechnet:

```cpp
float error = setpointDeg - angleDeg;
```

Beispiel:

```text
Sollwinkel: 0°
Istwinkel:  5°
Fehler:    -5°
```

Der PID-Ausgang wird dann berechnet mit:

```cpp
float output = Kp * error + Ki * integral + Kd * derivative;
```

Die Bedeutung:

```text
Kp: reagiert direkt auf den aktuellen Winkel
Ki: korrigiert langsame dauerhafte Abweichungen
Kd: dämpft schnelle Bewegungen und Schwingungen
```

Für den Anfang solltest du verwenden:

```text
Kp = 8
Ki = 0
Kd = 0.3
```

Danach wird zuerst `Kp` erhöht, bis der Roboter deutlich reagiert. Danach wird `Kd` erhöht, um Schwingungen zu reduzieren. `Ki` sollte erst ganz am Ende sehr vorsichtig hinzugefügt werden.

---

## 9. Sicherheitsabschaltung

Diese Zeile legt fest, ab welchem Winkel der Roboter als umgefallen gilt:

```cpp
const float FALL_ANGLE_LIMIT = 35.0f;
```

Wenn der Roboter stärker als `35°` kippt, werden die Motoren abgeschaltet:

```cpp
if (abs(angleDeg) > FALL_ANGLE_LIMIT) {
  stopMotors();
  integral = 0.0f;
  previousError = 0.0f;
}
```

Das verhindert, dass die Räder mit voller Leistung weiterdrehen, obwohl der Roboter bereits liegt.

---

## 10. Hauptprogramm

Im `setup()` passiert Folgendes:

```text
1. Serielle Kommunikation starten
2. Motorpins konfigurieren
3. PWM für die Motoren einrichten
4. WLAN Access Point starten
5. Webserver starten
6. SPI für den MPU6500 starten
7. MPU6500 initialisieren
8. Gyroskop kalibrieren
9. Startwinkel bestimmen
```

Im `loop()` passiert dann dauerhaft:

```text
1. Webserver abfragen
2. Zeitdifferenz dt berechnen
3. MPU6500 auslesen
4. Winkel berechnen
5. Prüfen, ob der Roboter zu stark gekippt ist
6. PID-Regler berechnen
7. Motoren ansteuern
8. Werte im seriellen Monitor ausgeben
```

---

## 11. Was tun, wenn der Roboter falsch herum fährt?

Wenn der Roboter nach vorne kippt, müssen die Räder nach vorne fahren. Wenn er stattdessen nach hinten fährt, ist die Richtung falsch.

Dann musst du diese Zeile ändern:

```cpp
setMotors(motorCommand);
```

zu:

```cpp
setMotors(-motorCommand);
```

Alternativ kannst du auch die Motorkabel tauschen oder die Variable ändern:

```cpp
const bool INVERT_MOTORS = false;
```

zu:

```cpp
const bool INVERT_MOTORS = true;
```

---

## 12. Was tun, wenn der Winkel falsch herum ist?

Wenn der Roboter nach vorne kippt, der Winkel im seriellen Monitor aber negativ wird, musst du die Winkelrichtung invertieren:

```cpp
const bool INVERT_ANGLE = false;
```

ändern zu:

```cpp
const bool INVERT_ANGLE = true;
```

Das ist abhängig davon, wie der MPU6500 auf dem Roboter montiert ist.

---

## 13. Empfohlene Testreihenfolge

Gehe beim Testen so vor:

1. Roboter ohne Motorleistung testen
   Prüfen, ob der MPU6500 erkannt wird.

2. Winkel im seriellen Monitor prüfen
   Der Winkel sollte sich ändern, wenn du den Roboter nach vorne und hinten kippst.

3. Motorrichtung prüfen
   Bei positiver Motorleistung müssen beide Räder in dieselbe Richtung drehen.

4. Kleine PID-Werte einstellen
   Zum Beispiel:

```text
Kp = 8
Ki = 0
Kd = 0.3
```

5. Roboter festhalten und testen
   Noch nicht frei fahren lassen.

6. Richtung korrigieren
   Wenn der Roboter in die falsche Richtung beschleunigt, `setMotors(-motorCommand)` verwenden.

7. PID-Werte langsam erhöhen
   Erst `Kp`, dann `Kd`, zuletzt sehr vorsichtig `Ki`.

---

## 14. Typische Fehler

### MPU6500 wird nicht erkannt

Mögliche Ursachen:

```text
SPI falsch angeschlossen
NCS nicht verbunden
AD0/SDO nicht verbunden
3.3 V fehlt
GND fehlt
falsche Library verwendet
```

### Ein Motor dreht nicht

Mögliche Ursachen:

```text
L298N falsch angeschlossen
GPIO34 oder GPIO35 verwendet
Motorversorgung fehlt
GND von ESP32 und L298N nicht verbunden
```

### Roboter fährt sofort weg

Mögliche Ursachen:

```text
Motoren laufen in falscher Richtung
Winkel ist invertiert
Kp ist zu groß
MPU6500 ist falsch montiert
```

### Roboter zittert stark

Mögliche Ursachen:

```text
Kd zu klein
Kp zu groß
Motoren haben Spiel
Roboter ist mechanisch zu weich
Sensor ist nicht fest montiert
```

### Roboter reagiert zu schwach

Mögliche Ursachen:

```text
Kp zu klein
MAX_PWM zu klein
MIN_START_PWM zu klein
Akkuspannung zu niedrig
Motoren zu schwach
```

---

## 15. Kurz gesagt

Der Code macht im Kern Folgendes:

```text
MPU6500 misst Winkel
PID-Regler berechnet notwendige Motorleistung
ESP32 steuert L298N an
L298N treibt beide Motoren an
Webserver erlaubt Einstellung von Kp, Ki und Kd
```

Der wichtigste Punkt ist: Der Roboter muss immer in die Richtung fahren, in die er kippt. Nur dann kann er sich selbst wieder aufrichten.
