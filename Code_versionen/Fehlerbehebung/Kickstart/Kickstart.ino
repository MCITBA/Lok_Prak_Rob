/*
  Self-Balancing Robot - ESP32
  Motorsteuerung austesten mit zeitgesteuertem Kickstart

  L298N Wiring
  ENA  -> GPIO26
  IN1  -> GPIO25
  IN2  -> GPIO27
  IN3  -> GPIO14
  IN4  -> GPIO33
  ENB  -> GPIO32
*/

// =======================================================
// Motor Pins
// =======================================================
const int ENA = 26;
const int IN1 = 25;
const int IN2 = 27;

const int IN3 = 14;
const int IN4 = 33;
const int ENB = 32;

// =======================================================
// Steuerungsparameter
// =======================================================
// Steuerungsparameter - hohes Drehmoment
const int PWM_FREQ       = 255;     // Niedrige Frequenz für besseres Drehmoment am L298N
const int PWM_RESOLUTION = 8;       // 0-255
const int MIN_START_PWM  = 80;      // Die Grenze, ab der sich der Motor sicher bewegt
const int MAX_PWM        = 255;

// Motor-Trimm-Faktoren
const float TRIM_LEFT  = 1.00f;  
const float TRIM_RIGHT = 1.00f;  

// =======================================================
// KICKSTART-PARAMETER & VARIABLEN
// =======================================================
// Wie viele Schleifendurchläufe der Kickstart-Impuls gehalten werden soll.
// Bei einer sehr schnellen Loop (z.B. delay(2)) entsprechen 10 Zyklen etwa 30ms.
const int KICKSTART_CYCLES = 10; 

// Die Zähler für beide Motoren, um die Impulsdauer im Auge zu behalten
int kickCounterLeft = 0;
int kickCounterRight = 0;

// =======================================================
// MOTOR FUNKTIONEN
// =======================================================
// Der Funktion übergeben wir den Zähler als Referenz (&), damit sie ihn verändern kann
void setMotor(int ena, int in1, int in2, int command, int &kickCounter) {   
  command = constrain(command, -255, 255);  
  int pwm = abs(command);

  // Wenn der Motor laut Regler/Code komplett stehen soll
  if (pwm == 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(ena, 0);
    kickCounter = 0; // Bereit machen für den nächsten Start aus dem Stand
    return;
  }

  // --- HIER STECKT DEINE KICKSTART-LOGIK ---
  // Wenn der Ziel-PWM-Wert kleiner als die Losbrechschwelle (70) ist:
  if (pwm < MIN_START_PWM) {
    if (kickCounter < KICKSTART_CYCLES) {
      // Phase A: Er kriegt für X Schleifen den vollen Tritt (MIN_START_PWM = 70)
      pwm = MIN_START_PWM; 
      kickCounter++; 
    } else {
      // Phase B: Die Zeit ist abgelaufen, wir fallen auf deinen echten, kleineren Wert ab
      pwm = pwm; 
    }
  } else {
    // Wenn der Wert ohnehin >= 70 ist, dreht sich der Motor sowieso. Kein Kickstart nötig.
    kickCounter = 0; 
  }
  // ----------------------------------------

  pwm = constrain(pwm, 0, MAX_PWM);

  // Richtung bestimmen
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
  int commandLeft  = (int)(command * TRIM_LEFT);
  int commandRight = (int)(command * TRIM_RIGHT);

  // Wir reichen die Zähler-Variablen für beide Seiten an die Einzelsteuerung weiter
  setMotor(ENA, IN1, IN2, commandLeft, kickCounterLeft);
  setMotor(ENB, IN3, IN4, commandRight, kickCounterRight);
}

void stopMotors() {
  setMotors(0);
}

// =======================================================
// SETUP
// =======================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32 startet...");

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  ledcAttach(ENA, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(ENB, PWM_FREQ, PWM_RESOLUTION);

  stopMotors();
}

// =======================================================
// MAIN LOOP (Intervall-Test)
// =======================================================
void loop() {
  // Test 1: Wir fordern einen Wert UNTERHALB von MIN_START_PWM (z.B. 40)
  // Der Motor sollte jetzt kurz mit 70 anrucken und danach auf 40 abfallen.
  Serial.println("Fahre langsam an (Ziel: PWM 40)...");
  unsigned long startTime = millis();
  while (millis() - startTime < 3000) {
    setMotors(40); // Hier testen wir deinen Wunschwert von 40
    delay(2);      // Entspricht deinem schnellen Takt beim Balancieren
  }

  // Test 2: Vollständiger Stopp für 3 Sekunden, damit die Zähler sich resetten
  Serial.println("Stopp / Stillstand...");
  startTime = millis();
  while (millis() - startTime < 3000) {
    stopMotors();
    delay(2);
  }
}