/*
  Self-Balancing Robot - ESP32
  Motorsteuerung austesten

  L298N Wiring
  ENA  -> GPIO26
  IN1  -> GPIO25
  IN2  -> GPIO27
  IN3  -> GPIO14
  IN4  -> GPIO33
  ENB  -> GPIO32
*/


// =======================================================
// Motor
// =======================================================

// Motort Pins
const int ENA = 26;
const int IN1 = 25;
const int IN2 = 27;

const int IN3 = 14;
const int IN4 = 33;
const int ENB = 32;

// Testwert
const int test = 70;

// Steuerungsparameter - hohe Frequenz
/*
const int PWM_FREQ       = 1000;    
const int PWM_RESOLUTION = 8;       // 0-255
const int MIN_START_PWM  = 100;
const int MAX_PWM        = 255;
*/

// Steuerungsparameter - hohes Drehmoment
const int PWM_FREQ       = 255;    // Niedrige Frequenz für besseres Drehmoment am L298N
const int PWM_RESOLUTION = 8;       // 0-255
const int MIN_START_PWM  = 70;
const int MAX_PWM        = 255;


// Motor-Trimm-Faktor
const float TRIM_LEFT  = 1.00f;  // 100% Leistung
const float TRIM_RIGHT = 1.00f;  // 100% Leistung



// Motor Funktionen
void setMotor(int ena, int in1, int in2, int command) {   
  command = constrain(command, -255, 255);  // command muss zwischen -255 und 255 sein
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



void setup() {
  // put your setup code here, to run once:
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

}

void loop() {
  // put your main code here, to run repeatedly:
  setMotors(test);
  Serial.println(test);
}
