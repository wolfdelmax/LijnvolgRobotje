#include <QTRSensors.h>

// ==========================================
// --- FINETUNING VARIABELEN ---
// ==========================================
float Kp = 0.1;
float Kd = 0.007;
int lastError = 0;

// --- SNELHEID INSTELLINGEN (in procenten) ---
int snelheidMapping    = 40;
int minBochSnelheid    = 25;
int kalibratieSnelheid = 30;

// ==========================================

// --- COMPONENTEN ---
QTRSensors qtr;

// --- SENSOR PINS (QTR-8) ---
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
const uint8_t sensorPinnen[] = {23, 15, 32, 27, 26, 14, 12, 13};

// --- MOTOR PINS (TB6612) ---
const int pinAIN1 = 16;
const int pinAIN2 = 4;
const int pinPWMA = 18;
const int pinBIN1 = 17;
const int pinBIN2 = 5;
const int pinPWMB = 19;
const int pwmFreq = 5000;
const int pwmResolution = 8;

// --- SCHAKELAAR ---
const int pinModeSchakelaar = 1;

// --- BEREKENDE SNELHEDEN ---
int baseSpeed;
int minBochSpeed;
int kalibSpeed;

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(pinModeSchakelaar, INPUT_PULLUP);

  baseSpeed    = (snelheidMapping * 255) / 100;
  minBochSpeed = (minBochSnelheid * 255) / 100;
  kalibSpeed   = (kalibratieSnelheid * 255) / 100;

  pinMode(pinAIN1, OUTPUT);
  pinMode(pinAIN2, OUTPUT);
  pinMode(pinBIN1, OUTPUT);
  pinMode(pinBIN2, OUTPUT);
  ledcAttach(pinPWMA, pwmFreq, pwmResolution);
  ledcAttach(pinPWMB, pwmFreq, pwmResolution);

  qtr.setTypeRC();
  qtr.setSensorPins(sensorPinnen, SensorCount);

  kalibreerRobot();

  Serial.println("\n=== LIJNVOLGEN GESTART ===");
}

void loop() {
  if (digitalRead(pinModeSchakelaar) == HIGH) {
    remmen();
    return;
  }

  uint16_t positie = qtr.readLineBlack(sensorValues);

  int error = (int)positie - 3500;
  float dynamischeSnelheid = baseSpeed - (abs(error) * 0.025);
  dynamischeSnelheid = constrain(dynamischeSnelheid, minBochSpeed, baseSpeed);

  int correctie = (error * Kp) + ((error - lastError) * Kd);
  lastError = error;

  rijden((int)dynamischeSnelheid, correctie);
}

// === MOTOR CONTROL ===

void rijden(int snelheid, int correctie) {
  setMotorLinks(constrain(snelheid + correctie, -255, 255));
  setMotorRechts(constrain(snelheid - correctie, -255, 255));
}

void setMotorLinks(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(pinAIN1, HIGH);
    digitalWrite(pinAIN2, LOW);
    ledcWrite(pinPWMA, speed);
  }
  else if (speed < 0) {
    digitalWrite(pinAIN1, LOW);
    digitalWrite(pinAIN2, HIGH);
    ledcWrite(pinPWMA, -speed);
  }
  else {
    digitalWrite(pinAIN1, LOW);
    digitalWrite(pinAIN2, LOW);
    ledcWrite(pinPWMA, 0);
  }
}

void setMotorRechts(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(pinBIN1, HIGH);
    digitalWrite(pinBIN2, LOW);
    ledcWrite(pinPWMB, speed);
  }
  else if (speed < 0) {
    digitalWrite(pinBIN1, LOW);
    digitalWrite(pinBIN2, HIGH);
    ledcWrite(pinPWMB, -speed);
  }
  else {
    digitalWrite(pinBIN1, LOW);
    digitalWrite(pinBIN2, LOW);
    ledcWrite(pinPWMB, 0);
  }
}

void remmen() {
  setMotorLinks(0);
  setMotorRechts(0);
}

// === KALIBRATIE ===

void kalibreerRobot() {
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);

  Serial.println("KALIBRATIE GESTART (automatisch draaien)...");

  for (uint16_t i = 0; i < 400; i++) {
    if (i < 100) {
      setMotorLinks(-kalibSpeed);
      setMotorRechts(kalibSpeed);
    } else if (i < 200) {
      setMotorLinks(kalibSpeed);
      setMotorRechts(-kalibSpeed);
    } else if (i < 300) {
      setMotorLinks(-kalibSpeed);
      setMotorRechts(kalibSpeed);
    } else {
      setMotorLinks(kalibSpeed);
      setMotorRechts(-kalibSpeed);
    }

    qtr.calibrate();

    if (i % 50 == 0) {
      Serial.print(".");
    }
  }

  remmen();
  digitalWrite(2, LOW);
  Serial.println("\nKALIBRATIE KLAAR!");
  delay(1000);
}