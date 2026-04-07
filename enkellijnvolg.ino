#include <QTRSensors.h>

// ==========================================
// --- FINETUNING VARIABELEN ---
// ==========================================
float Kp = 0.8;
float Kd = 1.5;           // Dempt het slingeren
int snelheid = 35;
const int drempelRotatie = 15; // Hoeveel ticks verschil voordat we de 'sweep' filteren
const int kalibratieSnelheid = 80; // PWM snelheid tijdens kalibratie

// ==========================================
// --- PIN DEFINITIES ---
const int pinSchakelaar = 1;
const uint8_t SensorCount = 8;
const uint8_t sensorPinnen[] = {23, 15, 32, 27, 26, 14, 12, 13};

// Encoders (Pas de pinnen aan naar jouw ESP32 setup)
const int pinEncL = 34; 
const int pinEncR = 35;

// Motoren
const int pinAIN1 = 16; const int pinAIN2 = 4;  const int pinPWMA = 18;
const int pinBIN1 = 17; const int pinBIN2 = 5;  const int pinPWMB = 19;
const int pwmFreq = 5000;
const int pwmResolution = 8;

// ==========================================
// --- GLOBALE VARIABELEN ---
QTRSensors qtr;
uint16_t sensorValues[SensorCount];
int baseSpeed;
int lastError = 0;

volatile long countsL = 0;
volatile long countsR = 0;
long lastCountsL = 0;
long lastCountsR = 0;

// Interrupt functions voor encoders
void IRAM_ATTR readEncL() { countsL++; }
void IRAM_ATTR readEncR() { countsR++; }

// Handmatige positieberekening op basis van sensorwaarden
uint16_t berekenPositie(uint16_t *vals, uint8_t count) {
  uint32_t gewogen = 0;
  uint32_t totaal = 0;
  for (uint8_t i = 0; i < count; i++) {
    gewogen += (uint32_t)vals[i] * i * 1000;
    totaal += vals[i];
  }
  if (totaal == 0) return 3500;
  return gewogen / totaal;
}

void setup() {
  Serial.begin(115200);
  pinMode(pinSchakelaar, INPUT_PULLUP);
  baseSpeed = (snelheid * 255) / 100;
  
  // Motor Setup
  pinMode(pinAIN1, OUTPUT); pinMode(pinAIN2, OUTPUT);
  pinMode(pinBIN1, OUTPUT); pinMode(pinBIN2, OUTPUT);
  ledcAttach(pinPWMA, pwmFreq, pwmResolution);
  ledcAttach(pinPWMB, pwmFreq, pwmResolution);
  
  // Encoder Setup
  pinMode(pinEncL, INPUT_PULLUP);
  pinMode(pinEncR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinEncL), readEncL, RISING);
  attachInterrupt(digitalPinToInterrupt(pinEncR), readEncR, RISING);

  // Sensor Setup
  qtr.setTypeRC();
  qtr.setSensorPins(sensorPinnen, SensorCount);
  
  Serial.println("Kalibratie begint...");
  kalibreer();
  stopMotoren();
  Serial.println("Klaar! Zet schakelaar LAAG om te rijden.");
}

void loop() {
  if (digitalRead(pinSchakelaar) == HIGH) {
    stopMotoren();
    return;
  }

  // 1. Bereken de fysieke rotatie via encoders
  long diffL = countsL - lastCountsL;
  long diffR = countsR - lastCountsR;
  long rotationDelta = diffL - diffR;
  
  lastCountsL = countsL;
  lastCountsR = countsR;

  // 2. Lees de sensoren
  uint16_t position = qtr.readLineBlack(sensorValues);

  // 3. GEOMETRISCHE VALIDATIE (De Sweep-Filter)
  if (rotationDelta > drempelRotatie) {
    bool lineInMiddle = (sensorValues[3] > 600 || sensorValues[4] > 600);
    if (!lineInMiddle) {
      sensorValues[0] = 0; sensorValues[1] = 0; sensorValues[2] = 0;
      position = berekenPositie(sensorValues, SensorCount);
    }
  } 
  else if (rotationDelta < -drempelRotatie) {
    bool lineInMiddle = (sensorValues[3] > 600 || sensorValues[4] > 600);
    if (!lineInMiddle) {
      sensorValues[5] = 0; sensorValues[6] = 0; sensorValues[7] = 0;
      position = berekenPositie(sensorValues, SensorCount);
    }
  }

  // 4. PD Berekening
  int error = position - 3500;
  int motorSpeed = (Kp * error) + (Kd * (error - lastError));
  lastError = error;

  int leftSpeed = baseSpeed + motorSpeed;
  int rightSpeed = baseSpeed - motorSpeed;

  stuurMotoren(leftSpeed, rightSpeed);
}

// --- AANSTURING ---

void stuurMotoren(int links, int rechts) {
  links = constrain(links, -255, 255);
  rechts = constrain(rechts, -255, 255);

  digitalWrite(pinAIN1, links >= 0 ? HIGH : LOW);
  digitalWrite(pinAIN2, links >= 0 ? LOW : HIGH);
  ledcWrite(pinPWMA, abs(links));

  digitalWrite(pinBIN1, rechts >= 0 ? HIGH : LOW);
  digitalWrite(pinBIN2, rechts >= 0 ? LOW : HIGH);
  ledcWrite(pinPWMB, abs(rechts));
}

void stopMotoren() {
  stuurMotoren(0, 0);
}

void kalibreer() {
  // Draai heen en weer over de lijn zodat alle sensoren zwart én wit zien
  for (uint16_t i = 0; i < 400; i++) {
    if (i < 100) {
      // Draai naar links
      stuurMotoren(-kalibratieSnelheid, kalibratieSnelheid);
    } else if (i < 300) {
      // Draai naar rechts
      stuurMotoren(kalibratieSnelheid, -kalibratieSnelheid);
    } else {
      // Draai terug naar midden
      stuurMotoren(-kalibratieSnelheid, kalibratieSnelheid);
    }
    qtr.calibrate();
  }
  stopMotoren();
}