#include <QTRSensors.h>

// ==========================================
// --- FINETUNING VARIABELEN ---
// ==========================================
float Kp = 0.12;
float Kd = 0.007;
int lastError = 0;

// --- SNELHEID INSTELLINGEN (in procenten) ---
int snelheidMapping    = 40;
int minBochSnelheid    = 25;
int kalibratieSnelheid = 30;

// --- ENCODER AFSTANDEN ---
int doorrijTicks  = 125;
int eindvlakTicks = 280;

// --- DRAAIEN ---
const float ticksPerGraad = 2.135f;
float maxDraaiGraden = 140.0f;  // max rotatie tijdens DRAAIEN
const long maxDraaiTicks = (long)(maxDraaiGraden * ticksPerGraad);

// --- LED ---
const int pinLed = 2;
const unsigned long ledKnipperMs = 100;

// ==========================================

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
const uint8_t sensorPinnen[] = {23, 15, 32, 27, 26, 14, 12, 13};

const int pinAIN1 = 16, pinAIN2 = 4,  pinPWMA = 18;
const int pinBIN1 = 17, pinBIN2 = 5,  pinPWMB = 19;
const int pwmFreq = 5000, pwmResolution = 8;

const int pinEncoderL = 34;
const int pinEncoderR = 35;
volatile long encoderTellerL = 0;
volatile long encoderTellerR = 0;

void IRAM_ATTR encoderISR_L() { encoderTellerL++; }
void IRAM_ATTR encoderISR_R() { encoderTellerR++; }

const int pinModeSchakelaar = 1;

int baseSpeed, minBochSpeed, kalibSpeed;

enum RobotModus { MAPPING, KLAAR };
RobotModus huidigeRobotModus = MAPPING;

enum RobotStatus { VOLGEN, NAAR_KRUISPUNT, DRAAIEN, UTURN, DOORRIJDEN, DOORRIJDEN_UTURN, STOP };
RobotStatus huidigeStatus = VOLGEN;

unsigned long startTime     = 0;
unsigned long mappingTijdMs = 0;

bool snapLinks  = false;
bool snapRechts = false;

unsigned long actieStartTijd = 0;
bool lijnVerlaten = false;
int  draaiSpdL = 0, draaiSpdR = 0;
bool draaiTeruggedraaid = false;

// LED knipperstatus
unsigned long ledLaatsteWissel = 0;
bool ledStatus = false;

// --- FORWARD DECLARATIES ---
void kalibreerRobot();
void runMapping();
void finishMapping();
void updateLed();

void setup() {
  pinMode(pinModeSchakelaar, INPUT_PULLUP);
  pinMode(pinLed, OUTPUT);
  digitalWrite(pinLed, LOW);

  baseSpeed    = (snelheidMapping    * 255) / 100;
  minBochSpeed = (minBochSnelheid    * 255) / 100;
  kalibSpeed   = (kalibratieSnelheid * 255) / 100;

  pinMode(pinAIN1, OUTPUT); pinMode(pinAIN2, OUTPUT);
  pinMode(pinBIN1, OUTPUT); pinMode(pinBIN2, OUTPUT);
  ledcAttach(pinPWMA, pwmFreq, pwmResolution);
  ledcAttach(pinPWMB, pwmFreq, pwmResolution);

  pinMode(pinEncoderL, INPUT_PULLUP);
  pinMode(pinEncoderR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinEncoderL), encoderISR_L, RISING);
  attachInterrupt(digitalPinToInterrupt(pinEncoderR), encoderISR_R, RISING);

  qtr.setTypeRC();
  qtr.setSensorPins(sensorPinnen, SensorCount);

  huidigeRobotModus = MAPPING;
  kalibreerRobot();

  startTime = millis();
}

void loop() {
  bool schakelaarHoog = (digitalRead(pinModeSchakelaar) == HIGH);

  if (huidigeRobotModus == KLAAR) {
    remmen();
    digitalWrite(pinLed, LOW);
    return;
  }

  if (schakelaarHoog) {
    remmen();
    digitalWrite(pinLed, LOW);
    return;
  }

  if (huidigeRobotModus == MAPPING) {
    runMapping();
    updateLed();
  }
}

// ==========================================
// LED STURING
// ==========================================
void updateLed() {
  if (huidigeStatus == VOLGEN) {
    digitalWrite(pinLed, HIGH);
    ledStatus = true;
  }
  else if (huidigeStatus == NAAR_KRUISPUNT
        || huidigeStatus == DRAAIEN
        || huidigeStatus == UTURN
        || huidigeStatus == DOORRIJDEN
        || huidigeStatus == DOORRIJDEN_UTURN) {
    unsigned long nu = millis();
    if (nu - ledLaatsteWissel >= ledKnipperMs) {
      ledStatus = !ledStatus;
      digitalWrite(pinLed, ledStatus ? HIGH : LOW);
      ledLaatsteWissel = nu;
    }
  }
  else {
    digitalWrite(pinLed, LOW);
    ledStatus = false;
  }
}

// ==========================================
// MAPPING STATE MACHINE
// ==========================================
void runMapping() {
  uint16_t positie = qtr.readLineBlack(sensorValues);

  switch (huidigeStatus) {

    case VOLGEN: {
      bool bL = (sensorValues[0] > 600 || sensorValues[1] > 600);
      bool bR = (sensorValues[6] > 600 || sensorValues[7] > 600);

      if (bL || bR) {
        snapLinks = bL; snapRechts = bR;
        startNaarKruispunt(baseSpeed);
        break;
      }
      if (isDoodlopend()) {
        encoderTellerL = encoderTellerR = 0;
        huidigeStatus = DOORRIJDEN_UTURN;
        break;
      }

      int error = (int)positie - 3500;
      float dyn = constrain(baseSpeed - abs(error) * 0.025f, (float)minBochSpeed, (float)baseSpeed);
      int cor = (int)(error * Kp + (error - lastError) * Kd);
      lastError = error;
      rijden((int)dyn, cor);
      break;
    }

    case NAAR_KRUISPUNT: {
      if (sensorValues[0] > 600 || sensorValues[1] > 600) snapLinks  = true;
      if (sensorValues[6] > 600 || sensorValues[7] > 600) snapRechts = true;

      int zwartTel = 0;
      for (int i = 0; i < SensorCount; i++) if (sensorValues[i] > 600) zwartTel++;
      long gemTicks = (encoderTellerL + encoderTellerR) / 2;

      if (zwartTel >= 6) {
        if (gemTicks >= eindvlakTicks) {
          finishMapping();
        } else {
          setMotorLinks(baseSpeed);
          setMotorRechts(baseSpeed);
        }
        break;
      }

      if (gemTicks < doorrijTicks) {
        setMotorLinks(baseSpeed);
        setMotorRechts(baseSpeed);
        break;
      }

      bool kanS = (sensorValues[3] > 600 || sensorValues[4] > 600);

      if (snapLinks)       { startDraai(-baseSpeed, baseSpeed); }
      else if (kanS)       { huidigeStatus = VOLGEN; }
      else if (snapRechts) { startDraai(baseSpeed, -baseSpeed); }
      else                 { startUTurnMapping(); }
      break;
    }

    case DRAAIEN: {
      uint16_t draaiPositie = qtr.readLineBlack(sensorValues);
      if (!lijnVerlaten) {
        if (sensorValues[3] < 300 && sensorValues[4] < 300) lijnVerlaten = true;
      } else if (sensorValues[3] > 500 || sensorValues[4] > 500) {
        lastError = (int)draaiPositie - 3500;
        huidigeStatus = VOLGEN;
        break;
      }
      long draaiTicks = (encoderTellerL + encoderTellerR) / 2;
      if (!draaiTeruggedraaid && draaiTicks >= maxDraaiTicks) {
        bool lijnZichtbaar = false;
        for (int i = 1; i <= 6; i++) if (sensorValues[i] > 500) { lijnZichtbaar = true; break; }
        if (lijnZichtbaar) {
          lastError = (int)draaiPositie - 3500;
          huidigeStatus = VOLGEN;
        } else {
          encoderTellerL = encoderTellerR = 0;
          setMotorLinks(-draaiSpdL);
          setMotorRechts(-draaiSpdR);
          draaiTeruggedraaid = true;
          lijnVerlaten = false;
        }
      }
      break;
    }

    case UTURN: {
      uint16_t uPositie = qtr.readLineBlack(sensorValues);
      if (!lijnVerlaten) {
        if (sensorValues[3] < 300 && sensorValues[4] < 300) lijnVerlaten = true;
      } else if (sensorValues[3] > 500 || sensorValues[4] > 500) {
        lastError = (int)uPositie - 3500;
        huidigeStatus = VOLGEN;
      }
      break;
    }

    case DOORRIJDEN:
      if ((encoderTellerL + encoderTellerR) / 2 >= doorrijTicks) huidigeStatus = VOLGEN;
      break;

    case DOORRIJDEN_UTURN:
      setMotorLinks(baseSpeed);
      setMotorRechts(baseSpeed);
      if ((encoderTellerL + encoderTellerR) / 2 >= doorrijTicks) startUTurnMapping();
      break;

    case STOP:
      remmen();
      break;
  }
}

// ==========================================
// FINISH FUNCTIE
// ==========================================
void finishMapping() {
  remmen();
  mappingTijdMs = millis() - startTime;
  huidigeStatus = STOP;
  huidigeRobotModus = KLAAR;
}

// ==========================================
// NAVIGATIE HELPERS
// ==========================================

bool isDoodlopend() {
  for (int i = 0; i < SensorCount; i++)
    if (sensorValues[i] > 300) return false;
  return true;
}

void startNaarKruispunt(int spd) {
  encoderTellerL = encoderTellerR = 0;
  setMotorLinks(spd);
  setMotorRechts(spd);
  huidigeStatus = NAAR_KRUISPUNT;
}

void startDraai(int spdL, int spdR) {
  encoderTellerL = encoderTellerR = 0;
  draaiSpdL = spdL;
  draaiSpdR = spdR;
  draaiTeruggedraaid = false;
  setMotorLinks(spdL);
  setMotorRechts(spdR);
  actieStartTijd = millis();
  lijnVerlaten   = false;
  huidigeStatus  = DRAAIEN;
}

void startUTurnMapping() {
  setMotorLinks(baseSpeed);
  setMotorRechts(-baseSpeed);
  lijnVerlaten = false;
  huidigeStatus = UTURN;
}

void rijden(int snelheid, int correctie) {
  setMotorLinks (constrain(snelheid + correctie, -255, 255));
  setMotorRechts(constrain(snelheid - correctie, -255, 255));
}

void setMotorLinks(int s) {
  s = constrain(s, -255, 255);
  if (s > 0)      { digitalWrite(pinAIN1, HIGH); digitalWrite(pinAIN2, LOW);  ledcWrite(pinPWMA,  s); }
  else if (s < 0) { digitalWrite(pinAIN1, LOW);  digitalWrite(pinAIN2, HIGH); ledcWrite(pinPWMA, -s); }
  else            { digitalWrite(pinAIN1, LOW);  digitalWrite(pinAIN2, LOW);  ledcWrite(pinPWMA,  0); }
}

void setMotorRechts(int s) {
  s = constrain(s, -255, 255);
  if (s > 0)      { digitalWrite(pinBIN1, HIGH); digitalWrite(pinBIN2, LOW);  ledcWrite(pinPWMB,  s); }
  else if (s < 0) { digitalWrite(pinBIN1, LOW);  digitalWrite(pinBIN2, HIGH); ledcWrite(pinPWMB, -s); }
  else            { digitalWrite(pinBIN1, LOW);  digitalWrite(pinBIN2, LOW);  ledcWrite(pinPWMB,  0); }
}

void remmen() {
  setMotorLinks(0);
  setMotorRechts(0);
}

void kalibreerRobot() {
  pinMode(pinLed, OUTPUT);
  digitalWrite(pinLed, HIGH);
  for (uint16_t i = 0; i < 400; i++) {
    if      (i < 100) { setMotorLinks(-kalibSpeed); setMotorRechts( kalibSpeed); }
    else if (i < 200) { setMotorLinks( kalibSpeed); setMotorRechts(-kalibSpeed); }
    else if (i < 300) { setMotorLinks(-kalibSpeed); setMotorRechts( kalibSpeed); }
    else              { setMotorLinks( kalibSpeed); setMotorRechts(-kalibSpeed); }
    qtr.calibrate();
  }
  remmen();
  digitalWrite(pinLed, LOW);
  delay(1000);
}