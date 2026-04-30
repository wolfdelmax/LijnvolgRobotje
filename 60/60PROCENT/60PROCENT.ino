#include <QTRSensors.h>

// ==========================================
// --- FINETUNING VARIABELEN ---
// ==========================================
float Kp = 0.18;
float Kd = 0.007;
int lastError = 0;

// --- SNELHEID INSTELLINGEN (in procenten) ---
int snelheidMapping    = 55;
int minBochSnelheid    = 25;
int kalibratieSnelheid = 30;
int snelheidDraaien    = 40;  // rotatiesnelheid voor DRAAIEN en UTURN

// --- ENCODER AFSTANDEN ---
int doorrijTicks  = 110;
int eindvlakTicks = 280;

// --- DRAAIEN ---
const float ticksPerGraad = 2.135f;
float maxDraaiGraden = 150.0f; // max rotatie tijdens DRAAIEN
float MinDraaiTicks = 12.5f; // min rotatie voor detectie
const long maxDraaiTicks = (long)(maxDraaiGraden * ticksPerGraad);
int lijnDetectieIdx = 2;        // inner-sensor index voor lijn-detectie tijdens draai
                                // 3 = centrum (3+4), 2 = vroeger (2+5), 1 = nog vroeger (1+6). bv gerbuikt sensore 2 en 5
int kruispuntDrempel = 4;       // min aantal donkere sensoren voor kruispunt-detectie
int donkerDrempel    = 600;     // sensor-waarde drempel om "donker" te zijn

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
const int pinBoot = 0;  // BOOT-knop: ingedrukt bij opstart = always-right modus

int baseSpeed, minBochSpeed, kalibSpeed, draaiSpeed;
bool altijdRechts = false;

enum RobotModus { MAPPING, KLAAR };
RobotModus huidigeRobotModus = MAPPING;

enum RobotStatus { VOLGEN, NAAR_KRUISPUNT, DRAAIEN, UTURN, DOORRIJDEN_UTURN, STOP };
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
  pinMode(pinBoot, INPUT_PULLUP);
  pinMode(pinLed, OUTPUT);

  // Modus-keuze venster: 2s lang LED snel knipperen, BOOT indrukken = always-right
  unsigned long modusEinde = millis() + 2000;
  while (millis() < modusEinde) {
    digitalWrite(pinLed, (millis() / 100) % 2);
    if (digitalRead(pinBoot) == LOW) altijdRechts = true;
  }
  digitalWrite(pinLed, altijdRechts ? LOW : HIGH);

  baseSpeed    = (snelheidMapping    * 255) / 100;
  minBochSpeed = (minBochSnelheid    * 255) / 100;
  kalibSpeed   = (kalibratieSnelheid * 255) / 100;
  draaiSpeed   = (snelheidDraaien    * 255) / 100;

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
    digitalWrite(pinLed, altijdRechts ? LOW : HIGH);
    return;
  }

  if (schakelaarHoog) {
    remmen();
    digitalWrite(pinLed, altijdRechts ? LOW : HIGH);
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
  digitalWrite(pinLed, altijdRechts ? LOW : HIGH);
}

// ==========================================
// MAPPING STATE MACHINE
// ==========================================
void runMapping() {
  uint16_t positie = qtr.readLineBlack(sensorValues);

  switch (huidigeStatus) {

    case VOLGEN: {
      int donkerTel = 0;
      for (int i = 0; i < SensorCount; i++) if (sensorValues[i] > donkerDrempel) donkerTel++;
      bool bL = (sensorValues[0] > donkerDrempel || sensorValues[1] > donkerDrempel) && donkerTel >= kruispuntDrempel;
      bool bR = (sensorValues[6] > donkerDrempel || sensorValues[7] > donkerDrempel) && donkerTel >= kruispuntDrempel;

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
      if (sensorValues[0] > donkerDrempel || sensorValues[1] > donkerDrempel) snapLinks  = true;
      if (sensorValues[6] > donkerDrempel || sensorValues[7] > donkerDrempel) snapRechts = true;

      int zwartTel = 0;
      for (int i = 0; i < SensorCount; i++) if (sensorValues[i] > donkerDrempel) zwartTel++;
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

      bool kanS = (sensorValues[3] > donkerDrempel || sensorValues[4] > donkerDrempel);

      if (altijdRechts) {
        if (snapRechts)      { startDraai(draaiSpeed, -draaiSpeed); }
        else if (kanS)       { huidigeStatus = VOLGEN; }
        else if (snapLinks)  { startDraai(-draaiSpeed, draaiSpeed); }
        else                 { startUTurnMapping(); }
      } else {
        if (snapLinks)       { startDraai(-draaiSpeed, draaiSpeed); }
        else if (kanS)       { huidigeStatus = VOLGEN; }
        else if (snapRechts) { startDraai(draaiSpeed, -draaiSpeed); }
        else                 { startUTurnMapping(); }
      }
      break;
    }

    case DRAAIEN: {
      uint16_t draaiPositie = qtr.readLineBlack(sensorValues);
      int idxL = lijnDetectieIdx, idxR = 7 - lijnDetectieIdx;
      long draaiTicks = (encoderTellerL + encoderTellerR) / 2;
      const long minDraaiTicks = (long)(MinDraaiTicks * ticksPerGraad);
      if (draaiTicks >= minDraaiTicks) {
        if (!lijnVerlaten) {
          if (sensorValues[idxL] < 300 && sensorValues[idxR] < 300) lijnVerlaten = true;
        } else if (sensorValues[idxL] > 500 || sensorValues[idxR] > 500) {
          lastError = 0;
          huidigeStatus = VOLGEN;
          break;
        }
      }
      if (!draaiTeruggedraaid && draaiTicks >= maxDraaiTicks) {
        bool lijnZichtbaar = false;
        for (int i = 1; i <= 6; i++) if (sensorValues[i] > 500) { lijnZichtbaar = true; break; }
        if (lijnZichtbaar) {
          lastError = 0;
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
      int idxL = lijnDetectieIdx, idxR = 7 - lijnDetectieIdx;
      if (!lijnVerlaten) {
        if (sensorValues[idxL] < 300 && sensorValues[idxR] < 300) lijnVerlaten = true;
      } else if (sensorValues[idxL] > 500 || sensorValues[idxR] > 500) {
        lastError = 0;
        huidigeStatus = VOLGEN;
      }
      break;
    }

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
  setMotorLinks(draaiSpeed);
  setMotorRechts(-draaiSpeed);
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