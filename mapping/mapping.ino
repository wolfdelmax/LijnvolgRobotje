#include <QTRSensors.h>
#include <Preferences.h>

// ==========================================
// --- FINETUNING VARIABELEN ---
// ==========================================
float Kp = 0.1;
float Kd = 0.007;
int lastError = 0;

// --- SNELHEID INSTELLINGEN (in procenten) ---
int snelheidMapping    = 40;
int snelheidRace       = 40;
int minBochSnelheid    = 25;
int kalibratieSnelheid = 30;

// --- ENCODER AFSTANDEN ---
int doorrijTicks  = 200;
int eindvlakTicks = 280;

// --- DRAAI CORRECTIE VARIABELEN ---
// Stel hier in hoeveel graden de robot MAX mag draaien voordat de correctie triggert.
// 2.153 ticks per graad (775 ticks / 360°)
// Aanbevolen: tussen 120° en 170°
const float TICKS_PER_GRAAD = 2.153;
int maxDraaiGraden = 150;                                    // ← pas deze waarde aan!
int draaiTicksMax  = (int)(maxDraaiGraden * TICKS_PER_GRAAD); // wordt automatisch berekend
int laatsteSpdL = 0;
int laatsteSpdR = 0;

// --- LED ---
const int pinLed = 2;
const unsigned long ledKnipperMs = 100;

// ==========================================

QTRSensors qtr;
Preferences preferences;

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

int baseSpeed, raceSpeed, minBochSpeed, kalibSpeed;

enum RobotModus { MAPPING, WACHT_RACE, RACE, KLAAR };
RobotModus huidigeRobotModus = MAPPING;

enum RobotStatus { VOLGEN, NAAR_KRUISPUNT, DRAAIEN, DOORRIJDEN, DOORRIJDEN_UTURN, STOP };
RobotStatus huidigeStatus = VOLGEN;

const int MAX_PAD_LENGTE = 150;
char pad[MAX_PAD_LENGTE];
int  padLengte = 0;
char verkortPad[MAX_PAD_LENGTE];
int  verkortLengte = 0;
int  raceIndex = 0;

unsigned long startTime     = 0;
unsigned long raceStartTime = 0;
unsigned long mappingTijdMs = 0;
unsigned long opgeslagenRaceTijd = 0;

bool snapLinks  = false;
bool snapRechts = false;

unsigned long actieStartTijd = 0;
bool lijnVerlaten = false;

bool vorigeSchakelaarHoog = true;

// LED knipperstatus
unsigned long ledLaatsteWissel = 0;
bool ledStatus = false;

const int MAX_EVENTS = 200;
struct EventEntry {
  unsigned long tijd;
  char  event;
  int   sensorLinks;
  int   sensorRechts;
  int   extra;
};
EventEntry mappingLog[MAX_EVENTS];
int        mappingLogIdx = 0;
EventEntry raceLog[MAX_EVENTS];
int        raceLogIdx = 0;

// --- FORWARD DECLARATIES ---
void optimaliseerPad(char* padArray, int &lengte);
void kalibreerRobot();
void runMapping();
void runRace();
void finishMapping();
void finishRace();
void resetVoorRace();
void updateLedRace();

void setup() {
  pinMode(pinModeSchakelaar, INPUT_PULLUP);
  pinMode(pinLed, OUTPUT);
  digitalWrite(pinLed, LOW);

  baseSpeed    = (snelheidMapping    * 255) / 100;
  raceSpeed    = (snelheidRace       * 255) / 100;
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

  preferences.begin("robot-data", false);
  int runState = preferences.getInt("state", 0);

  if (runState == 0) {
    huidigeRobotModus = MAPPING;
    kalibreerRobot();
  } 
  else if (runState == 1) {
    padLengte = preferences.getInt("padLen", 0);
    preferences.getBytes("pad", pad, MAX_PAD_LENGTE);
    verkortLengte = preferences.getInt("vpadLen", 0);
    preferences.getBytes("vpad", verkortPad, MAX_PAD_LENGTE);
    mappingTijdMs = preferences.getULong("mTijd", 0);
    mappingLogIdx = preferences.getInt("mLogCnt", 0);
    preferences.getBytes("mLog", mappingLog, sizeof(mappingLog));

    huidigeRobotModus = WACHT_RACE;
    kalibreerRobot();
  }
  else if (runState == 2) {
    padLengte = preferences.getInt("padLen", 0);
    preferences.getBytes("pad", pad, MAX_PAD_LENGTE);
    verkortLengte = preferences.getInt("vpadLen", 0);
    preferences.getBytes("vpad", verkortPad, MAX_PAD_LENGTE);
    mappingTijdMs = preferences.getULong("mTijd", 0);
    mappingLogIdx = preferences.getInt("mLogCnt", 0);
    preferences.getBytes("mLog", mappingLog, sizeof(mappingLog));

    opgeslagenRaceTijd = preferences.getULong("rTijd", 0);
    raceLogIdx = preferences.getInt("rLogCnt", 0);
    preferences.getBytes("rLog", raceLog, sizeof(raceLog));

    huidigeRobotModus = KLAAR;
  }
  
  preferences.end();
  startTime = millis();
}

void loop() {
  bool schakelaarHoog = (digitalRead(pinModeSchakelaar) == HIGH);

  if (huidigeRobotModus == KLAAR) {
    remmen();
    digitalWrite(pinLed, LOW);

    // Edge-detectie: wissen bij overgang HIGH → LOW
    if (vorigeSchakelaarHoog && !schakelaarHoog) {
      preferences.begin("robot-data", false);
      preferences.clear();
      preferences.end();
      huidigeRobotModus = (RobotModus)99;
    }
    vorigeSchakelaarHoog = schakelaarHoog;
    return;
  }

  if (huidigeRobotModus == WACHT_RACE) {
    remmen();
    digitalWrite(pinLed, LOW);
    if (vorigeSchakelaarHoog && !schakelaarHoog) {
      resetVoorRace();
      delay(200);
    }
    vorigeSchakelaarHoog = schakelaarHoog;
    return;
  }

  vorigeSchakelaarHoog = schakelaarHoog;

  if (schakelaarHoog) {
    remmen();
    digitalWrite(pinLed, LOW);
    return;
  }

  if (huidigeRobotModus == MAPPING) {
    digitalWrite(pinLed, LOW);
    runMapping();
  }
  else if (huidigeRobotModus == RACE) {
    runRace();
    updateLedRace();
  }
}

void resetVoorRace() {
  huidigeRobotModus = RACE;
  huidigeStatus     = VOLGEN;
  raceIndex         = 0;
  raceLogIdx        = 0;
  lastError         = 0;
  encoderTellerL    = 0;
  encoderTellerR    = 0;
  snapLinks         = false;
  snapRechts        = false;
  lijnVerlaten      = false;
  actieStartTijd    = 0;
  raceStartTime     = millis();
  ledLaatsteWissel  = 0;
  ledStatus         = false;

  qtr.readLineBlack(sensorValues);
}

// ==========================================
// LED STURING (alleen in race)
// ==========================================
void updateLedRace() {
  if (huidigeStatus == VOLGEN) {
    digitalWrite(pinLed, HIGH);
    ledStatus = true;
  }
  else if (huidigeStatus == NAAR_KRUISPUNT
        || huidigeStatus == DRAAIEN
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
          logMapping('F', 0);
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

      if (snapLinks)       { pad[padLengte++] = 'L'; logMapping('L', 0); startDraai(-baseSpeed, baseSpeed); }
      else if (kanS)       { pad[padLengte++] = 'S'; logMapping('S', 0); huidigeStatus = VOLGEN; }
      else if (snapRechts) { pad[padLengte++] = 'R'; logMapping('R', 0); startDraai(baseSpeed, -baseSpeed); }
      else                 { startUTurnMapping(); }
      break;
    }

    case DRAAIEN: {
      qtr.readLineBlack(sensorValues);

      long afgelegdeDraai = (abs(encoderTellerL) + abs(encoderTellerR)) / 2;

      // --- CORRECTIE: Als we aan het terugdraaien zijn (pad is al 'S'), wacht tot ticks bereikt ---
      if (padLengte > 0 && pad[padLengte - 1] == 'S' && afgelegdeDraai < draaiTicksMax) {
        // Nog aan het terugdraaien, niet naar lijndetectie kijken
        break;
      }
      if (padLengte > 0 && pad[padLengte - 1] == 'S' && afgelegdeDraai >= draaiTicksMax) {
        // Terugdraai klaar → verder rechtdoor rijden
        huidigeStatus = VOLGEN;
        break;
      }

      // --- CORRECTIE: Te ver gedraaid zonder lijn te vinden? ---
      // Als L of R aftakking was, maar we hebben de max hoek bereikt zonder lijn → terugdraaien
      if (padLengte > 0 && (pad[padLengte - 1] == 'L' || pad[padLengte - 1] == 'R') && afgelegdeDraai >= draaiTicksMax) {
        pad[padLengte - 1] = 'S'; // Overschrijf de L/R naar een S (Rechtdoor)
        logMapping('S', maxDraaiGraden);

        // Start met terugdraaien. Hierdoor beginnen de encoders weer op 0.
        startDraai(-laatsteSpdL, -laatsteSpdR);
        break;
      }
      // --- EINDE CORRECTIE LOGICA ---

      if (!lijnVerlaten) {
        if (sensorValues[3] < 300 && sensorValues[4] < 300) lijnVerlaten = true;
      } else if (sensorValues[3] > 500 || sensorValues[4] > 500) {
        huidigeStatus = VOLGEN;
      }
      if (millis() - actieStartTijd > 1500) huidigeStatus = VOLGEN;
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
// RACE STATE MACHINE
// ==========================================
void runRace() {
  uint16_t positie = qtr.readLineBlack(sensorValues);

  switch (huidigeStatus) {

    case VOLGEN: {
      bool bL = (sensorValues[0] > 600 || sensorValues[1] > 600);
      bool bR = (sensorValues[6] > 600 || sensorValues[7] > 600);

      if (bL || bR) {
        snapLinks = bL; snapRechts = bR;
        startNaarKruispunt(raceSpeed);
        break;
      }
      if (isDoodlopend()) {
        logRace('!', raceIndex);
        encoderTellerL = encoderTellerR = 0;
        huidigeStatus = DOORRIJDEN_UTURN;
        break;
      }

      int error = (int)positie - 3500;
      float dyn = constrain(raceSpeed - abs(error) * 0.025f, (float)minBochSpeed, (float)raceSpeed);
      int cor = (int)(error * Kp + (error - lastError) * Kd);
      lastError = error;
      rijden((int)dyn, cor);
      break;
    }

    case NAAR_KRUISPUNT: {
      int zwartTel = 0;
      for (int i = 0; i < SensorCount; i++) if (sensorValues[i] > 600) zwartTel++;
      long gemTicks = (encoderTellerL + encoderTellerR) / 2;

      if (zwartTel >= 6) {
        if (gemTicks >= eindvlakTicks) {
          logRace('F', raceIndex);
          finishRace();
        } else {
          setMotorLinks(raceSpeed);
          setMotorRechts(raceSpeed);
        }
        break;
      }

      if (gemTicks < doorrijTicks) {
        setMotorLinks(raceSpeed);
        setMotorRechts(raceSpeed);
        break;
      }

      if (raceIndex < verkortLengte) {
        char inst = verkortPad[raceIndex++];
        logRace(inst, raceIndex - 1);

        if      (inst == 'L') startDraai(-raceSpeed,  raceSpeed);
        else if (inst == 'R') startDraai( raceSpeed, -raceSpeed);
        else if (inst == 'S') huidigeStatus = VOLGEN;
        else if (inst == 'U') startDraai( raceSpeed, -raceSpeed);
      } else {
        logRace('?', raceIndex);
        huidigeStatus = VOLGEN;
      }
      break;
    }

    case DRAAIEN:
      qtr.readLineBlack(sensorValues);
      if (!lijnVerlaten) {
        if (sensorValues[3] < 300 && sensorValues[4] < 300) lijnVerlaten = true;
      } else if (sensorValues[3] > 500 || sensorValues[4] > 500) {
        huidigeStatus = VOLGEN;
      }
      if (millis() - actieStartTijd > 1500) huidigeStatus = VOLGEN;
      break;

    case DOORRIJDEN:
      if ((encoderTellerL + encoderTellerR) / 2 >= doorrijTicks) huidigeStatus = VOLGEN;
      break;

    case DOORRIJDEN_UTURN:
      setMotorLinks(raceSpeed);
      setMotorRechts(raceSpeed);
      if ((encoderTellerL + encoderTellerR) / 2 >= doorrijTicks) {
        startDraai(raceSpeed, -raceSpeed);
      }
      break;

    case STOP:
      remmen();
      break;
  }
}

// ==========================================
// FINISH FUNCTIES
// ==========================================

void finishMapping() {
  remmen();
  mappingTijdMs = millis() - startTime;

  memcpy(verkortPad, pad, padLengte);
  verkortLengte = padLengte;
  optimaliseerPad(verkortPad, verkortLengte);

  preferences.begin("robot-data", false);
  preferences.putInt("state", 1);
  preferences.putInt("padLen", padLengte);
  preferences.putBytes("pad", pad, MAX_PAD_LENGTE);
  preferences.putInt("vpadLen", verkortLengte);
  preferences.putBytes("vpad", verkortPad, MAX_PAD_LENGTE);
  preferences.putULong("mTijd", mappingTijdMs);
  preferences.putInt("mLogCnt", mappingLogIdx);
  preferences.putBytes("mLog", mappingLog, sizeof(mappingLog));
  preferences.end();

  huidigeRobotModus = WACHT_RACE;
  vorigeSchakelaarHoog = false;
}

void finishRace() {
  remmen();
  unsigned long raceTijdMs = millis() - raceStartTime;
  opgeslagenRaceTijd = raceTijdMs;

  preferences.begin("robot-data", false);
  preferences.putInt("state", 2);
  preferences.putULong("rTijd", raceTijdMs);
  preferences.putInt("rLogCnt", raceLogIdx);
  preferences.putBytes("rLog", raceLog, sizeof(raceLog));
  preferences.end();

  huidigeRobotModus = KLAAR;
  vorigeSchakelaarHoog = false;
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
  // Reset de encoders om de draai-afstand schoon te meten
  encoderTellerL = 0;
  encoderTellerR = 0;

  // Sla de richtingen op voor een eventuele omkeeractie (correctie)
  laatsteSpdL = spdL;
  laatsteSpdR = spdR;

  setMotorLinks(spdL);
  setMotorRechts(spdR);
  actieStartTijd = millis();
  lijnVerlaten   = false;
  huidigeStatus  = DRAAIEN;
}

void startUTurnMapping() {
  if (padLengte < MAX_PAD_LENGTE) {
    pad[padLengte++] = 'U';
    logMapping('U', 0);
  }
  startDraai(baseSpeed, -baseSpeed);
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

void logMapping(char event, int extra) {
  if (mappingLogIdx < MAX_EVENTS) {
    mappingLog[mappingLogIdx] = { millis() - startTime, event,
                                  (int)sensorValues[0], (int)sensorValues[7], extra };
    mappingLogIdx++;
  }
}

void logRace(char event, int padIdx) {
  if (raceLogIdx < MAX_EVENTS) {
    raceLog[raceLogIdx] = { millis() - raceStartTime, event,
                            (int)sensorValues[0], (int)sensorValues[7], padIdx };
    raceLogIdx++;
  }
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

void optimaliseerPad(char* padArray, int &lengte) {
  bool veranderd = true;
  while (veranderd) {
    veranderd = false;
    for (int i = 0; i < lengte - 2; i++) {
      if (padArray[i + 1] == 'U') {
        char actie = ' ';
        if      (padArray[i] == 'L' && padArray[i+2] == 'R') actie = 'U';
        else if (padArray[i] == 'L' && padArray[i+2] == 'S') actie = 'R';
        else if (padArray[i] == 'R' && padArray[i+2] == 'L') actie = 'U';
        else if (padArray[i] == 'S' && padArray[i+2] == 'L') actie = 'R';
        else if (padArray[i] == 'S' && padArray[i+2] == 'S') actie = 'U';
        else if (padArray[i] == 'L' && padArray[i+2] == 'L') actie = 'S';
        else if (padArray[i] == 'R' && padArray[i+2] == 'R') actie = 'S';
        else if (padArray[i] == 'R' && padArray[i+2] == 'S') actie = 'L';
        else if (padArray[i] == 'S' && padArray[i+2] == 'R') actie = 'L';

        if (actie != ' ') {
          padArray[i] = actie;
          for (int j = i + 1; j < lengte - 2; j++) padArray[j] = padArray[j + 2];
          lengte -= 2;
          veranderd = true;
          break;
        }
      }
    }
  }
}