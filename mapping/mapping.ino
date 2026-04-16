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

bool snapLinks  = false;
bool snapRechts = false;

unsigned long actieStartTijd = 0;
bool lijnVerlaten = false;

bool vorigeSchakelaarHoog = true;

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
void printAlleData(unsigned long raceTijdMs);
void resetVoorRace();

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(pinModeSchakelaar, INPUT_PULLUP);

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
  preferences.clear();
  preferences.end();

  huidigeRobotModus = MAPPING;

  kalibreerRobot();

  Serial.println("\n=== MAPPING MODUS ===");
  Serial.println("Zet schakelaar LOW om te starten...");

  startTime = millis();
}

void loop() {
  bool schakelaarHoog = (digitalRead(pinModeSchakelaar) == HIGH);

  if (huidigeRobotModus == KLAAR) {
    remmen();
    return;
  }

  if (huidigeRobotModus == WACHT_RACE) {
    remmen();
    if (vorigeSchakelaarHoog && !schakelaarHoog) {
      Serial.println("\n=== RACE GESTART ===");
      resetVoorRace();
      delay(200);  // korte settle-tijd zodat eerste sensor-read stabiel is
    }
    vorigeSchakelaarHoog = schakelaarHoog;
    return;
  }

  vorigeSchakelaarHoog = schakelaarHoog;

  if (schakelaarHoog) {
    remmen();
    return;
  }

  if (huidigeRobotModus == MAPPING) runMapping();
  else if (huidigeRobotModus == RACE) runRace();
}

// ==========================================
// COMPLETE RESET VOOR RACE
// ==========================================
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

  // Forceer eerste sensor read om waarden te initialiseren
  qtr.readLineBlack(sensorValues);
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
          Serial.println(">> EINDVAK!");
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

      if (snapLinks)       { pad[padLengte++] = 'L'; Serial.println("→ LINKS");     logMapping('L', 0); startDraai(-baseSpeed, baseSpeed); }
      else if (kanS)       { pad[padLengte++] = 'S'; Serial.println("→ RECHTDOOR"); logMapping('S', 0); huidigeStatus = VOLGEN; }
      else if (snapRechts) { pad[padLengte++] = 'R'; Serial.println("→ RECHTS");    logMapping('R', 0); startDraai(baseSpeed, -baseSpeed); }
      else                 { startUTurnMapping(); }
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

  // Debug: print positie en sensor-samenvatting elke 500ms
  static unsigned long laatsteDebugPrint = 0;
  if (millis() - laatsteDebugPrint > 500) {
    laatsteDebugPrint = millis();
    Serial.print("[RACE] pos="); Serial.print(positie);
    Serial.print(" s0="); Serial.print(sensorValues[0]);
    Serial.print(" s3="); Serial.print(sensorValues[3]);
    Serial.print(" s4="); Serial.print(sensorValues[4]);
    Serial.print(" s7="); Serial.print(sensorValues[7]);
    Serial.print(" status="); Serial.print(huidigeStatus);
    Serial.print(" idx="); Serial.println(raceIndex);
  }

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
          Serial.println(">> EINDVAK tijdens race!");
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
        Serial.print("→ Race "); Serial.print(raceIndex); Serial.print(": "); Serial.println(inst);
        logRace(inst, raceIndex - 1);

        if      (inst == 'L') startDraai(-raceSpeed,  raceSpeed);
        else if (inst == 'R') startDraai( raceSpeed, -raceSpeed);
        else if (inst == 'S') huidigeStatus = VOLGEN;
        else if (inst == 'U') startDraai( raceSpeed, -raceSpeed);
      } else {
        Serial.println(">> Pad op, onverwachte splitsing - rechtdoor");
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

  Serial.println("\n=== MAPPING KLAAR ===");
  Serial.print("Pad ("); Serial.print(padLengte); Serial.print("): ");
  for (int i = 0; i < padLengte; i++) Serial.print(pad[i]);
  Serial.println();
  Serial.print("Verkort ("); Serial.print(verkortLengte); Serial.print("): ");
  for (int i = 0; i < verkortLengte; i++) Serial.print(verkortPad[i]);
  Serial.println();
  Serial.println("\n>> Zet robot terug op start.");
  Serial.println(">> Zet schakelaar HIGH, dan LOW om race te starten.");

  huidigeRobotModus = WACHT_RACE;
  vorigeSchakelaarHoog = false;
}

void finishRace() {
  remmen();

  unsigned long raceTijdMs = millis() - raceStartTime;

  Serial.println("\n=== RACE KLAAR ===");
  Serial.print("Race tijd: "); Serial.print(raceTijdMs / 1000.0, 2); Serial.println(" sec");

  printAlleData(raceTijdMs);

  huidigeRobotModus = KLAAR;
}

void printAlleData(unsigned long raceTijdMs) {
  Serial.println("\n========== MAPPING DATA ==========");
  Serial.print("Mapping tijd: "); Serial.print(mappingTijdMs / 1000.0, 2); Serial.println(" sec");
  Serial.print("Pad ("); Serial.print(padLengte); Serial.print("): ");
  for (int i = 0; i < padLengte; i++) Serial.print(pad[i]);
  Serial.println();
  Serial.print("Verkort ("); Serial.print(verkortLengte); Serial.print("): ");
  for (int i = 0; i < verkortLengte; i++) Serial.print(verkortPad[i]);
  Serial.println();

  Serial.println("\n--- Mapping events ---");
  Serial.println("Tijd,Event,SensL,SensR,Extra");
  for (int i = 0; i < mappingLogIdx; i++) {
    Serial.print(mappingLog[i].tijd); Serial.print(",");
    Serial.print(mappingLog[i].event); Serial.print(",");
    Serial.print(mappingLog[i].sensorLinks); Serial.print(",");
    Serial.print(mappingLog[i].sensorRechts); Serial.print(",");
    Serial.println(mappingLog[i].extra);
  }

  Serial.println("\n========== RACE DATA ==========");
  Serial.print("Race tijd: "); Serial.print(raceTijdMs / 1000.0, 2); Serial.println(" sec");
  Serial.print("Pad ("); Serial.print(padLengte); Serial.print("): ");
  for (int i = 0; i < padLengte; i++) Serial.print(pad[i]);
  Serial.println();
  Serial.print("Verkort ("); Serial.print(verkortLengte); Serial.print("): ");
  for (int i = 0; i < verkortLengte; i++) Serial.print(verkortPad[i]);
  Serial.println();

  Serial.println("\n--- Race events ---");
  Serial.println("Tijd,Event,SensL,SensR,PadIdx");
  for (int i = 0; i < raceLogIdx; i++) {
    Serial.print(raceLog[i].tijd); Serial.print(",");
    Serial.print(raceLog[i].event); Serial.print(",");
    Serial.print(raceLog[i].sensorLinks); Serial.print(",");
    Serial.print(raceLog[i].sensorRechts); Serial.print(",");
    Serial.println(raceLog[i].extra);
  }
  Serial.println("\n========== EINDE ==========");
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
  setMotorLinks(spdL);
  setMotorRechts(spdR);
  actieStartTijd = millis();
  lijnVerlaten   = false;
  huidigeStatus  = DRAAIEN;
}

void startUTurnMapping() {
  if (padLengte < MAX_PAD_LENGTE) {
    pad[padLengte++] = 'U';
    Serial.println("→ U-TURN");
    logMapping('U', 0);
  }
  startDraai(baseSpeed, -baseSpeed);
}

// ==========================================
// MOTOR CONTROL
// ==========================================

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

// ==========================================
// EVENT LOGGING
// ==========================================

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

// ==========================================
// KALIBRATIE
// ==========================================

void kalibreerRobot() {
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  Serial.println("KALIBRATIE...");
  for (uint16_t i = 0; i < 400; i++) {
    if      (i < 100) { setMotorLinks(-kalibSpeed); setMotorRechts( kalibSpeed); }
    else if (i < 200) { setMotorLinks( kalibSpeed); setMotorRechts(-kalibSpeed); }
    else if (i < 300) { setMotorLinks(-kalibSpeed); setMotorRechts( kalibSpeed); }
    else              { setMotorLinks( kalibSpeed); setMotorRechts(-kalibSpeed); }
    qtr.calibrate();
    if (i % 50 == 0) Serial.print(".");
  }
  remmen();
  digitalWrite(2, LOW);
  Serial.println("\nKALIBRATIE KLAAR!");
  delay(1000);
}

// ==========================================
// PAD OPTIMALISATIE
// ==========================================

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