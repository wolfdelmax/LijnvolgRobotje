#include <QTRSensors.h>
#include <Preferences.h>

// ==========================================
// --- FINETUNING VARIABELEN ---
// ==========================================
float Kp = 0.1;
float Kd = 0.007;
int lastError = 0;

// --- SNELHEID INSTELLINGEN (in procenten) ---
int snelheidMapping   = 40;
int minBochSnelheid   = 25;
int kalibratieSnelheid = 30;

// --- ENCODER AFSTANDEN ---
int doorrijTicks = 200;      // ~13 cm: tot de wielas bij de splitsing is
int eindvlakTicks = 280;     // ~18,5 cm continu zwart → eindvlak bevestigd

// ==========================================

// --- COMPONENTEN ---
QTRSensors qtr;
Preferences preferences;

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

// --- ENCODER PINS ---
const int pinEncoderL = 34;
const int pinEncoderR = 35;
volatile long encoderTellerL = 0;
volatile long encoderTellerR = 0;
const float mmPerTick = (43.0 * PI) / 205.0;

void IRAM_ATTR encoderISR_L() { encoderTellerL++; }
void IRAM_ATTR encoderISR_R() { encoderTellerR++; }

// --- SCHAKELAAR ---
const int pinModeSchakelaar = 1;

// --- PATH OPSLAG ---
const int MAX_PAD_LENGTE = 150;
char pad[MAX_PAD_LENGTE];
int padLengte = 0;

// --- STATUS ---
enum RobotStatus { VOLGEN, NAAR_KRUISPUNT, DRAAIEN, DOORRIJDEN, DOORRIJDEN_UTURN, STOP };
RobotStatus huidigeStatus = VOLGEN;

// --- BEREKENDE SNELHEDEN ---
int baseSpeed;
int minBochSpeed;
int kalibSpeed;

// --- TIMING & FLAGS ---
unsigned long actieStartTijd = 0;
bool lijnVerlaten = false;
unsigned long startTime = 0;
bool isFinished = false;

// --- KRUISPUNT DETECTIE (snapshot, blijft updaten tijdens NAAR_KRUISPUNT) ---
bool snapLinks = false;
bool snapRechts = false;

// --- EVENT LOG ---
const int MAX_EVENTS = 200;
struct EventEntry {
  unsigned long tijd;
  char event;
  int sensorLinks;
  int sensorRechts;
  int error;
};
EventEntry eventLog[MAX_EVENTS];
int eventIndex = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  preferences.begin("robot-data", false);

  bool dataKlaar = preferences.getBool("isKlaar", false);

  if (dataKlaar) {
    Serial.println("\n--- OPGESLAGEN DATA GEVONDEN ---");

    int opgeslagenPadLengte = preferences.getInt("padLen", 0);
    char opgeslagenPad[MAX_PAD_LENGTE];
    preferences.getBytes("pad", opgeslagenPad, MAX_PAD_LENGTE);

    Serial.print("Pad ("); Serial.print(opgeslagenPadLengte); Serial.print(" stappen): ");
    for (int i = 0; i < opgeslagenPadLengte; i++) Serial.print(opgeslagenPad[i]);
    Serial.println();

    unsigned long raceTijd = preferences.getULong("raceTijd", 0);
    Serial.print("Racetijd: "); Serial.print(raceTijd / 1000.0, 2); Serial.println(" seconden");

    int opgeslagenEvents = preferences.getInt("eventCnt", 0);
    EventEntry opgeslagenLog[MAX_EVENTS];
    preferences.getBytes("events", opgeslagenLog, sizeof(opgeslagenLog));

    Serial.println("\nTijd(ms),Event,SensorL,SensorR,Error");
    for (int i = 0; i < opgeslagenEvents; i++) {
      Serial.print(opgeslagenLog[i].tijd);
      Serial.print(","); Serial.print(opgeslagenLog[i].event);
      Serial.print(","); Serial.print(opgeslagenLog[i].sensorLinks);
      Serial.print(","); Serial.print(opgeslagenLog[i].sensorRechts);
      Serial.print(","); Serial.println(opgeslagenLog[i].error);
    }

    Serial.println("--- EINDE DATA ---");
    Serial.println("Geheugen gewist voor volgende run.");

    preferences.putBool("isKlaar", false);
    preferences.end();

    while (true) { delay(1000); }
  }

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

  pinMode(pinEncoderL, INPUT_PULLUP);
  pinMode(pinEncoderR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinEncoderL), encoderISR_L, RISING);
  attachInterrupt(digitalPinToInterrupt(pinEncoderR), encoderISR_R, RISING);

  qtr.setTypeRC();
  qtr.setSensorPins(sensorPinnen, SensorCount);

  kalibreerRobot();

  Serial.println("\n=== MAPPING GESTART ===");
  startTime = millis();

  preferences.clear();
}

void loop() {
  if (isFinished) return;

  if (digitalRead(pinModeSchakelaar) == HIGH) {
    remmen();
    return;
  }

  uint16_t positie = qtr.readLineBlack(sensorValues);

  switch (huidigeStatus) {

    case VOLGEN: {
      bool buitenLinks  = (sensorValues[0] > 600 || sensorValues[1] > 600);
      bool buitenRechts = (sensorValues[6] > 600 || sensorValues[7] > 600);

      // --- Begin van kruispunt of eindvlak? ---
      if (buitenLinks || buitenRechts) {
        snapLinks  = buitenLinks;
        snapRechts = buitenRechts;
        startNaarKruispunt();
        break;
      }

      // --- Doodlopend ---
      if (isDoodlopend()) {
        encoderTellerL = 0;
        encoderTellerR = 0;
        huidigeStatus = DOORRIJDEN_UTURN; // Ga nu eerst doorrijden in plaats van direct draaien
        break;
      }

      // --- Normaal lijn volgen met PD ---
      int error = (int)positie - 3500;
      float dynamischeSnelheid = baseSpeed - (abs(error) * 0.025);
      dynamischeSnelheid = constrain(dynamischeSnelheid, minBochSpeed, baseSpeed);

      int correctie = (error * Kp) + ((error - lastError) * Kd);
      lastError = error;

      rijden((int)dynamischeSnelheid, correctie);
      break;
    }

    case NAAR_KRUISPUNT: {
      // 1. Blijf snapshots updaten (vangt scheve aanrijding op)
      if (sensorValues[0] > 600 || sensorValues[1] > 600) snapLinks = true;
      if (sensorValues[6] > 600 || sensorValues[7] > 600) snapRechts = true;

      // 2. Eindvlak-check: zien we een groot zwart vlak?
      int zwartTel = 0;
      for (int i = 0; i < SensorCount; i++) {
        if (sensorValues[i] > 600) zwartTel++;
      }

      long gemTicks = (encoderTellerL + encoderTellerR) / 2;

      if (zwartTel >= 6) {
        if (gemTicks >= eindvlakTicks) {
          Serial.println(">> EINDVAK gedetecteerd!");
          logEventEntry('F', 0);
          finishMapping();
        } else {
          // Blijf rechtdoor rijden en meten
          setMotorLinks(baseSpeed);
          setMotorRechts(baseSpeed);
        }
        break;
      }

      // 3. Geen eindvlak: wacht tot wielas op splitsing staat
      if (gemTicks < doorrijTicks) {
        setMotorLinks(baseSpeed);
        setMotorRechts(baseSpeed);
        break;
      }

      // 4. Beslissing maken volgens always-left (L → S → R)
      bool kanRechtdoor = (sensorValues[3] > 600 || sensorValues[4] > 600);

      if (snapLinks) {
        pad[padLengte++] = 'L';
        Serial.println("→ LINKS");
        logEventEntry('L', 0);
        startDraaiLinks();
      }
      else if (kanRechtdoor) {
        pad[padLengte++] = 'S';
        Serial.println("→ RECHTDOOR");
        logEventEntry('S', 0);
        huidigeStatus = VOLGEN;
      }
      else if (snapRechts) {
        pad[padLengte++] = 'R';
        Serial.println("→ RECHTS");
        logEventEntry('R', 0);
        startDraaiRechts();
      }
      else {
        startOmdraaien();
      }
      break;
    }

    case DRAAIEN:
      qtr.readLineBlack(sensorValues);

      if (!lijnVerlaten) {
        if (sensorValues[3] < 300 && sensorValues[4] < 300) {
          lijnVerlaten = true;
        }
      }
      else if (sensorValues[3] > 500 || sensorValues[4] > 500) {
        huidigeStatus = VOLGEN;
      }

      if (millis() - actieStartTijd > 1500) {
        huidigeStatus = VOLGEN;
      }
      break;

    case DOORRIJDEN:
      if ((encoderTellerL + encoderTellerR) / 2 >= doorrijTicks) {
        huidigeStatus = VOLGEN;
      }
      break;

    case DOORRIJDEN_UTURN:
      // Laat de motoren vooruit draaien tijdens de extra afstand
      setMotorLinks(baseSpeed);
      setMotorRechts(baseSpeed);

      // Controleer of de robot de doorrijTicks heeft gehaald
      if ((encoderTellerL + encoderTellerR) / 2 >= doorrijTicks) {
        startOmdraaien(); // Zet de U-turn in
      }
      break;

    case STOP:
      remmen();
      break;
  }
}

// === NAVIGATIE HELPERS ===

bool isDoodlopend() {
  for (int i = 0; i < SensorCount; i++) {
    if (sensorValues[i] > 300) return false;
  }
  return true;
}

void startNaarKruispunt() {
  encoderTellerL = 0;
  encoderTellerR = 0;
  setMotorLinks(baseSpeed);
  setMotorRechts(baseSpeed);
  huidigeStatus = NAAR_KRUISPUNT;
}

// === DRAAI ACTIES ===

void startDraaiLinks() {
  setMotorLinks(-baseSpeed);
  setMotorRechts(baseSpeed);
  actieStartTijd = millis();
  lijnVerlaten = false;
  huidigeStatus = DRAAIEN;
}

void startDraaiRechts() {
  setMotorLinks(baseSpeed);
  setMotorRechts(-baseSpeed);
  actieStartTijd = millis();
  lijnVerlaten = false;
  huidigeStatus = DRAAIEN;
}

void startOmdraaien() {
  if (padLengte < MAX_PAD_LENGTE) {
    pad[padLengte++] = 'U';
    Serial.println("→ U-TURN (doodlopen)");
    logEventEntry('U', 0);
  }
  setMotorLinks(baseSpeed);
  setMotorRechts(-baseSpeed); // Draait naar rechts (klok mee)
  actieStartTijd = millis();
  lijnVerlaten = false;
  huidigeStatus = DRAAIEN;
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

// === FINISH & OPSLAG ===

void logEventEntry(char event, int error) {
  if (eventIndex < MAX_EVENTS) {
    eventLog[eventIndex].tijd = millis() - startTime;
    eventLog[eventIndex].event = event;
    eventLog[eventIndex].sensorLinks = sensorValues[0];
    eventLog[eventIndex].sensorRechts = sensorValues[7];
    eventLog[eventIndex].error = error;
    eventIndex++;
  }
}

void finishMapping() {
  remmen();
  isFinished = true;

  unsigned long raceTime = millis() - startTime;

  preferences.putInt("padLen", padLengte);
  preferences.putBytes("pad", pad, MAX_PAD_LENGTE);
  preferences.putULong("raceTijd", raceTime);
  preferences.putInt("eventCnt", eventIndex);
  preferences.putBytes("events", eventLog, sizeof(eventLog));
  preferences.putBool("isKlaar", true);
  preferences.end();

  Serial.println("\n=== MAPPING KLAAR ===");
  Serial.print("Pad lengte: ");
  Serial.println(padLengte);
  Serial.print("Pad: ");
  for (int i = 0; i < padLengte; i++) {
    Serial.print(pad[i]);
  }
  Serial.println("\n");
  Serial.println("Sluit USB uit en herstart voor output te zien.");
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