#include <QTRSensors.h>
#include <Wire.h>
// #include <Adafruit_VL6180X.h>  // UITGESCHAKELD VOOR TESTEN
#include <Preferences.h> 

// ==========================================
// --- 🛠️ FINETUNING VARIABELEN 🛠️ ---
// ==========================================
float Kp = 0.1;
float Kd = 0.007;
int lastError = 0;

int draaiTijd90 = 200;
int doorrijTijd = 150;

// --- OBJECT ONTWIJKING TIMING ---
int ontwijkZijdelings = 300;
int ontwijkVooruit    = 400;
// --- SNELHEID INSTELLINGEN (in procenten) ---
int snelheidMapping   = 40;
int snelheidRace      = 80;
int minBochSnelheid   = 25;
// ==========================================

// --- SCHAKELAAR PIN ---
const int pinModeSchakelaar = 1;

// --- ENUMS VOOR STATUS ---
enum RobotStatus { VOLGEN, DRAAIEN, DOORRIJDEN, STOP, ONTWIJKEN };
RobotStatus huidigeStatus = VOLGEN;

// --- ONTWIJKING STAPPENLIJST ---
struct OntwijkStap { int links; int rechts; int duur; };
OntwijkStap ontwijkStappen[5];
int aantalOntwijkStappen = 5;
int huidigeOntwijkStap = 0;

// --- COMPONENTEN ---
QTRSensors qtr;
// Adafruit_VL6180X vl = Adafruit_VL6180X();  // UITGESCHAKELD VOOR TESTEN
Preferences preferences;

// --- SENSOR PINNEN ---
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
const uint8_t sensorPinnen[] = {23, 15, 32, 27, 26, 14, 12, 13};

// --- MOTOR PINNEN ---
const int pinAIN1 = 16; const int pinAIN2 = 4;  const int pinPWMA = 18; 
const int pinBIN1 = 17; const int pinBIN2 = 5;  const int pinPWMB = 19;  
const int pwmFreq = 5000;
const int pwmResolution = 8;

// --- BEREKENDE SNELHEDEN ---
int baseSpeed;
int minBochSpeed;

// --- PATH SOLVING ---
const int MAX_PAD_LENGTE = 150;
char pad[MAX_PAD_LENGTE];       
int padLengte = 0;
int stapIndex = 0;   
const int stopAfstand = 60;

// --- TIMING & LOGICA ---
unsigned long actieStartTijd = 0;
unsigned long actieDuur = 0;
unsigned long laatsteObjectCheck = 0;
bool lijnVerlaten = false;
const unsigned long draaiTimeout = 1500;
int huidigePoging = 1;
bool isFinished = false;
unsigned long startTime = 0;
unsigned long finishTimer = 0;
const int vierkantDetectieTijd = 350; 

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // 1. SCHAKELAAR
  pinMode(pinModeSchakelaar, INPUT_PULLUP);
  huidigePoging = (digitalRead(pinModeSchakelaar) == LOW) ? 2 : 1;

  // 2. Geheugen laden
  preferences.begin("robot-data", false);
  padLengte = preferences.getInt("padLengte", 0);
  if (padLengte > 0 && padLengte <= MAX_PAD_LENGTE) {
    preferences.getBytes("pad", pad, MAX_PAD_LENGTE);
  } else {
    padLengte = 0; // FIX: Reset als ongeldige waarde in geheugen
  }

  // FIX: Controleer of er een geldig pad is voor race-modus
  if (huidigePoging == 2 && padLengte == 0) {
    Serial.println("FOUT: Geen geldig pad gevonden voor race-modus!");
    Serial.println("Doe eerst een mapping-run (schakelaar open).");
    while (1) delay(1000);
  }

  // 3. ToF Initialisatie - UITGESCHAKELD VOOR TESTEN
  // if (!vl.begin()) {
  //   Serial.println("ToF Fout! Controleer bedrading.");
  //   while (1);
  // }

  // 4. Snelheid berekenen
  int pct = (huidigePoging == 1) ? snelheidMapping : snelheidRace;
  baseSpeed    = (pct * 255) / 100;
  minBochSpeed = (minBochSnelheid * 255) / 100;

  // 5. Motoren instellen
  pinMode(pinAIN1, OUTPUT); pinMode(pinAIN2, OUTPUT);
  pinMode(pinBIN1, OUTPUT); pinMode(pinBIN2, OUTPUT);
  ledcAttach(pinPWMA, pwmFreq, pwmResolution);            
  ledcAttach(pinPWMB, pwmFreq, pwmResolution); 

  // 6. Lijn sensoren instellen
  qtr.setTypeRC(); 
  qtr.setSensorPins(sensorPinnen, SensorCount);

  kalibreerRobot();
  
  Serial.println("--- ROBOT STATUS ---");
  Serial.println(huidigePoging == 1 ? "MODUS: MAPPING (schakelaar open)" : "MODUS: RACE (schakelaar gesloten)");
  toonPad(); 
  Serial.println("--------------------");

  startTime = millis();
}

void loop() {
  if (isFinished) return;

  uint16_t positie = qtr.readLineBlack(sensorValues);

  switch (huidigeStatus) {
    
    case VOLGEN:
      // UITGESCHAKELD VOOR TESTEN
      // if (millis() - laatsteObjectCheck > 100) {
      //   laatsteObjectCheck = millis();
      //   if (objectGedetecteerd()) {
      //     startOntwijken();
      //     return;
      //   }
      // }

      if (millis() - startTime > 1500 && checkStopVak()) {
        finishActie();
        return;
      }

      if (sensorValues[0] > 700 && sensorValues[7] > 700) {
        startVerwerkSplitsing();
      } 
      else if (isDoodlopend() && huidigePoging == 1) {
        startOmdraaien();
      } 
      else {
        int error = (int)positie - 3500;
        float dynamischeSnelheid = baseSpeed - (abs(error) * 0.025);
        dynamischeSnelheid = constrain(dynamischeSnelheid, minBochSpeed, baseSpeed);
        int correctie = (error * Kp) + ((error - lastError) * Kd);
        lastError = error;
        rijden((int)dynamischeSnelheid, correctie);
      }
      break;

    case DRAAIEN:
      qtr.readLineBlack(sensorValues);
      if (!lijnVerlaten) {
        if (sensorValues[3] < 300 && sensorValues[4] < 300) {
          lijnVerlaten = true;
        }
      } else if (sensorValues[3] > 500 || sensorValues[4] > 500) {
        huidigeStatus = VOLGEN;
      }
      if (millis() - actieStartTijd > draaiTimeout) {
        huidigeStatus = VOLGEN;
      }
      break;

    case DOORRIJDEN:
      if (millis() - actieStartTijd > actieDuur) {
        huidigeStatus = VOLGEN;
      }
      break;

    case ONTWIJKEN:
      // FIX: Lees sensoren opnieuw voor lijndetectie
      qtr.readLineBlack(sensorValues);

      if (huidigeOntwijkStap >= aantalOntwijkStappen) {
        setMotorLinks(baseSpeed);
        setMotorRechts(baseSpeed);
        if (sensorValues[2] > 500 || sensorValues[3] > 500 ||
            sensorValues[4] > 500 || sensorValues[5] > 500) {
          Serial.println("ONTWIJKEN: Lijn gevonden, terug naar VOLGEN.");
          huidigeStatus = VOLGEN;
        }
        return;
      }

      if (millis() - actieStartTijd > ontwijkStappen[huidigeOntwijkStap].duur) {
        huidigeOntwijkStap++;
        if (huidigeOntwijkStap < aantalOntwijkStappen) {
          setMotorLinks(ontwijkStappen[huidigeOntwijkStap].links);
          setMotorRechts(ontwijkStappen[huidigeOntwijkStap].rechts);
          actieStartTijd = millis();
          Serial.print("ONTWIJKEN: Stap "); Serial.println(huidigeOntwijkStap + 1);
        }
      }
      break;

    case STOP:
      remmen();
      break;
  }
}

// --- ONTWIJKING STARTEN ---
void startOntwijken() {
  Serial.println("OBJECT GEDETECTEERD! Start ontwijking links.");
  remmen();
  ontwijkStappen[0] = { -baseSpeed,  baseSpeed, draaiTijd90      };
  ontwijkStappen[1] = {  baseSpeed,  baseSpeed, ontwijkZijdelings};
  ontwijkStappen[2] = {  baseSpeed, -baseSpeed, draaiTijd90      };
  ontwijkStappen[3] = {  baseSpeed,  baseSpeed, ontwijkVooruit   };
  ontwijkStappen[4] = {  baseSpeed, -baseSpeed, draaiTijd90      };
  huidigeOntwijkStap = 0;
  setMotorLinks(ontwijkStappen[0].links);
  setMotorRechts(ontwijkStappen[0].rechts);
  actieStartTijd = millis();
  huidigeStatus = ONTWIJKEN;
  Serial.println("ONTWIJKEN: Stap 1");
}

// --- MOTOR AANSTURING ---
void setMotorLinks(int speed) {
  speed = constrain(speed, -255, 255); 
  if (speed > 0) { digitalWrite(pinAIN1, HIGH); digitalWrite(pinAIN2, LOW);  ledcWrite(pinPWMA, speed);  }
  else if (speed < 0) { digitalWrite(pinAIN1, LOW);  digitalWrite(pinAIN2, HIGH); ledcWrite(pinPWMA, -speed); }
  else { digitalWrite(pinAIN1, LOW); digitalWrite(pinAIN2, LOW); ledcWrite(pinPWMA, 0); }
}

void setMotorRechts(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) { digitalWrite(pinBIN1, HIGH); digitalWrite(pinBIN2, LOW);  ledcWrite(pinPWMB, speed);  }
  else if (speed < 0) { digitalWrite(pinBIN1, LOW);  digitalWrite(pinBIN2, HIGH); ledcWrite(pinPWMB, -speed); }
  else { digitalWrite(pinBIN1, LOW); digitalWrite(pinBIN2, LOW); ledcWrite(pinPWMB, 0); }
}

// --- BEWEGINGS ACTIES ---
void rijden(int snelheid, int correctie) {
  setMotorLinks(constrain(snelheid + correctie, -255, 255));
  setMotorRechts(constrain(snelheid - correctie, -255, 255));
}

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
  if (padLengte < MAX_PAD_LENGTE) {  // FIX: Bounds check
    pad[padLengte++] = 'U';
  } else {
    Serial.println("FOUT: Pad array vol!");
    remmen();
    isFinished = true;
    return;
  }
  setMotorLinks(baseSpeed);
  setMotorRechts(-baseSpeed);
  actieStartTijd = millis();
  lijnVerlaten = false;
  huidigeStatus = DRAAIEN;
  optimaliseerPad();
  toonPad();
}

void startDoorrijden() {
  setMotorLinks(baseSpeed);
  setMotorRechts(baseSpeed);
  actieStartTijd = millis();
  actieDuur = doorrijTijd;
  huidigeStatus = DOORRIJDEN;
}

// --- NAVIGATIE LOGICA ---
void startVerwerkSplitsing() {
  if (huidigePoging == 1) {
    qtr.readLineBlack(sensorValues);

    bool linksOpen  = (sensorValues[0] > 700 || sensorValues[1] > 700);
    bool rechtsOpen = (sensorValues[6] > 700 || sensorValues[7] > 700);
    bool rechtdoor  = (sensorValues[3] > 700 || sensorValues[4] > 700);

    if (padLengte < MAX_PAD_LENGTE) {
      if (linksOpen)       { pad[padLengte++] = 'L'; startDraaiLinks();   }
      else if (rechtdoor)  { pad[padLengte++] = 'S'; startDoorrijden();   }
      else if (rechtsOpen) { pad[padLengte++] = 'R'; startDraaiRechts();  }
      else                 { startOmdraaien(); return; }
    } else {
      Serial.println("FOUT: Pad array vol!");
      remmen();
      isFinished = true;
      return;
    }

    optimaliseerPad();
    toonPad();
  } else {
    // Race-modus: volg opgeslagen pad
    if (stapIndex < padLengte) {  // FIX: Bounds check
      char actie = pad[stapIndex++];
      if (actie == 'L') startDraaiLinks();
      else if (actie == 'R') startDraaiRechts();
      else startDoorrijden();
    } else {
      Serial.println("FOUT: Pad index buiten bereik!");
      startDoorrijden();
    }
  }
}

void remmen() { 
  setMotorLinks(0); 
  setMotorRechts(0); 
}

void optimaliseerPad() {
  if (padLengte < 3 || pad[padLengte - 2] != 'U') return;
  char totaal[3] = {pad[padLengte-3], pad[padLengte-2], pad[padLengte-1]};
  char vervanging = ' ';
  if      (totaal[0] == 'L' && totaal[2] == 'R') vervanging = 'S';
  else if (totaal[0] == 'L' && totaal[2] == 'S') vervanging = 'R';
  else if (totaal[0] == 'R' && totaal[2] == 'L') vervanging = 'S';
  else if (totaal[0] == 'S' && totaal[2] == 'L') vervanging = 'R';
  else if (totaal[0] == 'S' && totaal[2] == 'S') vervanging = 'U';
  else if (totaal[0] == 'L' && totaal[2] == 'L') vervanging = 'S';
  if (vervanging != ' ') { padLengte -= 3; pad[padLengte++] = vervanging; }
}

bool isDoodlopend() { 
  for (int i = 0; i < SensorCount; i++) if (sensorValues[i] > 300) return false; 
  return true; 
}

bool checkStopVak() {
  int zwart = 0;
  for (int i = 0; i < SensorCount; i++) if (sensorValues[i] > 800) zwart++;
  if (zwart >= 7) {
    if (finishTimer == 0) finishTimer = millis();
    if (millis() - finishTimer > vierkantDetectieTijd) return true;
  } else {
    finishTimer = 0; 
  }
  return false;
}

void finishActie() {
  remmen();
  isFinished = true;
  if (huidigePoging == 1) {
    preferences.putInt("padLengte", padLengte);
    preferences.putBytes("pad", pad, MAX_PAD_LENGTE);
    Serial.println("FINISH! Pad opgeslagen. Zet schakelaar om voor race modus.");
  } else {
    Serial.println("FINISH! Race voltooid.");
  }
  preferences.end();
  toonPad();
}

// UITGESCHAKELD VOOR TESTEN
// bool objectGedetecteerd() {
//   uint8_t range = vl.readRange();
//   uint8_t status = vl.readRangeStatus();
//   return (status == VL6180X_ERROR_NONE && range < stopAfstand);
// }

void toonPad() { 
  Serial.print("Huidig pad: ");
  for (int i = 0; i < padLengte; i++) Serial.print(pad[i]);
  Serial.println();
}

void kalibreerRobot() {
  pinMode(2, OUTPUT); digitalWrite(2, HIGH); 
  Serial.println("KALIBRATIE GESTART...");
  // TIP: Beweeg de robot handmatig heen en weer over de lijn
  for (uint16_t i = 0; i < 400; i++) qtr.calibrate();
  digitalWrite(2, LOW); 
  Serial.println("KALIBRATIE KLAAR!");
  delay(1000);
}