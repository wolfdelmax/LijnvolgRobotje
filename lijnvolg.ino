#include <QTRSensors.h>
#include <Wire.h>
#include <Adafruit_VL6180X.h>
#include <Preferences.h> 

// ==========================================
// --- 🛠️ FINETUNING VARIABELEN 🛠️ ---
// Pas deze waarden aan tijdens het testen:
// ==========================================
float Kp = 0.05;              // Verhoog als hij bochten mist, verlaag als hij hevig waggelt (bijv. 0.04 of 0.06).
int minSnelheid = 30;         // Verhoog (bijv. 40-50) om het binnenste wiel meer te laten rollen in flauwe bochten.

int draaiTijd90 = 200;        // Milliseconden "blind" wegdraaien bij een 90 graden bocht (Links/Rechts).
int draaiTijd180 = 350;       // Milliseconden "blind" wegdraaien voor een U-turn (Doodlopend).
int doorrijTijd = 150;        // Milliseconden "blind" rechtdoor rijden op een kruispunt.
// ==========================================


// --- ENUMS VOOR STATUS ---
enum RobotStatus { VOLGEN, DRAAIEN, DOORRIJDEN, STOP };
RobotStatus huidigeStatus = VOLGEN;

// --- COMPONENTEN ---
QTRSensors qtr;
Adafruit_VL6180X vl = Adafruit_VL6180X();
Preferences preferences;

// --- SENSOR PINNEN ---
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// --- MOTOR PINNEN (ESP32 Core 3.0+) ---
const int pinAIN1 = 4;  const int pinAIN2 = 2;  const int pinPWMA = 15; 
const int pinBIN1 = 16; const int pinBIN2 = 17; const int pinPWMB = 5;  
const int pwmFreq = 5000;
const int pwmResolution = 8;

// --- SNELHEID INSTELLINGEN ---
int snelheidMapping = 40;
int snelheidRace = 80; 
int baseSpeed;

// --- PATH SOLVING (DE KAART) ---
char pad[150];       
int padLengte = 0;
int stapIndex = 0;   
const int stopAfstand = 60;

// --- TIMING & LOGICA ---
unsigned long actieStartTijd = 0;
unsigned long actieDuur = 0;
int huidigePoging = 1; 
bool isFinished = false;
unsigned long startTime = 0;
unsigned long finishTimer = 0;
const int vierkantDetectieTijd = 350; 

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // 1. Geheugen laden
  preferences.begin("robot-data", false);
  huidigePoging = preferences.getInt("poging", 1);
  padLengte = preferences.getInt("padLengte", 0);
  if (padLengte > 0) preferences.getBytes("pad", pad, 150);

  // 2. ToF Initialisatie
  if (!vl.begin()) {
    Serial.println("ToF Fout! Controleer bedrading.");
    while (1); 
  }

  // 3. Snelheid instellen
  int pct = (huidigePoging == 1) ? snelheidMapping : snelheidRace;
  baseSpeed = (pct * 255) / 100;

  // 4. Motoren Instellen
  pinMode(pinAIN1, OUTPUT); pinMode(pinAIN2, OUTPUT);
  pinMode(pinBIN1, OUTPUT); pinMode(pinBIN2, OUTPUT);
  ledcAttach(pinPWMA, pwmFreq, pwmResolution);            
  ledcAttach(pinPWMB, pwmFreq, pwmResolution); 

  // 5. Lijn sensoren instellen
  qtr.setTypeRC(); 
  qtr.setSensorPins((const uint8_t[]){32, 33, 34, 35, 36, 39, 25, 26}, SensorCount);

  kalibreerRobot();
  
  Serial.println("--- ROBOT STATUS ---");
  Serial.print("POGING: "); Serial.println(huidigePoging);
  toonPad(); 
  Serial.println("--------------------");

  startTime = millis();
}

void loop() {
  if (isFinished) return;

  // 1. VEILIGHEID: ToF wordt nu ALTIJD gecheckt, zonder vertraging
  if (objectGedetecteerd()) {
    remmen();
    return; 
  }

  // 2. SENSOREN UITLEZEN
  uint16_t positie = qtr.readLineBlack(sensorValues);

  // 3. STATE MACHINE: Wat is de robot aan het doen?
  switch (huidigeStatus) {
    
    case VOLGEN:
      // Check voor finishvak (pas mogelijk na 1.5s om direct stoppen op startvak te voorkomen)
      if (millis() - startTime > 1500 && checkStopVak()) {
        finishActie();
        return;
      }

      // Detecteer kruispunten/splitsingen (Buitenste sensoren zien zwart)
      if (sensorValues[0] > 700 && sensorValues[7] > 700) {
        startVerwerkSplitsing();
      } 
      // Detecteer doodlopend pad (Geen enkele sensor ziet zwart)
      else if (isDoodlopend() && huidigePoging == 1) {
        startOmdraaien();
      } 
      // Normaal lijnvolgen
      else {
        int error = (int)positie - 3500;
        rijden(berekenP(error));
      }
      break;

    case DRAAIEN:
      // Negeer sensoren totdat de 'blinde tijd' voorbij is
      if (millis() - actieStartTijd > actieDuur) {
        // Blinde tijd is voorbij, zoek nu naar de nieuwe lijn met middelste sensoren
        if (sensorValues[3] > 500 || sensorValues[4] > 500) {
          huidigeStatus = VOLGEN; // Lijn gevonden, ga terug naar lijnvolgen
        }
      }
      break;

    case DOORRIJDEN:
      // Rijdt blind rechtdoor over een kruispunt
      if (millis() - actieStartTijd > actieDuur) {
        huidigeStatus = VOLGEN;
      }
      break;
  }
}

// --- MOTOR AANSTURING ---

void setMotorLinks(int speed) {
  speed = constrain(speed, -255, 255); 
  if (speed > 0) { digitalWrite(pinAIN1, HIGH); digitalWrite(pinAIN2, LOW); ledcWrite(pinPWMA, speed); }
  else if (speed < 0) { digitalWrite(pinAIN1, LOW); digitalWrite(pinAIN2, HIGH); ledcWrite(pinPWMA, -speed); }
  else { digitalWrite(pinAIN1, LOW); digitalWrite(pinAIN2, LOW); ledcWrite(pinPWMA, 0); }
}

void setMotorRechts(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) { digitalWrite(pinBIN1, HIGH); digitalWrite(pinBIN2, LOW); ledcWrite(pinPWMB, speed); }
  else if (speed < 0) { digitalWrite(pinBIN1, LOW); digitalWrite(pinBIN2, HIGH); ledcWrite(pinPWMB, -speed); }
  else { digitalWrite(pinBIN1, LOW); digitalWrite(pinBIN2, LOW); ledcWrite(pinPWMB, 0); }
}

// --- NON-BLOCKING BEWEGINGS ACTIES ---

void rijden(int correctie) {
  setMotorLinks(constrain(baseSpeed + correctie, minSnelheid, 255));
  setMotorRechts(constrain(baseSpeed - correctie, minSnelheid, 255));
}

void startDraaiLinks() {
  setMotorLinks(0);
  setMotorRechts(baseSpeed);
  actieStartTijd = millis();
  actieDuur = draaiTijd90; 
  huidigeStatus = DRAAIEN;
}

void startDraaiRechts() {
  setMotorLinks(baseSpeed);
  setMotorRechts(0);
  actieStartTijd = millis();
  actieDuur = draaiTijd90;
  huidigeStatus = DRAAIEN;
}

void startOmdraaien() {
  pad[padLengte++] = 'U';
  setMotorLinks(baseSpeed);
  setMotorRechts(-baseSpeed); // Achteruit trekken voor scherpe U-turn
  actieStartTijd = millis();
  actieDuur = draaiTijd180;
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
    // Lees direct sensoren opnieuw om type kruising te bepalen
    qtr.readLineBlack(sensorValues);

    if (sensorValues[0] > 700) { pad[padLengte++] = 'L'; startDraaiLinks(); } 
    else if (sensorValues[3] > 700 || sensorValues[4] > 700) { pad[padLengte++] = 'S'; startDoorrijden(); } 
    else { pad[padLengte++] = 'R'; startDraaiRechts(); }
    
    optimaliseerPad(); 
    toonPad();
  } else {
    // Race mode: volg het opgeslagen pad
    char actie = pad[stapIndex++];
    if (actie == 'L') startDraaiLinks();
    else if (actie == 'R') startDraaiRechts();
    else startDoorrijden();
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
  if (totaal[0] == 'L' && totaal[2] == 'R') vervanging = 'S';
  else if (totaal[0] == 'L' && totaal[2] == 'S') vervanging = 'R';
  else if (totaal[0] == 'R' && totaal[2] == 'L') vervanging = 'S';
  else if (totaal[0] == 'S' && totaal[2] == 'L') vervanging = 'R';
  else if (totaal[0] == 'S' && totaal[2] == 'S') vervanging = 'U';
  else if (totaal[0] == 'L' && totaal[2] == 'L') vervanging = 'S';
  
  if (vervanging != ' ') { 
    padLengte -= 3; 
    pad[padLengte++] = vervanging; 
  }
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
  } else finishTimer = 0; 
  return false;
}

void finishActie() {
  remmen();
  isFinished = true;
  int volgende = huidigePoging + 1;
  if (volgende > 2) volgende = 1; // 1 = Map run, 2 = Fast run
  preferences.putInt("poging", volgende);
  preferences.putInt("padLengte", padLengte);
  preferences.putBytes("pad", pad, 150);
  preferences.end();
  Serial.println("FINISH! Gegevens opgeslagen.");
  toonPad();
}

int berekenP(int error) { 
  return error * Kp; 
}

bool objectGedetecteerd() { 
  return (vl.readRangeStatus() == VL6180X_ERROR_NONE && vl.readRange() < stopAfstand); 
}

void toonPad() { 
  Serial.print("Huidig pad: ");
  for (int i = 0; i < padLengte; i++) {
    Serial.print(pad[i]);
  }
  Serial.println();
}

void kalibreerRobot() {
  pinMode(2, OUTPUT); digitalWrite(2, HIGH); 
  Serial.println("KALIBRATIE GESTART...");
  for (uint16_t i = 0; i < 400; i++) qtr.calibrate();
  digitalWrite(2, LOW); 
  Serial.println("KALIBRATIE KLAAR!");
  delay(1000); // Korte pauze voor je de robot loslaat
}