#include <QTRSensors.h>
#include <Wire.h>
#include <Adafruit_VL6180X.h>
#include <Preferences.h> //slaat de kaart op

// --- COMPONENTEN ---
QTRSensors qtr;
Adafruit_VL6180X vl = Adafruit_VL6180X();
Preferences preferences;

// --- PINNEN ---
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
const int motorLinksPWM = 16;  
const int motorRechtsPWM = 17; 

// --- SNELHEID INSTELLINGEN (PROCENT) ---
int snelheidMapping = 40;
int snelheidRace = 80; 
int baseSpeed;                     

// --- PD INSTELLINGEN --- 
float Kp = 0.05;     
float Kd = 0.3;      
int lastError = 0;   

// --- PATH SOLVING (DE KAART) ---
char pad[150];       // Opslag voor afslagen: 'L', 'R', 'S'
int padLengte = 0;
int stapIndex = 0;   // Voor het afspelen in ronde 2/3
const int stopAfstand = 60;

// --- TIMING & STATUS ---
int huidigePoging = 1; 
bool isFinished = false;
unsigned long startTime = 0;
unsigned long finishTimer = 0;
const int vierkantDetectieTijd = 350; // Voor het 40x40cm zwarte vlak; deze naargelang snelheid moet nog worden verbeterd

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // 1. Geheugen laden
  preferences.begin("robot-data", false);
  huidigePoging = preferences.getInt("poging", 1);
  padLengte = preferences.getInt("padLengte", 0);
  if (padLengte > 0) {
    preferences.getBytes("pad", pad, 150);
  }

  // 2. ToF Initialisatie
  if (!vl.begin()) {
    Serial.println("ToF Fout!");
    while (1); 
  }

  // 3. Snelheid instellen
  int pct = (huidigePoging == 1) ? snelheidMapping : snelheidRace;
  baseSpeed = (pct * 255) / 100;

  pinMode(motorLinksPWM, OUTPUT);
  pinMode(motorRechtsPWM, OUTPUT);
  qtr.setTypeRC(); 
  qtr.setSensorPins((const uint8_t[]){32, 33, 34, 35, 36, 39, 25, 26}, SensorCount);

  kalibreerRobot();
  
  // VISUALISATIE: Laat het geladen pad zien bij opstarten
  Serial.println("--- ROBOT STATUS ---");
  Serial.print("POGING: "); Serial.println(huidigePoging);
  toonPad(); 
  Serial.println("--------------------");

  startTime = millis();
}

void loop() {
  if (isFinished) return;

  // 1. ToF Veiligheid (Object detectie)
  if (objectGedetecteerd()) {
    remmen();
    // print output fixen
    return; 
  }

  // 2. Stop-vak detectie (Zwart 40x40cm)
  // Negeer de eerste 4 sec voor de rand van het startvak, misschien minder?.
  if (millis() - startTime > 1500) {
    if (checkStopVak()) {
      finishActie();
      return;
    }
  }

  // 3. Navigatie & Lijnvolgen
  uint16_t positie = qtr.readLineBlack(sensorValues);

  // Check op splitsing (Kruispunt, T of Y)
  if (sensorValues[0] > 700 && sensorValues[7] > 700) {
    verwerkSplitsing();
  } 
  // Check op doodlopend eind (Geen lijn meer)
  else if (isDoodlopend()) {
    if (huidigePoging == 1) {
       verwerkDoodlopend();
    }
  } 
  // Standaard Lijnvolgen
  else {
    int error = (int)positie - 3500;
    int correctie = berekenPD(error);
    rijden(correctie);
  }
  // 4. Visualisatie (voor de Serial Plotter)
  plotGegevens(error, correctie);
}

// --- VISUALISATIE FUNCTIE ---

void toonPad() {
  Serial.print("Huidig Pad (");
  Serial.print(padLengte);
  Serial.print(" stappen): ");
  
  if (padLengte == 0) {
    Serial.println("LEEG (Nog geen mapping beschikbaar)");
  } else {
    for (int i = 0; i < padLengte; i++) {
      Serial.print(pad[i]);
      if (i < padLengte - 1) Serial.print(" -> ");
    }
    Serial.println();
  }
}

// --- NAVIGATIE FUNCTIES ---

void verwerkSplitsing() {
  // Rij een klein stukje door om de sensoren boven het midden van het kruispunt te krijgen
  analogWrite(motorLinksPWM, baseSpeed);
  analogWrite(motorRechtsPWM, baseSpeed);
  delay(50); 
  qtr.readLineBlack(sensorValues);

  if (huidigePoging == 1) {
    // MAPPING MODUS: Altijd links proberen (Left Hand Rule)
    if (sensorValues[0] > 700) {
      pad[padLengte++] = 'L';
      draaiLinks();
    } else if (sensorValues[3] > 700 || sensorValues[4] > 700) { 
      pad[padLengte++] = 'S';
      doorrijden();
    } else {
      pad[padLengte++] = 'R';
      draaiRechts();
    }
    optimaliseerPad(); // Direct opschonen als we een 'U' hebben toegevoegd
<<<<<<< HEAD
    toonpad();
=======
    toonPad();
>>>>>>> 9f5124e85d31bb3fdc671862a4cffa97922d9907
  } else {
    // RACE MODUS: Volg de opgeslagen kaart
    char actie = pad[stapIndex++];
    if (actie == 'L') draaiLinks();
    else if (actie == 'R') draaiRechts();
    else doorrijden();
  }
}

void verwerkDoodlopend() {
  pad[padLengte++] = 'U';
  omdraaien();
  optimaliseerPad();
  toonPad();
}

void optimaliseerPad() {
  // Als we een U-turn hebben gemaakt, kunnen we het pad verkorten
  if (padLengte < 3 || pad[padLengte - 2] != 'U') return;

  // Maze solving logica: vervang de foute afslag door de kortere weg
  // Bijvoorbeeld: Links + U-turn + *rechts* = Eigenlijk rechtdoor (S) > wordt dit daadwerkelijk uitgevoerd door de code

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

// --- BEWEGINGEN ---

void draaiLinks() {
  analogWrite(motorLinksPWM, 0);
  analogWrite(motorRechtsPWM, baseSpeed);
  delay(200);
  while (sensorValues[3] < 500 && sensorValues[4] < 500) {
    qtr.readLineBlack(sensorValues);
  }
}

void draaiRechts() {
  analogWrite(motorLinksPWM, baseSpeed);
  analogWrite(motorRechtsPWM, 0);
  delay(200);
  while (sensorValues[3] < 500 && sensorValues[4] < 500) {
    qtr.readLineBlack(sensorValues);
  }
}

void omdraaien() {
  analogWrite(motorLinksPWM, baseSpeed);
  analogWrite(motorRechtsPWM, 0);
  delay(500); // Draai ruim over de 90 graden heen
  while (sensorValues[3] < 500 && sensorValues[4] < 500) {
    qtr.readLineBlack(sensorValues);
  }
}

void doorrijden() {
  analogWrite(motorLinksPWM, baseSpeed);
  analogWrite(motorRechtsPWM, baseSpeed);
  delay(150); // Rij over de dwarslijn heen
}

// --- HULPFUNCTIES ---

bool isDoodlopend() {
  for (int i = 0; i < SensorCount; i++) {
    if (sensorValues[i] > 300) return false;
  }
  return true;
}

bool checkStopVak() {
  int zwart = 0;
  for (int i = 0; i < SensorCount; i++) if (sensorValues[i] > 800) zwart++;
  if (zwart >= 7) {
    if (finishTimer == 0) finishTimer = millis();
    if (millis() - finishTimer > vierkantDetectieTijd) return true;
  } else { finishTimer = 0; }
  return false;
}

void finishActie() {
  remmen();
  isFinished = true;
  int volgende = huidigePoging + 1;
  if (volgende > 3) volgende = 1;
  
  preferences.putInt("poging", volgende);
  preferences.putInt("padLengte", padLengte);
  preferences.putBytes("pad", pad, 150);
  preferences.end();
  Serial.println("FINISH! Kaart opgeslagen.");
  toonPad();
}

void rijden(int correctie) {
  analogWrite(motorLinksPWM, constrain(baseSpeed + correctie, 0, 255));
  analogWrite(motorRechtsPWM, constrain(baseSpeed - correctie, 0, 255));
}

int berekenPD(int error) {
  int correctie = (error * Kp) + ((error - lastError) * Kd);
  lastError = error;
  return correctie;
}

bool objectGedetecteerd() {
  return (vl.readRangeStatus() == VL6180X_ERROR_NONE && vl.readRange() < stopAfstand);
}

void remmen() { analogWrite(motorLinksPWM, 0); analogWrite(motorRechtsPWM, 0); }

void kalibreerRobot() {
  for (uint16_t i = 0; i < 400; i++) qtr.calibrate();
}

void plotGegevens(int error, int correctie) {
  Serial.print("Error:");
  Serial.print(error);
  Serial.print(",");
  Serial.print("Correctie:");
  Serial.print(correctie * 10); // Herschaling van de correctie 
  Serial.println(); 
}

// KALIBRATIE......................

void kalibreerRobot() {
  pinMode(2, OUTPUT); // Pin 2 is de ingebouwde LED op de meeste ESP32 boards
  digitalWrite(2, HIGH); // Zet LED aan: START KALIBRATIE
  
  Serial.println("KALIBRATIE GESTART: Beweeg de robot over de lijn!");

  // Deze loop duurt ongeveer 5 tot 10 seconden.
  // Verschuif de robot in deze tijd rustig van links naar rechts over de lijn.
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }

  digitalWrite(2, LOW); // Zet LED uit: KALIBRATIE KLAAR
  Serial.println("KALIBRATIE KLAAR! Zet hem neer, hij gaat zo rijden.");
  
  delay(2000); // Geef jezelf 2 seconden om je handen weg te halen
}