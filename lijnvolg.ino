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
int snelheidRace = 85; 
int baseSpeed;                     

// --- PD INSTELLINGEN --- 
float Kp = 0.05;     
float Kd = 0.3;      
int lastError = 0;   

// --- TIMING & STATUS ---
int huidigePoging = 1; 
bool isFinished = false;
unsigned long startTime = 0;
unsigned long finishTimer = 0;
const int vierkantDetectieTijd = 300; // Voor het 40x40cm zwarte vlak

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // 1. Geheugen laden
  preferences.begin("robot-data", false);
  huidigePoging = preferences.getInt("poging", 1);
  
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
  
  Serial.print("POGING: "); Serial.println(huidigePoging);
  startTime = millis();
}

void loop() {
  // 1. Controleer op obstakels
  if (objectGedetecteerd()) {
    remmen();
    // We plotten nog steeds, zodat we zien dat de error 0 is bij stilstand
    plotGegevens(0, 0); 
    return; 
  }

  // 2. Berekeningen
  int error = berekenFout();
  int correctie = berekenPD(error);

  // 3. Actie
  rijden(correctie);

  // 4. Visualisatie (voor de Serial Plotter)
  plotGegevens(error, correctie);
}

// --- FUNCTIES ---
bool checkStopVak() {
  int zwartSensoren = 0;
  for (int i = 0; i < SensorCount; i++) {
    if (sensorValues[i] > 800) zwartSensoren++;
  }

  // Als we op een massief zwart vlak rijden (zoals het stopvak van 40cm)
  if (zwartSensoren >= 7) {
    if (finishTimer == 0) finishTimer = millis();
    // Als we al 300ms lang zwart zien, is het GEEN kruispunt maar het stopvak
    if (millis() - finishTimer > vierkantDetectieTijd) return true;
  } else {
    finishTimer = 0; 
  }
  return false;
}

void finishActie() {
  remmen();
  isFinished = true;
  
  // Volgende poging opslaan voor de herstart
  int volgende = huidigePoging + 1;
  if (volgende > 3) volgende = 1; // Na poging 3 resetten naar 1
  
  preferences.putInt("poging", volgende);
  preferences.end();
  
  Serial.println("BESTEMMING BEREIKT. Poging opgeslagen in Flash.");
}

void kalibreerRobot() {
  Serial.println("Kalibratie start...");
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  Serial.println("Klaar! Open nu de Serial Plotter (Ctrl+Shift+L)");
  delay(2000);
}

bool objectGedetecteerd() {
  uint8_t range = vl.readRange();
  uint8_t status = vl.readRangeStatus();
  return (status == VL6180X_ERROR_NONE && range < stopAfstand);
}

int berekenFout() {
  uint16_t position = qtr.readLineBlack(sensorValues);
  return (int)position - 3500;
}

int berekenPD(int error) {
  int afgeleide = error - lastError;
  int correctie = (error * Kp) + (afgeleide * Kd);
  lastError = error; 
  return correctie;
}

void rijden(int correctie) {
  int links = constrain(baseSpeed + correctie, 0, 255);
  int rechts = constrain(baseSpeed - correctie, 0, 255);

  analogWrite(motorLinksPWM, links);
  analogWrite(motorRechtsPWM, rechts);
}

void remmen() {
  analogWrite(motorLinksPWM, 0);
  analogWrite(motorRechtsPWM, 0);
}

void plotGegevens(int error, int correctie) {
  Serial.print("Error:");
  Serial.print(error);
  Serial.print(",");
  Serial.print("Correctie:");
  Serial.print(correctie * 10); // Herschaling van de correctie 
  Serial.println(); 
}