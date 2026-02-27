#include <QTRSensors.h>
#include <Wire.h>
#include <Adafruit_VL6180X.h>

// --- COMPONENTEN ---
QTRSensors qtr;
Adafruit_VL6180X vl = Adafruit_VL6180X();

// --- PINNEN ---
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
const int motorLinksPWM = 16;  
const int motorRechtsPWM = 17; 

// --- PD INSTELLINGEN ---
const int baseSpeed = 150; 
float Kp = 0.05;     
float Kd = 0.3;      
int lastError = 0;   

// --- TOF INSTELLINGEN ---
const int stopAfstand = 50; 

void setup() {
  Serial.begin(115200); 
  Wire.begin();

  if (!vl.begin()) {
    Serial.println("ToF sensor niet gevonden!");
    while (1); 
  }

  pinMode(motorLinksPWM, OUTPUT);
  pinMode(motorRechtsPWM, OUTPUT);

  qtr.setTypeRC(); 
  qtr.setSensorPins((const uint8_t[]){32, 33, 34, 35, 36, 39, 25, 26}, SensorCount);

  kalibreerRobot();
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