#include <QTRSensors.h>
#include <Adafruit_VL6180X.h>

QTRSensors qtr;
Adafruit_VL6180X vl = Adafruit_VL6180X();
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// --- ESP32 MOTOR PINNEN ---
// Sluit hier de PWM/Signal pinnen van je MOSFETs op aan
const int motorLinksPWM = 16;  
const int motorRechtsPWM = 17; 

// --- INSTELLINGEN ---
const int baseSpeed = 150; // Snelheid rechtdoor (0 tot 255)  --> trager zetten om te testen!
const float Kp = 0.05;     // Bijstuur-agressiviteit  te traag reageren -> 0.1 van maken, en te schakkend -> 0.02 

void setup() {
  Serial.begin(115200); 
  // Motoren instellen als output
  pinMode(motorLinksPWM, OUTPUT);
  pinMode(motorRechtsPWM, OUTPUT);

  // --- ESP32 SENSOR PINNEN ---
  // Pas deze aan naar de pinnen die je fysiek hebt aangesloten. 
  // Let op: GPIO 34, 35, 36 en 39 kunnen alléén als input (sensor) gebruikt worden, dus die zijn hier perfect voor!
  qtr.setTypeAnalog(); 
  qtr.setSensorPins((const uint8_t[]){32, 33, 34, 35, 36, 39, 25, 26}, SensorCount);

  // Kalibratie-fase
  Serial.println("Kalibratie start. Beweeg de robot over de lijn...");
  delay(1000);
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  Serial.println("Kalibratie klaar!");
  delay(2000); // 2 seconden wachten voordat hij gaat rijden
}

void loop() {
  // 1. Lees de lijnpositie (0 tot 7000)  --> 0 = links en 7000 = rechts
  uint16_t position = qtr.readLineBlack(sensorValues);

  // 2. Bereken de foutmarge (3500 is het perfecte midden)
  int error = position - 3500;     // rechts = positieve fout , dus straks min doen(-)

  // 3. Bereken de aanpassing
  int motorSpeedAdjust = error * Kp;

  // 4. Nieuwe snelheden berekenen
  int linkermotorSnelheid = baseSpeed + motorSpeedAdjust;
  int rechtermotorSnelheid = baseSpeed - motorSpeedAdjust;

  // 5. Zorg dat we binnen de 0-255 grenzen blijven
  // Omdat we geen H-brug hebben, kunnen we niet achteruit (onder 0).
  if (linkermotorSnelheid > 255) linkermotorSnelheid = 255;
  if (linkermotorSnelheid < 0) linkermotorSnelheid = 0; // Remt de motor helemaal af bij een scherpe bocht
  
  if (rechtermotorSnelheid > 255) rechtermotorSnelheid = 255;
  if (rechtermotorSnelheid < 0) rechtermotorSnelheid = 0;

  // 6. Stuur de motoren aan
  analogWrite(motorLinksPWM, linkermotorSnelheid);
  analogWrite(motorRechtsPWM, rechtermotorSnelheid);
}

