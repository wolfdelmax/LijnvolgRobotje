// ==========================================
// DRAAI KALIBRATIE PROGRAMMA
// ==========================================
// Meet hoeveel encoder-ticks nodig zijn per graad draaien
//
// GEBRUIK:
// 1. Zet robot op de grond met referentiepunt (tape)
// 2. Pas TICKS_PER_DRAAI aan hieronder
// 3. Upload
// 4. Schakelaar HOOG = wachten, LAAG = draai uitvoeren
// 5. Na elke draai: meet met gradenboog hoeveel graden werkelijk gedraaid
// 6. Herhaal met verschillende TICKS_PER_DRAAI tot je 90° precies hebt
// ==========================================

// --- PAS DEZE AAN ---
int TICKS_PER_DRAAI = 200;   // aantal ticks per test
int draaiSnelheidPct = 30;   // snelheid in % (lager = preciezer, te laag = stilstand door wrijving)
bool draaiLinks = true;      // true = links, false = rechts

// ==========================================
// --- MOTOR PINS (TB6612) ---
const int pinAIN1 = 16, pinAIN2 = 4,  pinPWMA = 18;
const int pinBIN1 = 17, pinBIN2 = 5,  pinPWMB = 19;
const int pwmFreq = 5000, pwmResolution = 8;

// --- ENCODERS ---
const int pinEncoderL = 34;
const int pinEncoderR = 35;
volatile long encoderTellerL = 0;
volatile long encoderTellerR = 0;

void IRAM_ATTR encoderISR_L() { encoderTellerL++; }
void IRAM_ATTR encoderISR_R() { encoderTellerR++; }

// --- SCHAKELAAR & LED ---
const int pinModeSchakelaar = 1;
const int pinLed = 2;

int draaiSnelheid;
bool vorigeSchakelaarHoog = true;

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(pinModeSchakelaar, INPUT_PULLUP);
  pinMode(pinLed, OUTPUT);
  digitalWrite(pinLed, LOW);

  pinMode(pinAIN1, OUTPUT); pinMode(pinAIN2, OUTPUT);
  pinMode(pinBIN1, OUTPUT); pinMode(pinBIN2, OUTPUT);
  ledcAttach(pinPWMA, pwmFreq, pwmResolution);
  ledcAttach(pinPWMB, pwmFreq, pwmResolution);

  pinMode(pinEncoderL, INPUT_PULLUP);
  pinMode(pinEncoderR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinEncoderL), encoderISR_L, RISING);
  attachInterrupt(digitalPinToInterrupt(pinEncoderR), encoderISR_R, RISING);

  draaiSnelheid = (draaiSnelheidPct * 255) / 100;

  Serial.println("\n=== DRAAI KALIBRATIE ===");
  Serial.print("Target ticks: "); Serial.println(TICKS_PER_DRAAI);
  Serial.print("Snelheid: ");     Serial.print(draaiSnelheidPct); Serial.println("%");
  Serial.print("Richting: ");     Serial.println(draaiLinks ? "LINKS" : "RECHTS");
  Serial.println("Schakelaar LAAG = draai uitvoeren");
}

void loop() {
  bool schakelaarHoog = (digitalRead(pinModeSchakelaar) == HIGH);

  // Edge detectie: HOOG -> LAAG = start draai
  if (vorigeSchakelaarHoog && !schakelaarHoog) {
    voerDraaiUit();
  }
  vorigeSchakelaarHoog = schakelaarHoog;

  // LED knippert langzaam in ruststand
  digitalWrite(pinLed, (millis() / 500) % 2);
}

void voerDraaiUit() {
  Serial.println("\n--- START DRAAI ---");

  encoderTellerL = 0;
  encoderTellerR = 0;

  // Motoren aan in juiste richting
  if (draaiLinks) {
    setMotorLinks(-draaiSnelheid);
    setMotorRechts(draaiSnelheid);
  } else {
    setMotorLinks(draaiSnelheid);
    setMotorRechts(-draaiSnelheid);
  }

  // LED aan tijdens draai
  digitalWrite(pinLed, HIGH);

  // Wacht tot gemiddelde tick-telling het doel bereikt
  while (((encoderTellerL + encoderTellerR) / 2) < TICKS_PER_DRAAI) {
    delay(1);
  }

  // Remmen
  setMotorLinks(0);
  setMotorRechts(0);
  digitalWrite(pinLed, LOW);

  // Resultaat
  Serial.print("Ticks links: ");  Serial.println(encoderTellerL);
  Serial.print("Ticks rechts: "); Serial.println(encoderTellerR);
  Serial.print("Gemiddeld: ");    Serial.println((encoderTellerL + encoderTellerR) / 2);
  Serial.println("--> Meet nu met gradenboog hoeveel graden gedraaid!");
  Serial.println("    Zet schakelaar HOOG, lijn robot opnieuw uit, dan LAAG voor volgende test.");
}

// === MOTOR CONTROL ===
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