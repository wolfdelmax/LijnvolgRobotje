# Lijnvolg-robot met mapping

ESP32-gebaseerde lijnvolger die een parcours met vertakkingen kan verkennen. Gebruikt 8 QTR-sensoren voor lijndetectie, twee DC-motoren met encoders voor aandrijving en odometrie, en optioneel een VL6180X ToF-sensor voor obstakeldetectie.

## Repo-structuur

De repo bevat drie Arduino-sketches:

- [50/50PROCENT/50PROCENT.ino](50/50PROCENT/50PROCENT.ino) — basisversie: lijnvolgen + mapping zonder obstakeldetectie.
- [50+OBJECT/50_OBJECT/50_OBJECT.ino](50+OBJECT/50_OBJECT/50_OBJECT.ino) — zelfde gedrag plus obstakelontwijking via VL6180X ToF-sensor.
- [Gradenhoekenticks/testticksdraaien/testticksdraaien.ino](Gradenhoekenticks/testticksdraaien/testticksdraaien.ino) — kalibratie-sketch om `ticksPerGraad` te bepalen voor jouw motor/wielen-combinatie.

Upload één van de twee eerste sketches als hoofdprogramma; de derde is enkel een afstellingstool.

## Werking

De robot volgt een zwarte lijn met een PD-regelaar. Bij detectie van een vertakking (donkere sensoren aan een buitenkant + minimum aantal totale donkere sensoren) gaat hij naar het kruispunt-centrum, beslist welke richting te nemen en draait. Bij een doodloper voert hij een U-turn uit. Komt er een obstakel (cilinder Ø20 cm) op de lijn te liggen, dan wijkt de robot er rechts omheen via een symmetrische uitwijk-route op een aparte lagere snelheid en pikt daarna de lijn weer op. *(Obstakelontwijking enkel in de `50_OBJECT`-sketch.)*

Bij opstart kalibreert de robot eerst de QTR-sensoren door 4× links/rechts heen-en-weer te draaien op `kalibratieSnelheid` (~10 seconden). Pas daarna gaat hij in mapping-modus.

## Modi

Via de BOOT-knop op de ESP32 (indrukken tijdens 2-seconden venster bij opstart, terwijl de LED knippert):

- **Always-left** (default): bij een keuze gaat hij links → rechtdoor → rechts → U-turn. LED brandt continu.
- **Always-right**: bij een keuze gaat hij rechts → rechtdoor → links → U-turn. LED uit.

De aan/uit-schakelaar (pin 1) start en stopt de robot tijdens een run.

## Hardware-pinout

| Functie | Pin(s) |
|---|---|
| QTR-sensoren (8×) | 23, 15, 32, 27, 26, 14, 12, 13 |
| Motor links (TB6612: AIN1/AIN2/PWMA) | 16, 4, 18 |
| Motor rechts (TB6612: BIN1/BIN2/PWMB) | 17, 5, 19 |
| Encoder links / rechts | 34 / 35 |
| BOOT-knop (modus-keuze) | 0 |
| Aan/uit-schakelaar | 1 |
| Status-LED | 2 |
| VL6180X I²C (SDA / SCL) | 21 / 22 |

> ⚠️ Pin 1 is op de meeste ESP32-boards ook TX0. Als de schakelaar aan staat tijdens flashen kan dat de upload verstoren — zet hem in de "uit"-stand voor het uploaden.

## Vereiste libraries

In de Arduino IDE installeren via Library Manager:

- **QTRSensors** (Pololu)
- **Adafruit VL6180X** *(alleen nodig voor de `50_OBJECT`-sketch)*

Board: ESP32 Dev Module (of een variant die `ledcAttach` ondersteunt — werkt met ESP32 Arduino core 3.x).

## Tuning

De belangrijkste parameters staan bovenaan in beide hoofdsketches:

- `Kp`, `Kd`: PD-regelaar voor lijnvolgen
- `snelheidMapping`: rijsnelheid in procenten
- `snelheidDraaien`: rotatiesnelheid bij kruispunt-draaien
- `kruispuntDrempel`: aantal donkere sensoren nodig voor kruispunt-detectie
- `maxDraaiGraden`: max rotatie tijdens een kruispunt-draai voor hij begint terug te zoeken
- `lijnDetectieIdx`: hoe vroeg een lijn als "gevonden" wordt gezien tijdens rotatie
- `ticksPerGraad`: encoder-ticks per graad rotatie — bepaal met de `testticksdraaien`-sketch

## Obstakel omzeilen (VL6180X ToF)

*Enkel in [50_OBJECT.ino](50+OBJECT/50_OBJECT/50_OBJECT.ino).*

Een VL6180X ToF-sensor (I²C op SDA=21, SCL=22) meet vooraan continu de afstand tot eventuele obstakels. De sensor draait in **continuous mode** met een meet-periode van 50 ms, zodat het uitlezen non-blocking is en het lijnvolgen niet vertraagt. Lezingen waarvan de status niet OK is worden genegeerd (de vorige geldige meting blijft staan).

Wanneer de gemeten afstand kleiner wordt dan `omzeilDrempelMm` start de robot een vaste uitwijk-sequentie (altijd langs rechts) op `omzeilSnelheid`:

| Fase | State | Beweging | Stopconditie |
|---|---|---|---|
| 1 | `OBSTAKEL_DRAAI_UIT` | rechts draaien | `omzeilHoekUit` graden |
| 2 | `OBSTAKEL_LANGS` | rechtdoor langs cilinder | `omzeilZijTicks` ticks |
| 3 | `OBSTAKEL_DRAAI_TERUG` | links draaien | `omzeilHoekTerug` graden |
| 4 | `OBSTAKEL_NAAR_LIJN` | rechtdoor; lijn-check pas na `omzeilLijnMinTicks` | lijn gezien (sensor 0-3) **óf** `omzeilZijTicks` (timeout) |
| 5 | `VOLGEN` | PD-regelaar trekt uitlijning recht | — |

In fase 4 wordt de lijn-detectie pas geactiveerd na `omzeilLijnMinTicks` ticks rechtdoor. Dit voorkomt dat schaduwen of randjes van de cilinder een vroege false-positive veroorzaken (waarna de robot dacht dat hij in een doodloper zat en een U-turn startte). Vindt hij de lijn niet binnen `omzeilZijTicks`, dan keert hij sowieso terug naar `VOLGEN` zodat de PD-regelaar het overneemt.

Tijdens het omzeilen worden kruispunt- en doodloperdetectie automatisch overgeslagen. Na het hervatten van `VOLGEN` geldt een cooldown (`omzeilCooldownMs`) zodat de ToF niet meteen opnieuw triggert.

Als de VL6180X niet gedetecteerd wordt bij opstart blijft de obstakellogica uit en gedraagt de robot zich exact als de basis-sketch zonder ToF.

### Omzeil-parameters

- `omzeilDrempelMm` (default 120): afstand in mm waarbij uitwijken start
- `omzeilHoekUit` (default 70°) / `omzeilHoekTerug` (default 65°): draaihoeken (empirisch getuned — meetkundig zou de tweede groter moeten zijn dan de eerste, maar kleine onnauwkeurigheden in `ticksPerGraad` en motor-symmetrie compenseren dat)
- `omzeilZijTicks` (default 700): maximum-afstand zowel in fase 2 (langs cilinder) als fase 4 (terugweg-timeout) — **moet getuned worden op de robot**
- `omzeilSnelheid` (default 60 %): snelheid voor de hele uitwijk-procedure (zowel draaien als rechtdoor) — apart van `snelheidMapping` zodat de manoeuvre rustiger kan
- `omzeilLijnMinTicks` (default 525): aantal ticks rechtdoor in fase 4 voor er naar de lijn gekeken wordt
- `omzeilCooldownMs` (default 5000): wachttijd na omzeilen voor de sensor opnieuw mag triggeren

## `ticksPerGraad` kalibreren

De [testticksdraaien.ino](Gradenhoekenticks/testticksdraaien/testticksdraaien.ino)-sketch laat de robot op commando een vast aantal ticks draaien. Procedure:

1. Zet de robot op de grond met een referentielijntje (tape).
2. Pas `TICKS_PER_DRAAI` aan in de sketch (bv. 200) en upload.
3. Schakelaar **HOOG** = wachten, schakelaar **LAAG** = draai uitvoeren.
4. Meet met een gradenboog hoeveel graden hij werkelijk draaide.
5. Bereken `ticksPerGraad = TICKS_PER_DRAAI / werkelijke_graden` en vul die in de hoofdsketch in.

Herhaal tot 90° draaien betrouwbaar 90° geeft.
