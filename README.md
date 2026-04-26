# Lijnvolg-robot met mapping

ESP32-gebaseerde lijnvolger die een parcours met vertakkingen kan verkennen. Gebruikt 8 QTR-sensoren voor lijndetectie, twee DC-motoren met encoders voor aandrijving en odometrie, en een VL6180X ToF-sensor voor obstakeldetectie.

## Werking

De robot volgt een zwarte lijn met een PD-regelaar. Bij detectie van een vertakking (donkere sensoren aan een buitenkant + minimum aantal totale donkere sensoren) gaat hij naar het kruispunt-centrum, beslist welke richting te nemen en draait. Bij een doodloper voert hij een U-turn uit. Komt er een obstakel (cilinder Ø20 cm) op de lijn te liggen, dan wijkt de robot er rechts omheen via een symmetrische uitwijk-route op een aparte lagere snelheid en pikt daarna de lijn weer op.

## Modi

Via de BOOT-knop op de ESP32 (indrukken tijdens 2-seconden venster bij opstart, terwijl de LED knippert):

- **Always-left** (default): bij een keuze gaat hij links → rechtdoor → rechts → U-turn. LED brandt continu.
- **Always-right**: bij een keuze gaat hij rechts → rechtdoor → links → U-turn. LED uit.

De aan/uit-schakelaar (pin 1) start en stopt de robot tijdens een run.

## Tuning

De belangrijkste parameters staan bovenaan in [mapping.ino](mapping/mapping.ino):

- `Kp`, `Kd`: PD-regelaar voor lijnvolgen
- `snelheidMapping`: rijsnelheid in procenten
- `snelheidDraaien`: rotatiesnelheid bij kruispunt-draaien
- `kruispuntDrempel`: aantal donkere sensoren nodig voor kruispunt-detectie
- `maxDraaiGraden`: max rotatie tijdens een kruispunt-draai voor hij begint terug te zoeken
- `lijnDetectieIdx`: hoe vroeg een lijn als "gevonden" wordt gezien tijdens rotatie

## Obstakel omzeilen (VL6180X ToF)

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

Als de VL6180X niet gedetecteerd wordt bij opstart blijft de obstakellogica uit en gedraagt de robot zich exact als de oude versie zonder ToF.

### Omzeil-parameters

- `omzeilDrempelMm` (default 120): afstand in mm waarbij uitwijken start
- `omzeilHoekUit` (default 70°) / `omzeilHoekTerug` (default 75°): draaihoeken (empirisch getuned — meetkundig zou de tweede groter moeten zijn dan de eerste, maar kleine onnauwkeurigheden in `ticksPerGraad` en motor-symmetrie compenseren dat)
- `omzeilZijTicks` (default 700): maximum-afstand zowel in fase 2 (langs cilinder) als fase 4 (terugweg-timeout) — **moet getuned worden op de robot**
- `omzeilSnelheid` (default 45 %): snelheid voor de hele uitwijk-procedure (zowel draaien als rechtdoor) — apart van `snelheidMapping` zodat de manoeuvre rustiger kan
- `omzeilLijnMinTicks` (default 450): aantal ticks rechtdoor in fase 4 voor er naar de lijn gekeken wordt
- `omzeilCooldownMs` (default 2000): wachttijd na omzeilen voor de sensor opnieuw mag triggeren

### Vereiste library

In de Arduino IDE installeren via Library Manager: **Adafruit VL6180X**.