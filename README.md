# Lijnvolg-robot met mapping

ESP32-gebaseerde lijnvolger die een parcours met vertakkingen kan verkennen. Gebruikt 8 QTR-sensoren voor lijndetectie, twee DC-motoren met encoders voor aandrijving en odometrie.

## Werking

De robot volgt een zwarte lijn met een PD-regelaar. Bij detectie van een vertakking (donkere sensoren aan een buitenkant + minimum aantal totale donkere sensoren) gaat hij naar het kruispunt-centrum, beslist welke richting te nemen en draait. Bij een doodloper voert hij een U-turn uit.

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


50 procent : mapping 26041038 (dagmaanduur)