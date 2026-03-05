https://projecthub.arduino.cc/anova9347/line-follower-robot-with-pid-controller-01813f

prompt : Help me met het schrijven van de code voor een lijnvolgrobot. Dit zijn de componenten : VL6180X ToF distance ranging sensor die een object dat op het circuit ligt moet herkennen. De 8QTR RC gebruiken we om de lijn te zien met de robot

grote aanpassingen om 11:17 en om 13:41 zeker kijken naar het toegevoegde bericht

zorgt de constant lezing van ToF sensor voor een vertraging in de lus die niet te verwaarlozen valt. Zorgt deze vertraging voor een vermindering in snelheid  dat de QTR sensoren hun werk kunnen doen? Zorgt deze sensor ervoor dat de energieconsumptie van sensoren te hoog ligt en een impact heeft op de spanning over de motoren? Of maakt het weinig uit aangezien het in de tweede en derde ronde we het kunnnen onthouden op een bepaalde plek?

DYNAMISCHE_KP???
dit is de code voor de functie
int berekenPD_Dynamisch(int error) {
  float actueleKp = Kp_basis;

  // Bocht-detectie: Als de fout groter is dan 1500 (scherpe bocht)
  // verhogen we de Kp lineair voor meer stuurkracht.
  if (abs(error) > 1500) {
    // Kp wordt tot 2x zo groot bij een maximale error (3500)
    actueleKp = Kp_basis * (1.0 + (float)abs(error) / 3500.0);
  }

  int afgeleide = error - lastError;
  int correctie = (error * actueleKp) + (afgeleide * Kd);
  
  lastError = error;
  return correctie;
}
dit komt dan in de loop 
  // Standaard Lijnvolgen
  else {
    int error = (int)positie - 3500;
    int correctie = Dynamische_KP(error);
    rijden(correctie);
  }


TO DO:
< rond het object navigeren en zwarte lijn terug vinden>

< dynamische Kp ?>

< een commando toevoeg waarmee je het geheugen handmatig kunt wissen via de Serial Monitor? (Handig als je een foutje hebt gemaakt tijdens de mapping en opnieuw wilt beginnen)>

< LineLossFailsafe>

< Maze solving logica aanvullen (optimaliseerPad()), een soort tabel die bij bepaalde situaties zeker weet wat hij kan doen van sturen om het snelst uit de situatie kan komen. bijvoorbeeld het skippen van de zigag/golven/andere>

< reset knop dat geheugen wist naar poging 1>

< wiel dat stiltaat linken met PID???? >

< monitor die ons informatie geeft over de spanning en andere dingen>


PROBLEMEN:
< >

