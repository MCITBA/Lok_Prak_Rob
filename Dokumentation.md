# Ziele / Aufgabe
- Ein Roboter der auf nur zwei Rädern fährt und sich selbstständig aufrecht hält
    - hierfür wird ein PID-Regler verwendet
    - Daten liefert der MPU-6500
- Kommunikation mit einem Handy ermöglichen
    - BLE + entsprechende
    - Austausch von Sensordaten
    - Einstellung der PID-Parameter
        - einfaches Einstellen und Anpassen übers Handy
    - Vorgabe des Soll-Winkels
    - Lenkeinschlag
- Ermöglichen einer Lenkung
    - Ansteuerung ebenfalls über das Smartphone
    - Idee:
        - Soll-PWM für Motor wird "aufgeteilt"
        - prozentual, je nachdem wie stark gelenkt werden soll
- Zusätzliche Herausforderungen
    - wie kann sichergestellt werden, dass der Roboter auch gerade fährt?
        - Verwendung der Beschleunigungswerte?



# Warum haben wir welche Komponenten ausgewählt
## alte Powerbank
- 10.000 mAh
- Output: 2.1 A bei 5V

## ESP32D
- hat bereits eine Antenne verbaut
- ausreichend hohe Taktrate für die Regelung
- läuft auf 5V - passt also zu unserer Powerbank
- Verwendung eines Entwicklungsboards, um den ESP32 am Roboter zu befestigen

## TT-Motor
- ein Motor pro Rad, damit wir lenken können
- müssen für 3V ausgelegt sein (siehe Motortreiber)
- haben bereits ein Getriebe integriert, das ist notwendig um die drehzahl zu verringern


## Motortreiber - L298N
- I2C kompatibel
- Zur Steuerung der Drehgeschwindigkeit und **Drehrichtung**  <--wichtig!
    - H-Brücke
- kann zwei Motoren gleichzeitig ansteuern
- funktioniert auch noch bei 5V *untere Grenze!*
    - Logische Spannung bei 5V und dann über Jumper auch Antriebsspannung
    - interner Spannungsabfall von ca. 2 V (Spannungsabfall an den Transistoren der doppel H-Brücke)

## Gyroskop und Beschleunigungssensor - MPU-6500
- I2C kompatibel
- je drei Achsen
- für 3 bis 5V ausgelegt
- kleines Modul zur Bestimmung der Beschleunigung und der Verdrehung

## Zusätzliche Komponenten
- Schalter
- - gesamten Roboter an/aus schalten
- Smartphone zur Kommunikation 
    - Vorgaben des Sollwinkels
    - Einstellen der Regelparameter 
    - Lenken

## Zusammenfassung
- alle Komponenten laufen mit 5V
- Kommunikation über I2C möglich

# Schaltkreis
- optionale Kondensatoren hinzugefügt
