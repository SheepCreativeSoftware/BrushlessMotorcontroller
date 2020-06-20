# BrushlessMotorcontroller

Umfasende Hilfe von der Installation bis hin zum Herunterladen auf einen Arduino findest du im [Wiki](https://github.com/SheepCreativeSoftware/BrushlessMotorcontroller/wiki)
***
## Beschreibung
Grundsätzliche Funktionsweise ist es, über zweit Taster einen Motor, in Stufen, die Drehzahl hoch und runter fahren zulassen.
Die Anzahl der möglichen Stufen ist anpassbar.

Der Motor wird dabei über eine Heli Fahrregler mit Brushless Motor gesteuert. Dieser hat ein Recht lineares verhalten mit nur einer Drehrichtung.
Der verwendete Fahrregler hat außerdem einen Drehzahl Impulsausgang über welche die Drehzahl angezeigt werden kann.

Außerdem wird die Spannung des Akkus überwacht.

Über ein I2C Display können die einzelnen Daten angezeigt werden.
1. Akkuspannung
2. Stufe
3. Drehzahl
![Display](https://raw.githubusercontent.com/SheepCreativeSoftware/BrushlessMotorcontroller/master/Images/Testaufbau2.jpg)
***
### Initialisierung
Zusätzlich lässt sich der Fahrregler initialisieren.
Hierfür drückt man beide Tasten, für Hoch und Runter gleichzeitig und schaltet den Arduino bzw. die Schaltung ein. Dann wird der Fahrregler mit 100% angesteuert. Daraufhin muss man ein paar Sekunden, auf den Fahrregler warten (piepen) und danach lässt man die Tasten los.
Die Ansteuerung geht daraufhin auf 0%.
Nach ein paar Sekunden piept der Fahrregler erneut.
Die Initialisierung ist abgeschlossen.

***
## Verwendete Hardware
* [Arduino Nano](https://store.arduino.cc/arduino-nano)
* [Heliregler Hobbywing Platinium V4 60A](https://www.hobbywing.com/goods.php?id=481&filter_attr=)
* Brushless Motor
* I2C OLED Display 128*64 mit ssd1306 Treiber z.B. [Amazon](https://www.amazon.de/gp/product/B07J2QWF43/ref=ppx_yo_dt_b_asin_title_o00_s00?ie=UTF8&psc=1)
* Diverse Widerstände und Taster ([Siehe Schaltplan](https://github.com/SheepCreativeSoftware/BrushlessMotorcontroller/wiki/5.-Der-Schaltplan))

***
## Feedback
Fehler kannst du hier berichten: [Issues](https://github.com/SheepCreativeSoftware/BrushlessMotorcontroller/issues)<br/>
Danach auf "New Issue"
(hierfür wird ein Github Konto benötigt)

***
>BrushlessMotorcontroller
>
>Motorcontroller um einen Regler mit Brushlessmotor anzusteuern und per Tastendruck die Drehzahl zu verändern Copyright (C) 2020 Marina Egner | info AT sheepindustries DOT de
>
>This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
>
>This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
>
>You should have received a copy of the GNU General Public License along with this program. 
>If not, see https://www.gnu.org/licenses/. [In Deutsch]: https://www.gnu.de/documents/gpl-3.0.de.html

