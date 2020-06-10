/************************************ 
 * BrushlessMotorcontroller v0.0.1
 * Date: 10.06.2020 | 00:25
 * <Truck Light and function module>
 * Copyright (C) 2020 Marina Egner <info@sheepindustries.de>
 *
 * This program is free software: you can redistribute it and/or modify it 
 * under the terms of the GNU General Public License as published by the 
 * Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program. 
 * If not, see <https://www.gnu.org/licenses/>.
 ************************************/

/************************************
 * Programm Konfiguration
 ************************************/
#define version 1                   		//Version

//Ändere diesen Wert für unterschiedliche Debuging level
#define debugLevel 3						// | >2 = Serial Monitor

/************************************
 * Zusätzliche Dateien einbinden
 ************************************/
#include <Servo.h>							//Bibliothek für Servo PWM Ausgabe
/************************************
 * Definition IO Pins
 ************************************/
// TODO: setup correct pins
//Pinout Arduino Nano:
//Serial 0+1 (kein HW Serial) | Interrupt 2+3 | PWM 3, 5, 6, 9, 10, 11 | LED 13 | I2C A4(SDA) + A5(SCL) |
//Servo Lib deaktiviert PWM funktionalität für pin 9 und 10

//Inputs
//Pin 0+1 Reservieren für Serielle Kommunikation über USB!
#define inTasterHoch 3            			//Pin des Taster um die Drehzahl zu erhöhen
#define inTasterRunter 4            		//Pin des Taster um die Drehzahl zu verringern


//Outputs
#define outMotorRegler 9                    //Pin für Servo PWM Signal für Motor Regler

/************************************
 * Definition und Initialiierung
 * Globale Variablen, Klassen und Funktionen
 ************************************/
//Globale Programm Variablen



//Funktionen


//Klassen


void setup() {								// Setup Code, wird einmalig am Start ausgeführt
	/************************************
	* Setup Inputs 
	************************************/
	//pinMode(inFunction1ControlPPM, INPUT_PULLUP);
	//TODO: Pins definieren
	/************************************
	* Setup Outputs 
	************************************/
	//pinMode(outParkingLight, OUTPUT);
			
	#if (debugLevel >=2)					//Bedingte Kompilierung
		SerialUSB.begin(9600);  			//Starte Serielle Kommunikation per USB
	#endif
}

void loop() {                       		// Hauptcode, wiederholt sich zyklisch     
	 
}
