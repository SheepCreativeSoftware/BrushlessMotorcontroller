/*
 * BrushlessMotorcontroller v0.1.0
 * Date: 11.06.2020 | 16:56
 * <Motorcontroller um einen Regler mit Brushlessmotor anzusteuern und per Tastendruck die Drehzahl zu verändern>
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
 */

/************************************
 * Programm Konfiguration
 ************************************/
#define PROG_VERSION 1                   					//Version
#define MOTOR_MIN_PULSE	1000								//Minimale Position des PWM Servo signal in millisekunden | Default: 1000
#define MOTOR_MAX_PULSE	2000								//Maximale Position des PWM Servo signal in millisekunden | Default: 2000
#define MOTOR_STUFEN 10										//Stufenanzahl in welchen der Motor geschaltet werden kann.
#define ZEIT_TASTER_LANG 2000								//Zeit für einen Langen Tasterdruck
	
//Ändere diesen Wert für unterschiedliche Debuging level
#define DEBUGLEVEL 0										//0 = Aus | 1 = N/A | >2 = Serial Monitor

/************************************
 * Zusätzliche Dateien einbinden
 ************************************/
#include <Servo.h>											//Bibliothek für Servo PWM Ausgabe

/************************************
 * Definition IO Pins
 ************************************/
/* Pinout Arduino Nano:
 * Serial 0+1 (kein HW Serial) | Interrupt 2+3 | PWM 3, 5, 6, 9, 10, 11 | LED 13 | I2C A4(SDA) + A5(SCL) |
 * Servo Lib deaktiviert PWM funktionalität für pin 9 und 10
 */
 
//Inputs
//Pin 0+1 Reserviert halten für Serielle Kommunikation über USB!
#define inTasterHoch 3            							//Pin des Taster um die Drehzahl zu erhöhen
#define inTasterRunter 4            						//Pin des Taster um die Drehzahl zu verringern

//Outputs
#define outMotorRegler 9                    				//Pin für Servo PWM Signal für Motor Regler
#define statusLED 13										//Pin mit Status LED

/************************************
 * Definition und Initialiierung
 * Globale Variablen, Klassen und Funktionen
 ************************************/
//Globale Programm Variablen
uint16_t pwmPulse = MOTOR_MIN_PULSE;
uint8_t motorStufe = 0;
bool serialIsSent = 0;

//Vorkompilierte Definitionen
#define MOTOR_PULSE_BREITE (MOTOR_MAX_PULSE - MOTOR_MIN_PULSE)
#define MOTOR_PULSE_STUFE (MOTOR_PULSE_BREITE / MOTOR_STUFEN)
#ifndef	SerialUSB
	#define SerialUSB SERIAL_PORT_MONITOR
#endif
//Funktionen


//Klassen
class TasterEntprellen {
		uint8_t pin;
		uint8_t modus;										//INPUT 0x0 | INPUT_PULLUP 0x2
		bool tasterStatusLang;
		bool letzterTasterStatus;
		uint8_t entprellenVerzoegerung = 50;
		uint32_t zeitLangerDruck;
		uint32_t letzteFlankenZeit;
    public:
		void init(uint8_t tasterPin, uint8_t pinModus, uint16_t zeitLang);	//Pin des Tasters und Modus: INPUT oder INPUT_PULLUP
		uint8_t leseTaster();								//Gibt den aktuellen Status des Tasters entprellt wieder
};

TasterEntprellen tasterHoch;								//definiere Klassen Instanz für Taster
TasterEntprellen tasterRunter;								//definiere Klassen Instanz für Taster
Servo motorRegler;											//definiere Klassen Instanz für Servo

void setup() {												// Setup Code, wird einmalig am Start ausgeführt
	/************************************
	* Setup Inputs 
	************************************/
	//definiere Tasterpin für Auswertung, mit PullUp Widerstand und Zeitvorgabe für langen Druck
	tasterHoch.init(inTasterHoch, INPUT_PULLUP, ZEIT_TASTER_LANG);	
	tasterRunter.init(inTasterRunter, INPUT_PULLUP, ZEIT_TASTER_LANG);
	/************************************
	* Setup Outputs 
	************************************/
	motorRegler.attach(outMotorRegler);
	motorRegler.writeMicroseconds(MOTOR_MIN_PULSE); 		//Setze Ausgang auf 0% PWM (0-100% -> 1000-2000µs)
	
	#if (DEBUGLEVEL >=1)									//Bedingte Kompilierung
		pinMode(statusLED, OUTPUT);							//status LED definieren zur Anzeige des Status
	#endif		
	#if (DEBUGLEVEL >=2)									//Bedingte Kompilierung
		SerialUSB.begin(9600);  							//Starte Serielle Kommunikation per USB
	#endif
}

void loop() {                       						// Hauptcode, wiederholt sich zyklisch     
	uint8_t dynTasterHoch = tasterHoch.leseTaster();		//lese aktuellen Status des Tasters
	uint8_t dynTasterRunter = tasterRunter.leseTaster();	//lese aktuellen Status des Tasters
	if(dynTasterHoch && dynTasterRunter){
		//Nichts tun
	} else if (dynTasterHoch == 1) {
		if(motorStufe < MOTOR_STUFEN) {
			pwmPulse += MOTOR_PULSE_STUFE;
			motorStufe++;
		}	
	} else if (dynTasterHoch == 2) {
		pwmPulse = MOTOR_MAX_PULSE;
		motorStufe = MOTOR_STUFEN;
	} else if (dynTasterRunter == 1) {
		if(motorStufe > 0) {
			pwmPulse -= MOTOR_PULSE_STUFE;
			motorStufe--;
		}	
	} else if (dynTasterRunter == 2) {
		pwmPulse = MOTOR_MIN_PULSE;
		motorStufe = 0;
	}
	motorRegler.writeMicroseconds(pwmPulse);
	#if (DEBUGLEVEL >= 1)
		//Wenn einer der Taster gedrückt wird
		if(!digitalRead(inTasterHoch) || !digitalRead(inTasterRunter)) {
			digitalWrite(statusLED, HIGH);					//Dann Setze Status LED
		} else {
			digitalWrite(statusLED, LOW);					//Ansonsten setzte Sie zurück
		}
	#endif
	#if (DEBUGLEVEL >= 2)
		//Wird jede Sekunde ausgeführt
		if((millis()%1000 >= 500) && (serialIsSent == false)) {
			SerialUSB.println("----PWM Pulse----");
			SerialUSB.println(pwmPulse);
			SerialUSB.println("------Stufe------");
			SerialUSB.println(motorStufe);
			SerialUSB.println("---Pulsebreite--");
			SerialUSB.println(MOTOR_PULSE_BREITE);
			SerialUSB.println("----Puls Stufen---");
			SerialUSB.println(MOTOR_PULSE_STUFE);
			SerialUSB.println("-------End-------");
			serialIsSent = true;
		} else if((millis()%1000 < 500) && (serialIsSent == true)) {
			serialIsSent = false;							//Stellt sicher, dass Code nur einmal je Sekunde ausgeführt wird.
		}
	#endif
}
//22 3,873 16 2,816
void TasterEntprellen::init(uint8_t tasterPin, uint8_t pinModus, uint16_t zeitLang){ //Pin des Tasters, Modus: INPUT oder INPUT_PULLUP, Dauer für lang
	zeitLangerDruck = zeitLang;							//Speichere Zeit in Instanz ab
	pin = tasterPin;									//Speichere Pin in Instanz ab
	modus = pinModus;									//Speichere Pinmodus in Instanz ab
	pinMode(pin, modus);								//definiere Pin 
}

/******************************************************
 * Gibt eine 1 aus, wenn der Taster kurz HIGH war.
 * Auf zu kurze Impulse wird nicht reagiert (entprellen)
 * Gibt 2 aus, wenn der Taster lang gedrückt wurde.
 ******************************************************/
uint8_t TasterEntprellen::leseTaster(){
	bool leseStatus;
	uint8_t returnVal = 0;
	//Invertiert einlesen, wenn Pullup Widerstand gesetzt ist.
	if(modus == INPUT_PULLUP) {						
		leseStatus = !digitalRead(pin);					//lese Taster Status (!)Invertiert
	} else {											//Ansonsten behandle eingang als Input ohne Pullup
		leseStatus = digitalRead(pin);					//lese Taster Status Normal
	} 
	//wenn sich der status des Tasters verändert hat
	if (leseStatus != letzterTasterStatus) {
		//wenn der Taster losgelassen wurde und außreichend lange gedrückt wurde
		if (((millis() - letzteFlankenZeit) > entprellenVerzoegerung) && (letzterTasterStatus) && (!tasterStatusLang)) {
			returnVal = 1;								//dann gib 1 zurück
		} else {
			returnVal = 0;								//ansonsten gib 0 zurück
			tasterStatusLang = false;					//setze taster Lang Auswertung zurück
		}
		letzteFlankenZeit = millis();					//Speichere letzte Zeit
		letzterTasterStatus = leseStatus;				//Speichere letzten Status
	}
	//Wenn der Taster lange genug gedrückt wird
	if(((millis() - letzteFlankenZeit) > zeitLangerDruck) && (letzterTasterStatus)) {
		returnVal = 2;									//dann gib 2 zurück
		tasterStatusLang = true;						//Setze Status um zu vermeiden, dass Vergeleich zum entprellen Wahr wird.
	}
	return returnVal;									//gibt Wert zurück an voherige Instanz
}