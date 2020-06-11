/*
 * BrushlessMotorcontroller v0.0.2
 * Date: 11.06.2020 | 02:48
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
 */

/************************************
 * Programm Konfiguration
 ************************************/
#define PROG_VERSION 1                   			//Version
#define MOTOR_MIN_PULSE	1000						//Minimale Position des PWM Servo signal in millisekunden | Default: 1000
#define MOTOR_MAX_PULSE	2000						//Maximale Position des PWM Servo signal in millisekunden | Default: 2000
#define MOTOR_STUFEN 10								//Stufenanzahl in welchen der Motor geschaltet werden kann.
#define ZEIT_TASTER_LANG 3000						//Zeit für einen Langen Tasterdruck

//Ändere diesen Wert für unterschiedliche Debuging level
#define DEBUGLEVEL 3								//0 = Aus | 1 = N/A | >2 = Serial Monitor

/************************************
 * Zusätzliche Dateien einbinden
 ************************************/
#include <Servo.h>									//Bibliothek für Servo PWM Ausgabe

/************************************
 * Definition IO Pins
 ************************************/
//Pinout Arduino Nano:
//Serial 0+1 (kein HW Serial) | Interrupt 2+3 | PWM 3, 5, 6, 9, 10, 11 | LED 13 | I2C A4(SDA) + A5(SCL) |
//Servo Lib deaktiviert PWM funktionalität für pin 9 und 10

//Inputs
//Pin 0+1 Reservieren für Serielle Kommunikation über USB!
#define inTasterHoch 3            					//Pin des Taster um die Drehzahl zu erhöhen
#define inTasterRunter 4            				//Pin des Taster um die Drehzahl zu verringern


//Outputs
#define outMotorRegler 9                    		//Pin für Servo PWM Signal für Motor Regler

/************************************
 * Definition und Initialiierung
 * Globale Variablen, Klassen und Funktionen
 ************************************/
//Globale Programm Variablen
uint16_t pwmPulse = MOTOR_MIN_PULSE;
uint8_t motorStufe = 0;

//Vorkompilierte Definitionen
#define MOTOR_PULSE_BREITE MOTOR_MAX_PULSE - MOTOR_MIN_PULSE
#define MOTOR_PULSE_STUFE MOTOR_PULSE_BREITE / MOTOR_STUFEN

//Funktionen


//Klassen
class TasterEntprellen {
		uint8_t pin;
		uint8_t modus;								//INPUT 0x0 | INPUT_PULLUP 0x2
		bool tasterStatusLang;
		bool letzterTasterStatus;
		uint8_t entprellenVerzoegerung = 50;
		uint8_t zeitLangerDruck;
		uint32_t letzteFlankenZeit;
    public:
		void init(uint8_t tasterPin, uint8_t pinModus, uint16_t zeitLang);	//Pin des Tasters und Modus: INPUT oder INPUT_PULLUP
		uint8_t leseTaster();								//Gibt den aktuellen Status des Tasters entprellt wieder
};

TasterEntprellen tasterHoch;
TasterEntprellen tasterRunter;
Servo motorRegler;

void setup() {										// Setup Code, wird einmalig am Start ausgeführt
	/************************************
	* Setup Inputs 
	************************************/
	tasterHoch.init(inTasterHoch, INPUT_PULLUP, ZEIT_TASTER_LANG);
	tasterRunter.init(inTasterHoch, INPUT_PULLUP, ZEIT_TASTER_LANG);
	//TODO: Pins definieren
	/************************************
	* Setup Outputs 
	************************************/
	motorRegler.attach(outMotorRegler);
	motorRegler.writeMicroseconds(MOTOR_MIN_PULSE); //Setze Ausgang auf 0% PWM (0-100% -> 1000-2000µs)
			
	#if (DEBUGLEVEL >=2)							//Bedingte Kompilierung
		SerialUSB.begin(9600);  					//Starte Serielle Kommunikation per USB
	#endif
}

void loop() {                       				// Hauptcode, wiederholt sich zyklisch     
	uint8_t dynTasterHoch = tasterHoch.leseTaster();
	uint8_t dynTasterRunter = tasterRunter.leseTaster();
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
}
// #define 
// #define 
// #define 


void TasterEntprellen::init(uint8_t tasterPin, uint8_t pinModus, uint16_t zeitLang){ //Pin des Tasters, Modus: INPUT oder INPUT_PULLUP, Dauer für lang
	zeitLangerDruck = zeitLang;
	pin = tasterPin;
	modus = pinModus;
	pinMode(pin, modus);
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
		leseStatus = !digitalRead(pin);	//lese Taster Status (!)Invertiert
	} else {										//Ansonsten behandle eingang als Input ohne Pullup
		leseStatus = digitalRead(pin);	//lese Taster Status Normal
	} 

	if (leseStatus != letzterTasterStatus) {
		if (((millis() - letzteFlankenZeit) > entprellenVerzoegerung) && (letzterTasterStatus) && (!tasterStatusLang)) {
			returnVal = 1;
		} else {
			returnVal = 0;
			tasterStatusLang = false;
		}
		letzteFlankenZeit = millis();
		letzterTasterStatus = leseStatus;				//Speichere letzten Status
	}
	//warte kurz, bis das Signal lange genug ansteht.
	if(((millis() - letzteFlankenZeit) > zeitLangerDruck) && (letzterTasterStatus)) {
		returnVal = 2;
		tasterStatusLang = true;
	}
	return returnVal;
}