/*
 * BrushlessMotorcontroller v0.1.7
 * Date: 20.06.2020 | 17:41
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
 * [In Deutsch]: <https://www.gnu.de/documents/gpl-3.0.de.html>
 */

/************************************
 * Programm Konfiguration
 ************************************/
#define PROG_VERSION 1                   					//Version
#define MOTOR_MIN_PULSE	1000								//Minimale Position des PWM Servo signal in millisekunden | Default: 1000
#define MOTOR_MAX_PULSE	2000								//Maximale Position des PWM Servo signal in millisekunden | Default: 2000
#define MOTOR_STUFEN 10										//Stufenanzahl in welchen der Motor geschaltet werden kann.
#define ZEIT_TASTER_LANG 2000								//Zeit für einen Langen Tasterdruck

#define WIDERSTAND_R1 30000.0								//Widerstand gegen V+ für Spannungsmessung in Ohm (Wichtig Kommastelle(bzw. Punkt:'.') muss vorhanden sein)
#define WIDERSTAND_R2 7500.0								//Widerstand gegen GND für Spannungsmessung in Ohm (Wichtig Kommastelle(bzw. Punkt:'.') muss vorhanden sein)
#define SPANNUNG_KORREKTUR 0									//Korrektur der Spannung in mV
	
//Ändere diesen Wert für unterschiedliche Debuging level
#define DEBUGLEVEL 3										//0 = Aus | 1 = N/A | >2 = Serial Monitor
#define DISPLAY_AKTIV 1
/************************************
 * Zusätzliche Dateien einbinden
 ************************************/
#include <Servo.h>											//Bibliothek für Servo PWM Ausgabe
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
/************************************
 * Definition IO Pins
 ************************************/
/* Pinout Arduino Nano:
 * Serial 0+1 (kein HW Serial) | Interrupt 2+3 | PWM 3, 5, 6, 9, 10, 11 | LED 13 | I2C A4(SDA) + A5(SCL) |
 * Servo Lib deaktiviert PWM funktionalität für pin 9 und 10
 */
 
//Inputs
//Pin 0+1 Reserviert halten für Serielle Kommunikation über USB!
#define inTachoImpuls 3										//Pin für den Impuls der Drehzahl | Muss ein Interrupt Pin sein
#define inTasterHoch 4            							//Pin des Taster um die Drehzahl zu erhöhen
#define inTasterRunter 5            						//Pin des Taster um die Drehzahl zu verringern
#define inAkkuSpannung A0									//Pin zum Messen der Spannung

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
bool displaySenden = 0;
uint32_t spannungUmgerechnet = 0;
uint32_t spannungVoltDEC0 = 0;
uint32_t spannungVoltDEC1 = 0;
bool blinkPulse;


uint16_t drehzahl = 0;
volatile uint16_t volleUmdrehungen = 0;
uint32_t letzteZeit = 0;

#if (DISPLAY_AKTIV ==1)
uint8_t displayAdress = 0;
#endif
//Vorkompilierte Definitionen
#define MOTOR_PULSE_BREITE (MOTOR_MAX_PULSE - MOTOR_MIN_PULSE)
#define MOTOR_PULSE_STUFE (MOTOR_PULSE_BREITE / MOTOR_STUFEN)
#ifndef	SerialUSB
	#define SerialUSB SERIAL_PORT_MONITOR
#endif
//Funktion zum Berechnen der Spannung (Vout Ausgang Spannungsteiler/Eingang Arduino | Vin Eingang Spannungteiler/Akku)
//Vout = Vin * (R2 / (R1 + R2))
//Vin = Vout / (R2 / (R1 + R2))
#define SPANNUNG_OUT_MAX 5000
#define SPANNUNG_OUT_MIN 0
#define SPANNUNG_IN_MAX SPANNUNG_OUT_MAX / (WIDERSTAND_R2 / (WIDERSTAND_R2 + WIDERSTAND_R1))
#define SPANNUNG_IN_MIN 0
#define SPANNUNG_IN_MAX_CALC (SPANNUNG_IN_MAX - SPANNUNG_KORREKTUR)
#define SPANNUNG_IN_BEREICH (SPANNUNG_IN_MAX_CALC - SPANNUNG_IN_MIN)

//Funktionen
void interruptRPM();
#if (DISPLAY_AKTIV ==1)
void displayStart();
void displayAnzeigen();
uint8_t i2cScan();
#endif

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
#if (DISPLAY_AKTIV ==1)
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#endif
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {												// Setup Code, wird einmalig am Start ausgeführt
	/************************************
	* Setup Inputs 
	************************************/
	//definiere Tasterpin für Auswertung, mit PullUp Widerstand und Zeitvorgabe für langen Druck
	tasterHoch.init(inTasterHoch, INPUT_PULLUP, ZEIT_TASTER_LANG);	
	tasterRunter.init(inTasterRunter, INPUT_PULLUP, ZEIT_TASTER_LANG);
	pinMode(inAkkuSpannung, INPUT);
	pinMode(inTachoImpuls, INPUT);
	/************************************
	* Setup Outputs 
	************************************/
	motorRegler.attach(outMotorRegler);
		//Wenn beim start beide Taster gedrückt sind, dann führe Regler initialisierung durch.
	if(!digitalRead(inTasterHoch) || !digitalRead(inTasterRunter)) {
		while(!digitalRead(inTasterHoch) || !digitalRead(inTasterRunter)) {	
			motorRegler.writeMicroseconds(MOTOR_MAX_PULSE); 	//Setze Ausgang auf 100% PWM (0-100% -> 1000-2000µs)
		}
	}	
	motorRegler.writeMicroseconds(MOTOR_MIN_PULSE); 		//Setze Ausgang auf 0% PWM (0-100% -> 1000-2000µs)
	
	attachInterrupt(digitalPinToInterrupt(inTachoImpuls), interruptRPM, RISING); //Setup Interrupt bei steigender Flanke
	#if (DEBUGLEVEL >=1)									//Bedingte Kompilierung
		pinMode(statusLED, OUTPUT);							//status LED definieren zur Anzeige des Status
	#endif		
	#if (DEBUGLEVEL >=2)									//Bedingte Kompilierung
		SerialUSB.begin(9600);  							//Starte Serielle Kommunikation per USB
	#endif
	#if (DISPLAY_AKTIV ==1)
	displayStart();
	#endif
	delay(2000);
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
	
	uint32_t leseWert = analogRead(inAkkuSpannung);
	spannungUmgerechnet = map(leseWert, 0, 1023, SPANNUNG_IN_MIN, SPANNUNG_IN_MAX_CALC);
	spannungVoltDEC0 = spannungUmgerechnet/1000;
	uint32_t spannungVoltDEC = spannungVoltDEC0*10;
	spannungVoltDEC1 = spannungUmgerechnet/100;
	spannungVoltDEC1 = spannungVoltDEC1-spannungVoltDEC;
	
	//Wenn letzte Zeit größer als jetztige, dann gab es einen overflow (nach 50 Tagen), dann setzt letzte Zeit zurück
	if(letzteZeit > millis()) letzteZeit = 0;
	//Berechnung der Drehzahl 2 * (halbe Sekunde / Zeitdauer * Anzahl Impulse)
	if (volleUmdrehungen >= 10) {
		drehzahl = 30*1000/(millis() - letzteZeit)*volleUmdrehungen;
		drehzahl = drehzahl *2;
		letzteZeit = millis();
		volleUmdrehungen = 0;
	}
	
	//wenn kein impuls seit länger als einer Minute kam, dann setzte drehzahl zurück.
	if((millis() - letzteZeit) > 60000) drehzahl = 0;
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
			SerialUSB.println(F("--PWM Pulsedauer--"));
			SerialUSB.print(pwmPulse);
			SerialUSB.println(F("µs"));
			SerialUSB.println(F("------Stufe-------"));
			SerialUSB.println(motorStufe);
			SerialUSB.println(F("----Analogwert----"));
			SerialUSB.println(leseWert);
			SerialUSB.println(F("-----Spannung-----"));
			SerialUSB.print(spannungUmgerechnet);
			SerialUSB.println(F(" mV"));
			SerialUSB.println(F("-----Drehzahl-----"));
			SerialUSB.print(drehzahl);
			SerialUSB.println(F(" U/min"));
			SerialUSB.println(F("-------End--------"));
			serialIsSent = true;
		} else if((millis()%1000 < 500) && (serialIsSent == true)) {
			serialIsSent = false;							//Stellt sicher, dass Code nur einmal je Sekunde ausgeführt wird.
		}
	#endif
	#if (DISPLAY_AKTIV ==1)
	displayAnzeigen();
	#endif
}

void interruptRPM() { //Wird bei jedem Impuls ausgeführt.
	volleUmdrehungen++; 
}

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

#if (DISPLAY_AKTIV ==1)
void displayStart() {
	
	Wire.begin();
	displayAdress = EEPROM.read(0);							//lese Display Adresse von EEPROM Adresse 0
	#if (DEBUGLEVEL >=3)									//Bedingte Kompilierung
		SerialUSB.print(F("EEPROM Wert: 0x"));
		if (displayAdress<16) {
				SerialUSB.print(F("0"));
			}
		SerialUSB.println(displayAdress,HEX);
	#endif
	
	uint8_t error;
	if(displayAdress >= 127) {
		displayAdress = i2cScan();
	} else {
		Wire.beginTransmission(displayAdress);
		error = Wire.endTransmission();
		if(error != 0) {
			#if (DEBUGLEVEL >=3)									//Bedingte Kompilierung
				SerialUSB.println(F("Adresse aus EMPROM nicht ansprechbar"));
			#endif
			displayAdress = i2cScan();
			EEPROM.update(0, displayAdress);
		} else {
			#if (DEBUGLEVEL >=3)									//Bedingte Kompilierung
				SerialUSB.println(F("Adresse aus EEPROM ansprechbar!"));
			#endif
		}
	}
	
	if(!display.begin(SSD1306_SWITCHCAPVCC, displayAdress, true, false)) { // Address 0x3D for 128x64
		#if (DEBUGLEVEL >=3)
		Serial.println(F("SSD1306 Zuweisung gescheitert"));
		#endif
		for(;;); // Don't proceed, loop forever
	}

	// Show initial display buffer contents on the screen --
	// the library initializes this with an Adafruit splash screen.
	display.display();
}
void displayAnzeigen() {
	if((millis()%500 >= 250) && (displaySenden == false)) {
		blinkPulse = !blinkPulse;
		display.clearDisplay();
		display.setCursor(0,0);             // Start at top-left corner
		display.setTextSize(2);             // Normal 1:1 pixel scale
		display.setTextColor(SSD1306_WHITE);        // Draw white text
		if(spannungVoltDEC0 < 10) {
			display.print(F(" "));
		}
		display.print(spannungVoltDEC0);
		display.print(F("."));
		display.print(spannungVoltDEC1);
		display.println(F("V"));		
        display.fillRect(65, 4, 4, 6, SSD1306_WHITE); 		//batterie Pluspol | x,y,width,height,color    
		display.drawRect(69, 0, 58, 15, SSD1306_WHITE); 	//batterie Rahmen | x,y,width,height,color
		
		if(spannungUmgerechnet >= 20000) {
			display.fillRect(71, 3, 12, 9, SSD1306_WHITE); 	//Batterie 4/4 voll | x,y,width,height,color
		} else {
			
		}
		if(spannungUmgerechnet >= 19000) {
			display.fillRect(85, 3, 12, 9, SSD1306_WHITE); 	//Batterie 3/4 voll | x,y,width,height,color
		} else {
			display.drawLine(84, 3, 84, 11, SSD1306_WHITE); //Batterie Trennstrich | x,y,width,height,color
		}
		if(spannungUmgerechnet >= 18000) {
			display.fillRect(99, 3, 12, 9, SSD1306_WHITE); 	//Batterie 4/4 voll | x,y,width,height,color
		} else {
			display.drawLine(98, 3, 98, 11, SSD1306_WHITE); //Batterie Trennstrich | x,y,width,height,color
		}
		if(spannungUmgerechnet >= 17000) {
			display.fillRect(113, 3, 12, 9, SSD1306_WHITE); 	//Batterie 1/4 voll | x,y,width,height,color
		} else {
			if(blinkPulse) {
				display.fillRect(113, 3, 12, 9, SSD1306_WHITE); 	//Batterie 1/4 voll | x,y,width,height,color	
			} else {
				display.drawLine(112, 3, 112, 11, SSD1306_WHITE); //Batterie Trennstrich | x,y,width,height,color
			}
		}
		uint8_t balkenAnzahl;
		uint8_t aktuellerBalken;
		if(MOTOR_STUFEN <= 16) {
			balkenAnzahl = MOTOR_STUFEN;
			aktuellerBalken = motorStufe;
		} else {
			balkenAnzahl = 16;
			aktuellerBalken = map(motorStufe, 0, MOTOR_STUFEN, 0, 16);
			
		}
		uint8_t balkenBreite = 80/balkenAnzahl;
		uint8_t balkenStartHoehe = 5;		
		uint8_t balkenHoeheStufe = (32-balkenStartHoehe)/(balkenAnzahl-1);		
		uint8_t balkenHoehe = balkenStartHoehe;
		uint8_t aktuelleStufe = 1;
		//display.drawRect(48, 16, 80, 32, SSD1306_WHITE); 	//batterie Rahmen | x,y,width,height,color
		for(uint8_t i = 0; i < balkenAnzahl*balkenBreite; i+=balkenBreite){
			if(aktuellerBalken >= aktuelleStufe) {
				display.fillRect(49+i, 48-balkenHoehe, balkenBreite-2, balkenHoehe, SSD1306_WHITE); 	//batterie Rahmen | x,y,width,height,color
			} else {
				display.drawRect(49+i, 48-balkenHoehe, balkenBreite-2, balkenHoehe, SSD1306_WHITE); 	//batterie Rahmen | x,y,width,height,color
			}
			balkenHoehe += balkenHoeheStufe;
			aktuelleStufe++;
		}
		//MOTOR_STUFEN
		display.setCursor(8,20); // Start at top-left corner
		//display.print(F("Pulsedauer: "));
		//display.print(pwmPulse);
		//display.println(F("us"));
		display.setTextSize(1);  
		display.println(F("STUFE"));
		display.setCursor(16,30); // Start at top-left corner
		display.setTextSize(2);
		//display.print(F(" ")); 
		display.println(motorStufe);
		display.setCursor(0,50); // Start at top-left corner
		if(drehzahl < 10) {
			display.print(F("    "));
		} else  if(drehzahl < 100) {
			display.print(F("   "));
		} else  if(drehzahl < 1000) {
			display.print(F("  "));
		} else  if(drehzahl < 10000) {
			display.print(F(" "));
		}
		display.print(drehzahl);
		display.setCursor(64,50); // Start at top-left corner
		display.println(F("U/min"));
		display.display();
		displaySenden = true;
	} else if((millis()%500 < 250) && (displaySenden == true)) {
		displaySenden = false;							//Stellt sicher, dass Code nur einmal je Sekunde ausgeführt wird.
	}
	
}

uint8_t i2cScan() {
	uint8_t error, address, outValue;
	uint8_t nDevices = 0;
	SerialUSB.println(F("--Scane I2C Bus--"));
	for(address = 1; address <=127 ; address++ ) {
	// der Scan verwendet den Rückgabewert von Write.endTransmisstion 
	// um zu sehen ob ein Gerät angewortet hat
		Wire.beginTransmission(address);
		error = Wire.endTransmission();
		#if (DEBUGLEVEL >= 3)
			SerialUSB.print(F("Prüfe I2C Adresse: 0x"));
			if (address<16) {
				SerialUSB.print(F("0"));
			} 
			SerialUSB.println(address, HEX);
		#endif
		if (error == 0) {
			nDevices++;
			outValue = address;
			address = 127;
		}
	}
	if (nDevices == 0) {
		outValue = 0;
		#if (DEBUGLEVEL >= 3)
			SerialUSB.println(F("--Kein Gerät gefunden--"));
		#endif
	} else {
		#if (DEBUGLEVEL >= 3)
			SerialUSB.print(F("Gerät gefunden auf Adresse: 0x"));
			if (address<16) {
				SerialUSB.print(F("0"));
			} 
			SerialUSB.println(address, HEX);
		#endif
	}
		
	return outValue;
}
#endif

