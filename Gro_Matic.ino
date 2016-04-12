/*

  FILE: gro-matic
  AUTHOR: zrox, Galaktika, Driverone
  VERSION: 0.9.9.8

  2004 I2C LCD
  DS3231 I2C RTC
  BME-280 I2C Thermometer, Hydrometer
  GY-30 I2C Lux Meter
  Chirp I2C I2CSoilMoistureSensor

  Bei allen I2C Sensoren gilt, in Reihe schalten nicht sternförmig anschließen, und zwischen SDA +5V und SCL +5V
  je einen 4,7K Ohm Widerstand einlöten.
  Reihenfolge: Arduino > Wirderstände > DS3231 > 2004 LCD > BME-280 > GY-30 > Chirp
  (bei UNO, SDA = Pin A4, SCL = Pin A5)

  Taster mit INTERNAL_PULLUP an Pin10 zum ein/ausschalten des Displays.
  Taster mit INTERNAL_PULLUP Pin4 zum wechseln der screens

  Encoder KY-040
  Die Anschlüsse des KY-040 sind wie folgt belegt:

  CLK, an Pin3
  DT, an Pin2
  SW, mit INTERNAL_PULLUP an PinA1 zum wechseln des Lichtmodus, reset des Eproms, usw...
  +, an 5V
  GND, an GND des Arduino

  8er Relais Modul
  Relais 1, an Pin 9 als Wechsler für Stufentrafo
  Relais 2, an Pin 7 zur NDL (P braunes Kabel) Steuerung
  Relais 3, an Pin 6 zur LSR (P braunes Kabel) Steuerung
  Relais 4, an Pin 5 zur Umluftventi steuerung
  Relais 5, an Pin 11 zur Wasserpumpe


  Relais 1 Anschluss

  Pin rechts z.B. an 80V anschluss vom Stufentrafo, Pin in der Mitte P (braunes Kabel) an LTI und Pin links an z.B. 190V vom Stufentrafo.
  Anschüsse zum Stufentrafo  190V  P  80V
  |  |  |
  ---------
  |o  o  o|
  | Relais|
  ---------

  Anschluss Relais 2 -5
  Stromzufuhr (von der Steckdose) immer in der Mitte und anschluss zur NDL oder LSR rechts.

  LCD-Display LCD20x4
  Sonderzeichen:  Eingabe z.B. lcd.print(char(0xE1)); für ä
              ä = 0xE1
              ö = 0xEF
              ü = 0xF5
              ß = 0xE2
              ° = 0xDF
              µ = 0xE4
*/

/***************/
/* DEFINEMENTS */
/***************/
// Bekanntmachung der Relais
#define luft_relay    9   // luft_relay = LTI
#define licht_relay_p 7   // licht_relay = zur Steuerung des Hauptleuchtmittels
#define lsr_relay_p   6   // lsr_relay = zur Steuerung der LSR der Jungpflanzen
#define ventilator    5   // vetilator = zur steuerung des Relais Umluftventilators
#define irrigation    11  // wasser_relay = autobewaesserung

#define BACKLIGHT_PIN (3)

#define LED_ADDR (0x27)  // might need to be 0x3F, if 0x27 doesn't work

// GY-30 Lux Meter
#define BH1750_address 0x23  // I2C Addresse des GY-30

#define entprellZeit 500  // Zeit für Entprellung, anpassen!

//Backlight button
#define BUTTON_PIN 10

//Programm modus und reset Taster
#define wechslertPin A1  // Pinnummer des Tasters für zum Lichtmodus wechseln und Eprom Reset
#define screenPin 4  // Pin für Taster zum umschalten der LCD seite
#define encoderPinA 2
#define encoderPinB 3

/************/
/* INCLUDES */
/************/
#include "Wire.h"                   // https://www.arduino.cc/en/Reference/Wire (included with Arduino IDE)
#include "LiquidCrystal_I2C.h"      // https://bitbucket.org/fmalpartida/new-liquidcrystal
#include "DS3232RTC.h"              // https://github.com/JChristensen/DS3232RTC.git
#include "EEPROM.h"                 // https://www.arduino.cc/en/Reference/EEPROM
#include "I2CSoilMoistureSensor.h"  // https://github.com/Miceuz/i2c-moisture-sensor
#include "Adafruit_Sensor.h"        // https://github.com/adafruit/Adafruit_Sensor
#include "Adafruit_BME280.h"        // https://github.com/adafruit/Adafruit_BME280_Library
#include "Time.h"                   // https://github.com/PaulStoffregen/Time
#include "TimeAlarms.h"             // https://www.pjrc.com/teensy/td_libs_TimeAlarms.html
//#include "SdFat.h"                // https://github.com/greiman/SdFat
//#include <stdarg.h>               // http://playground.arduino.cc/Main/Printf
#include "Bounce2.h"                // https://github.com/thomasfredericks/Bounce2

/********************/
/* GLOBAL VARIABLES */
/********************/
/* Festlegen der verschiedenen Lichtprogramme */
enum { LSR, GROW, BLOOM };

/* Wenn MAGIC_NUMBER im EEPROM nicht übereinstimmt wird das EEPROM mit den default einstellungen neu geschreiben */
const uint32_t MAGIC_NUMBER = 0xAAEBCCDF;

/* Structure hält default einstellungen! diese werden von EEPROM überschrieben oder werden geschrieben falls noch nicht gesetzt */
struct setings_t {

  uint32_t MAGIC_NUMBER   = MAGIC_NUMBER;

  byte lichtmodus     = LSR;   // Speichern des gewählten Lichtmodus einstellen (enumeration)
  bool autowasse      = false; // Autobewasserung, on false = disabled

  byte bloom_counter  = 0;    // Speichern des bloom Tage counters.
  byte starttag       = 0;    // Speichern des Start Tages
  byte startmonat     = 0;    // Speichern des Start Monats

/* Ab hier Zeit für die Belichtungsmodis einstellen */
  byte lsr_an           = 5;     // Startzeit des LSR Modis
  byte lsr_aus          = 23;    // Endzeit des LSR Modis
  byte grow_licht_an    = 5;     // Startzeit des Grow Modis
  byte grow_licht_aus   = 23;    // Endzeit des Grow Modis
  byte bloom_licht_an   = 7;     // Startzeit des Bloom Modis
  byte bloom_licht_aus  = 19;    // Endzeit des Grow Modis
 
/* Temperaturwerte für LTI, ab erreichen der Temperaturen in den verschiedenen Licht Modis soll LTI in die hoechste Stufe geschaltet werden. */
  double lsr_temp   = 24.00; // Temp im LSR Modi
  double grow_temp  = 23.00; // Temp im Grow Modi
  double bloom_temp = 22.00; // Temp im Bloom Modi

/* RLF Werte für LTI, z.B. bei 40.00% RLF soll LTI in die hoechste Stufe geschaltet werden. */
  double lsr_rlf    = 60.00;  // RLF im LSR Modi
  double grow_rlf   = 55.00;  // RLF im Grow Modi
  double bloom_rlf  = 40.00;  // RLF im Bloom Modi

/* Autobewaesserung */
  byte autowasser     = 0;
  byte startwasser    = 7;
  byte auswasser      = 5;
  byte startwassermin = 0;
  byte sekauswasser   = 0;

} setings;

byte write_EEPROM = true; // false = 0;
bool save_EEPROM  = false;

// Encoder
volatile unsigned int encoderPos = 0;  // Encoder counter
volatile bool A_set     = false;
volatile bool B_set     = false;

byte temp_bereich = 0;
byte rlf_bereich  = 0;

// Custom Caracter
enum { MOON, SUN, THERMO, RLF, WATER_ON, WATER_OFF, VENTI_I, VENTI_II };
byte Moon[8]      = { 0b00000, 0b01110, 0b10101, 0b11111, 0b10001, 0b01110, 0b00000, 0b00000 };
byte Sun[8]       = { 0b00000, 0b00100, 0b10101, 0b01110, 0b01110, 0b10101, 0b00100, 0b00000 };
byte Thermo[8]    = { 0b00100, 0b01010, 0b01010, 0b01110, 0b01110, 0b11111, 0b11111, 0b01110 };
byte Rlf[8]       = { 0b00100, 0b00100, 0b01110, 0b01110, 0b11111, 0b11001, 0b11111, 0b01110 };
byte Water_on[8]  = { 0b11100, 0b01000, 0b11110, 0b11111, 0b00011, 0b00011, 0b00000, 0b00011 };
byte Water_off[8] = { 0b11100, 0b01000, 0b11100, 0b11110, 0b00011, 0b00011, 0b00000, 0b00000 };
byte Venti_I[8]   = { 0b00100, 0b01010, 0b00000, 0b00100, 0b10001, 0b11011, 0b00000, 0b00000 };
byte Venti_II[8]  = { 0b00000, 0b11011, 0b10001, 0b00100, 0b00000, 0b01010, 0b00100, 0b00000 };

/***********/
/* OBJECTS */
/***********/
LiquidCrystal_I2C lcd(LED_ADDR, 2, 1, 0, 4, 5, 6, 7, BACKLIGHT_PIN, POSITIVE);
Adafruit_BME280 bme; // I2C BME-280
I2CSoilMoistureSensor bodensensor; // setze Var fuer Bodenfeuchtesensor (chirp)

//ArduinoOutStream cout(lcd);

Bounce debounce   = Bounce();
Bounce debounce2  = Bounce();
Bounce debounce3  = Bounce();

/*************/
/* FUNCTIONS */
/*************/
// GY-30 Luxmeter
void BH1750_Init(int address){

  Wire.beginTransmission(address);
  Wire.write(0x10);  // 1 [lux] aufloesung
  Wire.endTransmission();

}

byte BH1750_Read(int address, byte *buff){

  byte i = 0;

  Wire.beginTransmission(address);
  Wire.requestFrom(address, 2);
  
  while(Wire.available()){
    
    buff[i] = Wire.read();
    i++;

  }

  Wire.endTransmission();

  return i;

}

/*
#define PRINTF_BUFFER 20 
void p(char *fmt, ...){
  
  char buf[PRINTF_BUFFER]; // resulting string limited to 128 chars
  
  va_list args;
  va_start (args, fmt );
  
  vsnprintf(buf, PRINTF_BUFFER, fmt, args);
  
  va_end (args);

  lcd.print(buf);
  Serial.print(buf);

}

void p(const __FlashStringHelper *fmt, ...){
  
  char buf[PRINTF_BUFFER]; // resulting string limited to 128 chars
  va_list args;
  va_start (args, fmt);
  
#ifdef __AVR__
  vsnprintf_P(buf, sizeof(PRINTF_BUFFER), (const char *)fmt, args); // progmem for AVR
#else
  vsnprintf(buf, sizeof(PRINTF_BUFFER), (const char *)fmt, args); // for the rest of the world
#endif

  va_end(args);
  
  lcd.print(buf);
  Serial.print(buf);
  
}
*/

void displayTime(){ // anzeige der Zeit und Datum auf dem Display
  
  lcd.setCursor(0, 0);
    
  if(hour() < 10)
    lcd.print("0");
 
  lcd.print(hour(), DEC);

  lcd.print(":");
    
  if(minute() < 10)
    lcd.print("0");
      
  lcd.print(minute(), DEC);
    
  lcd.print(":");
    
  if(second() < 10)
    lcd.print("0");

  lcd.print(second(), DEC);
  lcd.print(" ");

  const char *c_dayOfWeek[] = {"So", "Mo", "Di", "Mi", "Do", "Fr", "Sa"};
  lcd.print(c_dayOfWeek[weekday(now()) -1]);
 
  lcd.print(" ");
    
  if(day() < 10)
    lcd.print("0");

  lcd.print(day(), DEC);
  lcd.print(" ");
 
  const char *c_Month[] = {"Jan", "Feb", "Mar", "Apr", "Mai", "Jun", "Jul", "Aug", "Sep", "Okt", "Nov", "Dec"};
  lcd.print(c_Month[month() -1]);

}

double temp(){

  static unsigned long m;
  static double t;

  if(millis() - m > 1000){
    
    t = bme.readTemperature();
    m = millis();
    
  }

  return t;

}

double hum(){

  static unsigned long m;
  static double h;

  if(millis() - m > 1000){

    h = bme.readHumidity();
    m = millis();
    
  }

  return h;

}

void bme280(){ // Anzeige der Temp und RLF auf dem Display
    
  static unsigned long m;
  
  if(millis() - m > 3000){
    
    m = millis();

    // DISPLAY DATA
    lcd.setCursor(0, 1);  // setze curserposition
    lcd.write(THERMO);    // zeichne thermometer auf dem Display, siehe auch abschnitt Custom Caracter bzw. void setup
    lcd.print(F(" "));
    lcd.print((int) temp());
    lcd.print((char)223);
    lcd.print(F("C "));
    lcd.print(F(" "));
    lcd.write(RLF);    // zeichne Wassertropfen auf dem Display, siehe auch abschnitt Custom Caracter bzw. void setup
    lcd.print(F(" "));
    lcd.print((int) hum());
    lcd.print(F("%"));
    lcd.print(F("   "));
      
  }

}

void DS3231temp(){  // hole und zeige auf dem Display die Case Temperatur der RTC

  lcd.setCursor(0, 3);
  lcd.print(F("Case:"));
  lcd.print(RTC.temperature() / 4);
  lcd.print((char) 223);
  lcd.print(F("C"));

}

void LTI(){ // die Funtion des Rohrventilators 

  // Pruefe im LSR oder Grow Modus Temperatur und RLF ist die Temp unter 24 Grad C oder unter RLF unter 55%
  // bleibt Stufentrafo gedimmt (z.B. 80V)
  // ist Temp oder gleich oder höher wird auf hoechste stufe (z.B. 190V) geschaltet.
  
  if(setings.lichtmodus == LSR){ 
    
    if(temp() < setings.lsr_temp)
      digitalWrite(luft_relay, LOW);

    if(hum() < setings.lsr_rlf){
      
      digitalWrite(luft_relay, LOW);
    
    } else {
    
      digitalWrite(luft_relay, HIGH);
    
    }
  
  }
  
  // Pruefe im LSR oder Grow Modus Temperatur und RLF ist die Temp unter 24 Grad C oder unter RLF unter 55%
  // bleibt Stufentrafo gedimmt (z.B. 80V)
  // ist Temp oder gleich oder höher wird auf hoechste stufe (z.B. 190V) geschaltet.
  if(setings.lichtmodus == GROW){ 
    
    if(temp() < setings.grow_temp)
      digitalWrite(luft_relay, LOW);
    
    if(hum() < setings.grow_rlf){
      
      digitalWrite(luft_relay, LOW);
      
    } else {
      
      digitalWrite(luft_relay, HIGH);
    
    }
  
  }

  // Pruefe im Uebergangsmodus Grow>Bloom Temperatur und RLF
  if(setings.lichtmodus == BLOOM){
    
    if(temp() < setings.grow_temp)
      digitalWrite(luft_relay, LOW);
    
    if(hum() < setings.grow_rlf){
      
      digitalWrite(luft_relay, LOW);
      
    } else {
      
      digitalWrite(luft_relay, HIGH);
    
    }
  
  }

  // Pruefe im Bloom Modus Temperatur und RLF ist die Temp unter 22 Grad C oder unter RLF unter 40%
  // bleibt Stufentrafo gedimmt (z.B. 80V)
  // ist Temp oder RLF gleich oder höher wird auf hoechste stufe (z.B. 190V) geschaltet.
  if(setings.lichtmodus == BLOOM){
    
    if(temp() < setings.bloom_temp)
      digitalWrite(luft_relay, LOW);
 
    if(hum() < setings.bloom_rlf){
      
      digitalWrite(luft_relay, LOW);
      
    } else {
      
      digitalWrite(luft_relay, HIGH);
      
    }
    
  }
  
}

void gy30(){ // Luxmeter

  byte buff[2];
  float valf = 0;

  if(BH1750_Read(BH1750_address, buff) == 2){

    valf = ((buff[0] << 8) | buff[1]);

    lcd.setCursor(0, 2);
    lcd.print(F("LUX: "));

    if(valf < 1000)
      lcd.print(" ");

    if(valf < 100)
      lcd.print(" ");

    if(valf < 10)
      lcd.print(" ");

    lcd.print(valf, 2);

  }

}

#define DISPLAY_TIMEOUT 30 // sekunden
bool displaybeleuchtung(bool t){

  static bool hintergrund;
  static bool once;
  static unsigned long m;
  static bool s; // start/stop timeout.

  if(m == 0 || t){ // timer reset.

    m           = millis() + 1000 * DISPLAY_TIMEOUT; // Aktualiesiere timer.
    s           = true; // start timer.

    if(!hintergrund){
      
      hintergrund = true; // set display on.
      once        = true; // run...

    }

  }

  if(m <= millis() && s){ // timer match...

    hintergrund = false;  // set display off.
    once        = true;   // run...

  }

  debounce.update();
  if(debounce.fell()){

    hintergrund = !hintergrund;
    once = true;

  }

  if(hintergrund && once){ // display ist an

    lcd.display();
    lcd.setBacklight(255);

    m = millis() + 1000 * DISPLAY_TIMEOUT; // update timer
    s = true; // start timer.

  } else if(once){

    lcd.setBacklight(0);
    lcd.noDisplay();

    s = false; // stop timer

  }

  once = false;

  return hintergrund;

}

void tagec(){ // bluete Tagecounter

  bool relay_switching = digitalRead(licht_relay_p);
  static bool last_relay_state;

  if(relay_switching != last_relay_state){
    
    if(relay_switching == LOW)
      setings.bloom_counter++;
      
    write_EEPROM++;
    
  }
  
  last_relay_state = relay_switching;

}

void doEncoderA(){

  displaybeleuchtung(true); // update display timeout...

  if(digitalRead(encoderPinA) != A_set){ // debounce erneut
    
    A_set = !A_set;
    
    // stelle counter + 1 im Uhrzeigersinn
    if( A_set && !B_set)
      encoderPos += 1;
    
  }
  
}

void doEncoderB(){

  displaybeleuchtung(true); // update display timeout...

  if(digitalRead(encoderPinB) != B_set){
    
    B_set = !B_set;
    
    //  stelle counter - 1 gegen Uhrzeigersinn
    if( B_set && !A_set )
      encoderPos -= 1;
    
  }
  
}

/* Lese EEPROM in setings oder schreibe defaults in EEPROM */
void readEEPROM(){

  /* Lese EEPROM structure in speicher*/
  EEPROM.get(0, setings);

  if(setings.MAGIC_NUMBER != MAGIC_NUMBER ){ // Vergleiche Magic number wenn ungleich schreibe EEPROM mit defaults neu.

    Serial.println("write defaults!");

    EEPROM.put(0, setings); // Schreibe settings_b, enthält default einstellungen.

  }

}


void updateEEPROM(){

  if(write_EEPROM && save_EEPROM){

    setings_t setings_b;

    if(memcmp(&setings, &setings_b, sizeof setings) != 0){ // Do noting if noting to do

      EEPROM.put(0, setings);

    }

    write_EEPROM  = false;
    save_EEPROM   = false;

  }

}

void SplashScreen(){

  displaybeleuchtung(true); // set display timeout (30 sec.)

  lcd.setCursor(0, 0);
  lcd.print(F("..:: Gro-Matic ::.."));
  lcd.setCursor(0, 2);
  lcd.print(F(" Community Edition"));
  lcd.setCursor(0, 3);
  lcd.print(F("     V. 0.9.9.9"));
  Alarm.delay(3000);
  lcd.clear();

}

void setup(){

  Serial.begin(9600);
  Serial.println(sizeof(setings));

  /* Lese EEPROM in setings oder schreibe defaults in EEPROM */
  readEEPROM();
  Wire.begin();
  lcd.begin(20, 4); // stelle LCD groesse ein
  bme.begin();
  
  setSyncProvider(RTC.get);   // Function to get the time from RTC
  setSyncInterval(60000*5); // (5 Minuten)

  BH1750_Init(BH1750_address);
  Alarm.delay(500);

  SplashScreen();

  digitalWrite(licht_relay_p, HIGH);    // alle Relais Pins beim Start auf HIGH setzen und damit ausschalten.
  digitalWrite(lsr_relay_p, HIGH);
  digitalWrite(luft_relay, HIGH);
  digitalWrite(ventilator, HIGH);
  digitalWrite(irrigation, HIGH);

  pinMode(luft_relay,  OUTPUT);         // alle Relais Pins als ausgang setzen
  pinMode(licht_relay_p,  OUTPUT);
  pinMode(lsr_relay_p,  OUTPUT);
  pinMode(ventilator, OUTPUT);
  pinMode(irrigation, OUTPUT);
  
  pinMode(BUTTON_PIN, INPUT_PULLUP);    // den backlight Taster als Input setzen
  pinMode(wechslertPin, INPUT_PULLUP);  // Modus-Taster Pin wird als Eingang gesetzt
  pinMode(screenPin, INPUT_PULLUP);     // Modus-Taster Pin wird als Eingang gesetzt
  
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  
  digitalWrite(encoderPinA, HIGH);
  digitalWrite(encoderPinB, HIGH);

  // Backlight toogle.
  debounce.attach(BUTTON_PIN);
  debounce.interval(5); // interval in ms

  // switch screen.
  debounce2.attach(screenPin);
  debounce2.interval(5);

  debounce3.attach(wechslertPin);
  debounce3.interval(7);

  attachInterrupt(0, doEncoderA, CHANGE); // Encoder pin an interrupt 0 (pin 2)
  attachInterrupt(1, doEncoderB, CHANGE); // Encoder pin an interrupt 1 (pin 3)

  // erstelle die Custom character
  lcd.createChar(MOON,      Moon);
  lcd.createChar(SUN,       Sun);
  lcd.createChar(THERMO,    Thermo);
  lcd.createChar(RLF,       Rlf);
  lcd.createChar(WATER_ON,  Water_on);
  lcd.createChar(WATER_OFF, Water_off);
  lcd.createChar(VENTI_I,   Venti_I);
  lcd.createChar(VENTI_II,  Venti_II);

}

void loop(){

  // Alarm tasks.
  Alarm.delay(0);

  LTI();  // ruft die einfache LTI steuerung auf und prueft Temp und RLF und schaltet den Stufentrafo zwischen zwei Stufen.

  static unsigned int lastReportedPos;

  if(lastReportedPos != encoderPos)
    lastReportedPos = encoderPos;

  //***********************************************

  if(setings.lichtmodus == LSR){

    if((hour() >= setings.lsr_an) && (hour() < setings.lsr_aus)){
      
      digitalWrite(lsr_relay_p, LOW); //schaltet lsr um 5 Uhr an und um 22:59:59 Uhr aus
      digitalWrite(licht_relay_p, HIGH); //schaltet ndl Relais aus sollten sie noch an sein

      // Umluftventilator alle 15 minuten einschalten wenn licht an
      if((minute() >= 15 && minute() <= 29 ) || (minute() >= 45 && minute() <= 59)){
        
        digitalWrite(ventilator, LOW);
        
      } else {
        
        digitalWrite(ventilator, HIGH);
        
      }
      
    } else { 
      
      digitalWrite(lsr_relay_p, HIGH);
      
      if((hour() >= setings.grow_licht_aus) & (hour() < setings.grow_licht_an) || (hour() >= setings.bloom_licht_aus) & (hour() <= setings.bloom_licht_an))
        digitalWrite(licht_relay_p, HIGH);  //schaltet lsr Relais aus sollten sie noch an sein
      
      if((minute() >= 15) && (minute() <= 19)){ // schaltet Ventilator im Nachtmodus 1 x jede Stunde fuer 5 Min. an

        digitalWrite(ventilator, LOW);
        
      } else {

        digitalWrite(ventilator, HIGH);
        
      }
      
    }

  } else if(setings.lichtmodus == GROW){

    if((hour() >= setings.grow_licht_an) && (hour() < setings.grow_licht_aus)){ 
      
      digitalWrite(licht_relay_p, LOW); //schaltet ndl im Grow modus 18h licht um 5 Uhr an und um 22:59:59 Uhr aus
      digitalWrite(lsr_relay_p, HIGH);  //schaltet lsr Relais aus sollten sie noch an sein

      // Umluftventilator alle 15 minuten einschalten wenn licht an
      if((minute() >= 15 && minute() <= 29 ) || (minute() >= 45 && minute() <= 59) ){
        
        digitalWrite(ventilator, LOW);
        
      } else {
        
        digitalWrite(ventilator, HIGH);
        
      }

    } else {
      
      digitalWrite(licht_relay_p, HIGH);
      
      if((hour() >= setings.lsr_aus) & (hour() < setings.lsr_an))
        digitalWrite(lsr_relay_p, HIGH);  //schaltet lsr Relais aus sollten sie noch an sein
      
      if((minute() >= 15) && (minute() <= 19)){ // schaltet Ventilator im Nachtmodus 1 x jede Stunde fuer 5 Min. an

        digitalWrite(ventilator, LOW);
        
      } else {
        
        digitalWrite(ventilator, HIGH);
        
      }
      
    }

  } else if(setings.lichtmodus == BLOOM){

    if((hour() >= setings.grow_licht_an) && (hour() < setings.grow_licht_aus)){
      
      digitalWrite(licht_relay_p, LOW); //schaltet ndl im Grow modus 18h licht um 5 Uhr an und um 22:59:59 Uhr aus
      digitalWrite(lsr_relay_p, HIGH);  //schaltet lsr Relais aus sollten sie noch an sein
      
      // Umluftventilator alle 15 minuten einschalten wenn licht an
      if((minute() >= 15 &&  minute() <= 29) || (minute() >= 45 && minute() <= 59)){
        
        digitalWrite(ventilator, LOW);
        
      } else {
        
        digitalWrite(ventilator, HIGH);
        
      }
      
    } else { 
      
      setings.lichtmodus = BLOOM;
      write_EEPROM++;
      
    }
    
  } else if(setings.lichtmodus == BLOOM){
    
    tagec();

    if((hour() >= setings.bloom_licht_an) && (hour() < setings.bloom_licht_aus)){
      
      digitalWrite(licht_relay_p, LOW); //schaltet ndl im Bloom modus 12h licht um 5 Uhr an und um 16:00:00 Uhr aus
      digitalWrite(lsr_relay_p, HIGH);  //schaltet lsr Relais aus sollten sie noch an sein

      // Umluftventilator alle 15 minuten einschalten wenn licht an
      if((minute() >= 15 && minute() <= 29 ) || (minute() >= 45 && minute() <= 59)){
        
        digitalWrite(ventilator, LOW);
        
      } else {
        
        digitalWrite(ventilator, HIGH);
        
      }
      
    } else { 
      
      digitalWrite(licht_relay_p, HIGH);
      
      if((hour() >= setings.grow_licht_aus) & (hour() < setings.grow_licht_an) || (hour() >= setings.lsr_aus) & (hour() < setings.lsr_an) )
        digitalWrite(lsr_relay_p, HIGH);  //schaltet lsr Relais aus sollten sie noch an sein
      
      if((minute() >= 15) && (minute() <= 19)){ // schaltet Ventilator im Nachtmodus 1 x jede Stunde fuer 5 Min. an
 
        digitalWrite(ventilator, LOW);
        
      } else {
        
        digitalWrite(ventilator, HIGH);
        
      }
      
    }

  } // Lichtmodus Ende

  // Autobewaesserung
  if(setings.autowasser == 1){
  } // Autobewaesserung Ende

  Screens();
  //updateEEPROM();

}

void Screens(){

  static unsigned long screenBlock;
  static uint8_t screen;

  if(!displaybeleuchtung(false) || (millis() <= screenBlock + millis() && screenBlock != 0))
    return;

  screenBlock = 0;

  debounce2.update();
  if(debounce2.fell() || screen == 0){

    screen++;
    lcd.clear();
    displaybeleuchtung(true); // update display timeout.
    
  }

  if(screen == 1)
    Screen1(screen, screenBlock);
  else if(screen == 2)
    Screen2(screen, screenBlock);
  else if(screen == 3)
    Screen3(screen, screenBlock);
  else if(screen == 4)
    Screen4(screen, screenBlock);
  else if(screen == 5)
    Screen5(screen, screenBlock);
  else if(screen == 6)
    Screen6(screen, screenBlock);
  else if(screen == 7)
    Screen7(screen, screenBlock);
  else if(screen == 8)
    Screen8(screen, screenBlock);
  else if(screen == 9)
    Screen9(screen, screenBlock);
  else if(screen == 10)
    Screen10(screen, screenBlock);
  else if(screen == 11)
    Screen11(screen, screenBlock);
  else if(screen == 12)
    Screen12(screen, screenBlock);
  else if(screen == 13)
    Screen13(screen, screenBlock);
  else if(screen == 14)
    Screen14(screen, screenBlock);
  else if(screen == 15)
    Screen15(screen, screenBlock);
  else if(screen == 16)
    Screen16(screen, screenBlock);
  else if(screen == 17)
    Screen17(screen, screenBlock);
  else if(screen == 18)
    Screen18(screen, screenBlock);

}

void Screen1(uint8_t &screen, unsigned long &screenBlock){

    static bool relay_bloom_switching;
    static bool relay_lsr_switching;
    static bool relay_grow_switching;
    
    // Rufe funktionen für Seite 1 auf
    displayTime();        // zeige die RTC Daten auf dem LCD display,
    bme280();          // zeige temp und rlf auf dem LCD display,
    DS3231temp();  // prüfe gehaeuse temp und gib sie auf dem display aus
    gy30();  // Luxmeter
    
    debounce3.update();
    if(debounce3.fell()){

      displaybeleuchtung(true); // update display timeout...
      
      setings.lichtmodus++;  // lichtmodus wird um +1 erhöht
      write_EEPROM++;
    
    }

    // Ventilator Icon
    static bool           ventiicon;
    static unsigned long  previousMillis;
    unsigned long         currentMillis = millis();
    const unsigned long   OnTime1 = 300;
    const unsigned long   OnTime2 = OnTime1;
    
    if(digitalRead(ventilator) == LOW){
      
      if((ventiicon == HIGH) && (currentMillis - previousMillis >= OnTime1)){
        
        ventiicon = LOW;
        previousMillis = currentMillis;
        lcd.setCursor(17, 2);
        lcd.write(VENTI_I);
        
      }

      if((ventiicon == LOW) && (currentMillis - previousMillis >= OnTime2)){
        
        ventiicon = HIGH;
        previousMillis = currentMillis;
        lcd.setCursor(17, 2);
        lcd.write(VENTI_II);
        
      }
    }
    
    if(digitalRead(ventilator) == HIGH){
      
      lcd.setCursor(17, 2);
      lcd.print(F(" "));
      
    }
    
    //*************************************Programm-Modis**************************************

    // Wenn Lichtmodus 0 ist, starte im LSR modus
    if(setings.lichtmodus == LSR){
      
      lcd.setCursor(10, 3);
      lcd.print(F("  LSR Mode"));
      relay_lsr_switching = digitalRead(lsr_relay_p);
      
      if(relay_lsr_switching == LOW){
        
        lcd.setCursor(19, 2);
        lcd.write(SUN);
        
      }
      
      if(relay_lsr_switching == HIGH){
        
        lcd.setCursor(19, 2);
        lcd.write(MOON);
      
      }
      
    } else if(setings.lichtmodus == GROW){
      
      lcd.setCursor(10, 3);
      lcd.print(F(" Grow Mode"));
      relay_grow_switching = digitalRead(licht_relay_p);
      
      if(relay_grow_switching == LOW){
        
        lcd.setCursor(19, 2);
        lcd.write(SUN);
        
      }
      
      if(relay_grow_switching == HIGH){
        
        lcd.setCursor(19, 2);
        lcd.write(MOON);
        
      }
      
    } else if(setings.lichtmodus == BLOOM){
      
      lcd.setCursor(10, 3);
      lcd.print(F("Bloom Mode"));
      relay_bloom_switching = digitalRead(licht_relay_p);
      
      if(relay_bloom_switching == LOW){
        
        lcd.setCursor(19, 2);
        lcd.write(SUN);
        
      }
      
      if(relay_bloom_switching == HIGH){
        
        lcd.setCursor(19, 2);
        lcd.write(MOON);
        
      }
      
    } else { // Wenn der Lichtmodus auf 3 springt, setzte ihn wieder zurück auf 0 um von vorne zu beginnen

      setings.lichtmodus = LSR;
      
    } // Lichtmodus Ende

}

void Screen2(uint8_t &screen, unsigned long &screenBlock){

    static uint8_t letztertag;
    static uint8_t letztermonat;
    
    debounce3.update();
    if(debounce3.fell()){

      displaybeleuchtung(true); // update display timeout...
      
      for(int i = 0; i < 512; i++)
        EEPROM.write(i, 0);
        
      asm volatile ("jmp 0");
      
    }
    
    lcd.setCursor(0, 0);
    lcd.print(F("Starttag am: "));
    
    if(letztertag < 10)  
      lcd.print(F("0"));
    
    lcd.print(letztertag);
    lcd.print(F("."));
    
    if(letztermonat < 10)
      lcd.print(F("0"));
    
    lcd.print(letztermonat);
    lcd.setCursor(0, 1);
    lcd.print(F("Bl"));
    lcd.print((char)0xF5);
    lcd.print(F("tetag:"));
    lcd.print(setings.bloom_counter);
    lcd.setCursor(0, 2);
    lcd.print(F("dr"));
    lcd.print((char)0xF5);
    lcd.print(F("cke Enc.taste zum"));
    lcd.setCursor(0, 3);
    lcd.print(F("Speicher l"));
    lcd.print((char)0xEF);
    lcd.print(F("schen."));
  
}

void Screen3(uint8_t &screen, unsigned long &screenBlock){

    debounce3.update();
    if(debounce3.fell()){

      displaybeleuchtung(true); // update display timeout...
      
      setings.autowasser++;
      write_EEPROM++;

    }

    //*************************************Programm-Modis**************************************

    // Wenn Lichtmodus 0 ist, starte im LSR modus
    if(setings.autowasser == true){
      
      lcd.setCursor(0, 2);
      lcd.print(F("Autobew"));
      lcd.print((char)0xE1);
      lcd.print(F("sserung:"));
      lcd.setCursor(0, 3);
      lcd.write(WATER_OFF);
      lcd.print(F(" aus"));
      
    } else if(setings.autowasser == false){
      
      lcd.setCursor(0, 2);
      lcd.print(F("Autobew"));
      lcd.print((char)0xE1);
      lcd.print(F("sserung:"));
      lcd.setCursor(0, 3);
      lcd.write(WATER_ON);
      lcd.print(F(" an "));
      
    } else {
      
      setings.autowasser = true;
      
    } // Autobewaesserung Ende

    lcd.setCursor(0, 0);
    lcd.print(F("Boden "));
    lcd.write(RLF);
    lcd.print(F(" "));
    lcd.print(bodensensor.getCapacitance()); //lese bodensensor
    lcd.setCursor(0, 1);
    lcd.print(F("Boden "));
    lcd.write(THERMO);
    lcd.print(F(" "));
    lcd.print(bodensensor.getTemperature() / (float)10); //lese temperatur register des bodensensors
    lcd.print((char)223);
    lcd.print(F("C "));
  
}

void Screen4(uint8_t &screen, unsigned long &screenBlock){
  
    lcd.setCursor(0, 0);
    lcd.print(F("Schaltzeiten Licht"));
    lcd.setCursor(0, 1);
    lcd.print(F("LSR:  "));
    lcd.print(setings.lsr_an);
    lcd.print(F(":00-"));
    lcd.print(setings.lsr_aus);
    lcd.print(F(":00 Uhr"));
    lcd.setCursor(0, 2);
    lcd.print(F("Grow: "));
    lcd.print(setings.grow_licht_an);
    lcd.print(F(":00-"));
    lcd.print(setings.grow_licht_aus);
    lcd.print(F(":00 Uhr"));
    lcd.setCursor(0, 3);
    lcd.print(F("Bloom:"));
    lcd.print(setings.bloom_licht_an);
    lcd.print(F(":00-"));
    lcd.print(setings.bloom_licht_aus);
    lcd.print(F(":00 Uhr"));

    debounce3.update();
    if(debounce3.fell()){

      displaybeleuchtung(true); // update display timeout...

      lcd.clear();
      //screenBlock = 50;
      screen = 7;
      
    }
    
}

void Screen5(uint8_t &screen, unsigned long &screenBlock){

    lcd.setCursor(0, 0);
    lcd.print(F("eingest. LTI Werte"));
    lcd.setCursor(0, 1);
    lcd.print(F("LSR:  "));
    lcd.print(setings.lsr_temp);
    lcd.print((char)223);
    lcd.print(F("C:"));
    lcd.print(setings.lsr_rlf);
    lcd.print(F("%"));
    lcd.setCursor(0, 2);
    lcd.print(F("Grow: "));
    lcd.print(setings.grow_temp);
    lcd.print((char)223);
    lcd.print(F("C:"));
    lcd.print(setings.grow_rlf);
    lcd.print(F("%"));
    lcd.setCursor(0, 3);
    lcd.print(F("Bloom:"));
    lcd.print(setings.bloom_temp);
    lcd.print((char)223);
    lcd.print(F("C:"));
    lcd.print(setings.bloom_rlf);
    lcd.print(F("%"));

    debounce3.update();
    if(debounce3.fell()){

      displaybeleuchtung(true); // update display timeout...
      
      lcd.clear();
      //screenBlock = 200;
      temp_bereich = 0;
      rlf_bereich = 0;
      screen = 8;

    }
  
}

void Screen6(uint8_t &screen, unsigned long &screenBlock){
    
    screen = 10; // geht weiter zu Seite 10 
  
}

void Screen7(uint8_t &screen, unsigned long &screenBlock){

    static uint8_t anaus;
    
    if(encoderPos == 24)
      encoderPos = 0;

    if(encoderPos >= 24){
      
      lcd.clear();
      encoderPos = 23;
    
    }

    if(anaus == 0){

      lcd.setCursor(0, 0);
      lcd.print(F("LSR an:    "));
      
      if(encoderPos < 10){
        lcd.print("0");
      }
      
      lcd.print(encoderPos);
      lcd.print(F(" Uhr"));
      lcd.setCursor(0, 1);
      lcd.print(F("Startzeit f"));
      lcd.print((char)0xF5);
      lcd.print(F("r LSR"));
      lcd.setCursor(0, 2);
      lcd.print(F("w"));
      lcd.print((char)0xE1);
      lcd.print(F("hlen. optimale"));
      lcd.setCursor(0, 3);
      lcd.print(F("Dauer 18 Stunden"));

      debounce3.update();
      if(debounce3.fell()){

        displaybeleuchtung(true); // update display timeout...
        setings.lsr_an = encoderPos;
        write_EEPROM++;
        lcd.clear();
        anaus++;
        
      }
      
    }

    if(anaus == 1){
      
      lcd.setCursor(0, 0);
      lcd.print(F("LSR aus:   "));
      
      if(encoderPos < 10)
        lcd.print("0");
 
      lcd.print(encoderPos);
      lcd.print(F(" Uhr"));
      lcd.setCursor(0, 1);
      lcd.print(F("Endzeit f"));
      lcd.print((char)0xF5);
      lcd.print(F("r LSR"));
      lcd.setCursor(0, 2);
      lcd.print(F("w"));
      lcd.print((char)0xE1);
      lcd.print(F("hlen. optimale"));
      lcd.setCursor(0, 3);
      lcd.print(F("Dauer 18 Stunden"));

      debounce3.update();
      if(debounce3.fell()){

        displaybeleuchtung(true); // update display timeout...
        
        if(encoderPos == 0){
          
          encoderPos = 23;
          setings.lsr_aus = encoderPos;
          
        } else {
          
          setings.lsr_aus = encoderPos;
        
        }

        write_EEPROM++;
        lcd.clear();
        anaus++;
      
      }
    
    }

    if(anaus == 2){
      
      lcd.setCursor(0, 0);
      lcd.print(F("Grow an:   "));
      
      if(encoderPos < 10)
        lcd.print("0");

      lcd.print(encoderPos);
      lcd.print(F(" Uhr"));
      lcd.setCursor(0, 1);
      lcd.print(F("Startzeit f"));
      lcd.print((char)0xF5);
      lcd.print(F("r Grow"));
      lcd.setCursor(0, 2);
      lcd.print(F("w"));
      lcd.print((char)0xE1);
      lcd.print(F("hlen. optimale"));
      lcd.setCursor(0, 3);
      lcd.print(F("Dauer 18 Stunden"));

      debounce3.update();
      if(debounce3.fell()){

        displaybeleuchtung(true); // update display timeout...
        
        setings.grow_licht_an = encoderPos;
        write_EEPROM++;
        lcd.clear();
        anaus++;
        
      }
      
    }
    
    if(anaus == 3){
      
      lcd.setCursor(0, 0);
      lcd.print(F("Grow aus:  "));
      
      if(encoderPos < 10)
        lcd.print("0");
 
      lcd.print(encoderPos);
      lcd.print(F(" Uhr"));
      lcd.setCursor(0, 1);
      lcd.print(F("Endzeit f"));
      lcd.print((char)0xF5);
      lcd.print(F("r Grow"));
      lcd.setCursor(0, 2);
      lcd.print(F("w"));
      lcd.print((char)0xE1);
      lcd.print(F("hlen. optimale"));
      lcd.setCursor(0, 3);
      lcd.print(F("Dauer 18 Stunden"));

      debounce3.update();
      if(debounce3.fell()){

        displaybeleuchtung(true); // update display timeout...
        
        if(encoderPos == 0){
          
          encoderPos = 23;
          setings.grow_licht_aus = encoderPos;
        
        } else {
          
          setings.grow_licht_aus = encoderPos;
          
        }
        
        write_EEPROM++;
        lcd.clear();
        anaus++;
        
      }
      
    }
    
    if(anaus == 4){
      
      lcd.setCursor(0, 0);
      lcd.print(F("Bloom an:  "));
      
      if(encoderPos < 10)
        lcd.print("0");

      lcd.print(encoderPos);
      lcd.print(F(" Uhr"));
      lcd.setCursor(0, 1);
      lcd.print(F("Startzeit f"));
      lcd.print((char)0xF5);
      lcd.print(F("r Bloom"));
      lcd.setCursor(0, 2);
      lcd.print(F("w"));
      lcd.print((char)0xE1);
      lcd.print(F("hlen. optimale"));
      lcd.setCursor(0, 3);
      lcd.print(F("Dauer 12 Stunden"));

      debounce3.update();
      if(debounce3.fell()){

        displaybeleuchtung(true); // update display timeout...
        
        setings.bloom_licht_an = encoderPos;
        write_EEPROM++;
        lcd.clear();
        anaus++;
        
      }

    }
 
    if(anaus == 5){
      
      lcd.setCursor(0, 0);
      lcd.print(F("Bloom aus: "));
      
      if(encoderPos < 10)        
        lcd.print("0");

      lcd.print(encoderPos);
      lcd.print(F(" Uhr"));
      lcd.setCursor(0, 1);
      lcd.print(F("Startzeit f"));
      lcd.print((char)0xF5);
      lcd.print(F("r Bloom"));
      lcd.setCursor(0, 2);
      lcd.print(F("w"));
      lcd.print((char)0xE1);
      lcd.print(F("hlen. optimale"));
      lcd.setCursor(0, 3);
      lcd.print(F("Dauer 12 Stunden"));

      debounce3.update();
      if(debounce3.fell()){

        displaybeleuchtung(true); // update display timeout...
        
        if(encoderPos == 0){
          
          encoderPos = 23;
          setings.bloom_licht_aus = encoderPos;
          
        } else {
          
          setings.bloom_licht_aus = encoderPos;
          
        }
        
        write_EEPROM++;
        lcd.clear();
        //screenBlock = 100;
        screen = 4;
        anaus = 0;
        
      }
      
    }

}

void Screen8(uint8_t &screen, unsigned long &screenBlock){

    if(encoderPos >= 29){
      
      encoderPos = 17;
      
    }
    
    else if(encoderPos <= 16){
      
      lcd.clear();
      encoderPos = 28;
      
    }

    if(temp_bereich == 0){
      
      lcd.setCursor(0, 0);
      lcd.print(F("LSR max. "));
      lcd.write(THERMO);
      lcd.print(F(" :  "));
      lcd.print(encoderPos);
      lcd.print((char)223);
      lcd.print(F("C"));
      lcd.setCursor(0, 1);
      lcd.print(F("optimaler bereich"));
      lcd.setCursor(0, 2);
      lcd.print(F("zwischen 22"));
      lcd.print((char)223);
      lcd.print(F("C"));
      lcd.print(F(" - 25"));
      lcd.print((char)223);
      lcd.print(F("C"));

      debounce3.update();
      if(debounce3.fell()){

        displaybeleuchtung(true); // update display timeout...
        
        setings.lsr_temp = encoderPos;
        write_EEPROM++;
        lcd.clear();
        temp_bereich++;
        
      }

    }

    if(temp_bereich == 1){
      
      lcd.setCursor(0, 0);
      lcd.print(F("Grow max. "));
      lcd.write(THERMO);
      lcd.print(F(" : "));
      lcd.print(encoderPos);
      lcd.print((char)223);
      lcd.print(F("C"));
      lcd.setCursor(0, 1);
      lcd.print(F("optimaler bereich"));
      lcd.setCursor(0, 2);
      lcd.print(F("zwischen 21"));
      lcd.print((char)223);
      lcd.print(F("C"));
      lcd.print(F(" - 23"));
      lcd.print((char)223);
      lcd.print(F("C"));

      debounce3.update();
      if(debounce3.fell()){

        displaybeleuchtung(true); // update display timeout...
        
        setings.grow_temp = encoderPos;
        write_EEPROM++;
        lcd.clear();
        temp_bereich++;
        
      }
      
    }

    if(temp_bereich == 2){
      
      lcd.setCursor(0, 0);
      lcd.print(F("Bloom max. "));
      lcd.write(THERMO);
      lcd.print(F(" :"));
      lcd.print(encoderPos);
      lcd.print((char)223);
      lcd.print(F("C"));
      lcd.setCursor(0, 1);
      lcd.print(F("optimaler bereich"));
      lcd.setCursor(0, 2);
      lcd.print(F("zwischen 20"));
      lcd.print((char)223);
      lcd.print(F("C"));
      lcd.print(F(" - 22"));
      lcd.print((char)223);
      lcd.print(F("C"));

      debounce3.update();
      if(debounce3.fell()){
        
        displaybeleuchtung(true); // update display timeout...
        
        setings.bloom_temp = encoderPos;
        write_EEPROM++;
        lcd.clear();
        screen = 9;
        
      }
      
    }
  
}

void Screen9(uint8_t &screen, unsigned long &screenBlock){

    if(encoderPos >= 71){
      
      encoderPos = 15;
      
    } else if(encoderPos <= 14){
      
      lcd.clear();
      encoderPos = 70;
      
    }

    if(rlf_bereich == 0){
      
      lcd.setCursor(0, 0);
      lcd.print(F("LSR max. "));
      lcd.write(RLF);
      lcd.print(F(" : "));
      lcd.print(encoderPos);
      lcd.print(F(" %"));
      lcd.setCursor(0, 1);
      lcd.print(F("optimaler bereich"));
      lcd.setCursor(0, 2);
      lcd.print(F("zwischen 55 % - 60 %"));

      debounce3.update();
      if(debounce3.fell()){

        displaybeleuchtung(true); // update display timeout...
        
        setings.lsr_rlf = (double) encoderPos;
        write_EEPROM++;
        lcd.clear();
        rlf_bereich++;
        
      }
      
    }
    
    if(rlf_bereich == 1){
      
      lcd.setCursor(0, 0);
      lcd.print(F("Grow max. "));
      lcd.write(RLF);
      lcd.print(F(" : "));
      lcd.print(encoderPos);
      lcd.print(F(" %"));
      lcd.setCursor(0, 1);
      lcd.print(F("optimaler bereich"));
      lcd.setCursor(0, 2);
      lcd.print(F("zwischen 50 % - 55 %"));

      debounce3.update();
      if(debounce3.fell()){

        displaybeleuchtung(true); // update display timeout...
        
        setings.grow_rlf = (double) encoderPos;
        write_EEPROM++;
        lcd.clear();
        rlf_bereich++;
        
      }
      
    }
    
    if(rlf_bereich == 2){
      
      lcd.setCursor(0, 0);
      lcd.print(F("Bloom max. "));
      lcd.write(RLF);
      lcd.print(F(" : "));
      lcd.print(encoderPos);
      lcd.print(F(" %"));
      lcd.setCursor(0, 1);
      lcd.print(F("optimal nicht "));
      lcd.print((char)0xF5);
      lcd.print(F("ber"));
      lcd.setCursor(0, 2);
      lcd.print(F("40 % RLF"));

      debounce3.update();
      if(debounce3.fell()){

        displaybeleuchtung(true); // update display timeout...
        
        setings.bloom_rlf = (double) encoderPos;
        write_EEPROM++;
        lcd.clear();
        rlf_bereich++;
        
      }
      
    }
    
    if(rlf_bereich == 3){
      
      temp_bereich = 0;
      rlf_bereich = 0;
      screen = 5;
      
    }
      
}

void Screen10(uint8_t &screen, unsigned long &screenBlock){
  
    if(encoderPos > 1){
      
      encoderPos = 1;
      
    } else if(encoderPos < 0){
      
      lcd.clear();
      encoderPos = 0;
      
    }
    
    lcd.setCursor(0, 0);
    lcd.print(F("RTC einstellen?"));
    lcd.setCursor(0, 1);

    const char *jein[] = {"nein", "ja"};
    lcd.print(jein[encoderPos]);

    debounce3.update();
    if(debounce3.fell()){

      displaybeleuchtung(true); // update display timeout...
      
      if(encoderPos == 0)
        screen = 13;
 
      if(encoderPos == 1)
        screen = 12;

    }
  
}

void Screen11(uint8_t &screen, unsigned long &screenBlock){
  
     screen = 1;

}

void Screen12(uint8_t &screen, unsigned long &screenBlock){

    static uint8_t zeitstellen;
    // Variable (struct) zum einstellen der RTC
    static tmElements_t tm;

    if(zeitstellen == 0){
      
      if(encoderPos >= 32){
        
        encoderPos = 1;
        
      } else if(encoderPos <= 0){
        
        lcd.clear();
        encoderPos = 31;
        
      }

      lcd.setCursor(0, 0);
      lcd.print(F("Tag einstellen:"));
      lcd.setCursor(0, 1);
      
      if(encoderPos < 10)
        lcd.print("0");
      
      lcd.print(encoderPos);
      lcd.print(F("."));
      
      if(tm.Month < 10)
        lcd.print("0");
 
      lcd.print(tm.Month);
      lcd.print(F("."));
      lcd.print(tm.Year);

      debounce3.update();
      if(debounce3.fell()){

        displaybeleuchtung(true); // update display timeout...
        
        tm.Day = encoderPos;
        lcd.clear();
        zeitstellen++;
        
      }

    }

    if(zeitstellen == 1){
      
      if(encoderPos >= 13){
        
        encoderPos = 1;
        
      } else if(encoderPos <= 0){
        
        lcd.clear();
        encoderPos = 12;
        
      }

      lcd.setCursor(0, 0);
      lcd.print(F("Monat einstellen:"));
      lcd.setCursor(0, 1);
      
      if(tm.Day < 10)
        lcd.print("0");
        
      lcd.print(tm.Day);
      lcd.print(F("."));
      
      if(encoderPos < 10)
        lcd.print("0");

      lcd.print(encoderPos);
      lcd.print(F("."));
      lcd.print(tm.Year);


      debounce3.update();
      if(debounce3.fell()){

        displaybeleuchtung(true); // update display timeout...
        
        tm.Month = encoderPos;
        lcd.clear();
        zeitstellen++;
        
      }

    }

    if(zeitstellen == 2){
      
      encoderPos = 16;
      
      if(encoderPos <= 15){
        
        lcd.clear();
        encoderPos = 50;
        
      } else if(encoderPos >= 51){
        
        encoderPos = 16;
        
      }

      lcd.setCursor(0, 0);
      lcd.print(F("Jahr einstellen:"));
      lcd.setCursor(0, 1);
      
      if(tm.Day < 10)
        lcd.print("0");

      lcd.print(tm.Day);
      lcd.print(F("."));

      if(tm.Month < 10)
        lcd.print("0");

      lcd.print(tm.Month);
      lcd.print(F("."));
      lcd.print(2000 + encoderPos); // the nex year 3000 bug :)

      debounce3.update();
      if(debounce3.fell()){

        displaybeleuchtung(true); // update display timeout...
        
        tm.Year = 2000 + encoderPos;
        lcd.clear();
        zeitstellen++;
        
      }

    }

    if(zeitstellen == 3){
      
      if(encoderPos == 24)
        encoderPos = 0;
 
      if(encoderPos >= 24){
        
        lcd.clear();
        encoderPos = 23;
        
      }

      lcd.setCursor(0, 0);
      lcd.print(F("Stunde einstellen:"));
      lcd.setCursor(0, 1);
 
      if(encoderPos < 10)
        lcd.print("0");
        
      lcd.print(encoderPos);
      lcd.print(F(":"));
      
      if(tm.Minute < 10)
        lcd.print("0");

      lcd.print(tm.Minute);
      lcd.print(F(":"));

      if(tm.Second < 10)
        lcd.print("0");

      lcd.print(tm.Second);

      debounce3.update();
      if(debounce3.fell()){

        displaybeleuchtung(true); // update display timeout...

        tm.Hour = encoderPos;
        lcd.clear();
        zeitstellen++;
        
      }

    }

    if(zeitstellen == 4){
      
      if(encoderPos == 60){
       
        encoderPos = 0;
        
      } else if(encoderPos == 65535){
        
        lcd.clear();
        encoderPos = 59;
        
      }

      lcd.setCursor(0, 0);
      lcd.print(F("Minute einstellen:"));
      lcd.setCursor(0, 1);
      
      if(tm.Hour < 10)
        lcd.print("0");

      lcd.print(tm.Hour);
      lcd.print(F(":"));
      
      if(encoderPos < 10)
        lcd.print("0");

      lcd.print(encoderPos);
      lcd.print(F(":"));
      if(tm.Second < 10)
        lcd.print("0");

      lcd.print(tm.Second);

      debounce3.update();
      if(debounce3.fell()){

        displaybeleuchtung(true); // update display timeout...

        tm.Minute = encoderPos;
        lcd.clear();
        zeitstellen++;
        
      }

    }

    if(zeitstellen == 5){
      
      lcd.setCursor(0, 0);

      const char *c_dayOfWeek[] = { "Sonntag", "Montag", "Dienstag", "Mittwoch", "Donnerstag", "Freitag", "Samstag"};
      lcd.print(c_dayOfWeek[encoderPos]);
 
      lcd.setCursor(0, 1);
      
      if(tm.Hour < 10)
        lcd.print("0");

      lcd.print(tm.Hour);
      lcd.print(F(":"));
      
      if(tm.Minute < 10)
        lcd.print("0");

      lcd.print(tm.Minute);
      
      lcd.print(F(":"));
      if(tm.Second < 10)
        lcd.print("0");

      lcd.print(tm.Second);
      lcd.print(F(" "));
      
      if(tm.Day < 10)
        lcd.print("0");

      lcd.print(tm.Day);

      lcd.print(F("."));
      
      if(tm.Month < 10)
        lcd.print("0");
        
      lcd.print(tm.Month);
      lcd.print(F("."));
      lcd.print(tm.Year);

      lcd.setCursor(0, 2);
      lcd.print("best");
      lcd.print((char)0xE1);
      lcd.print("tigen um Datum,");
      lcd.setCursor(0, 3);
      lcd.print("Zeit zu setzen.");

      debounce3.update();
      if(debounce3.fell()){

        displaybeleuchtung(true); // update display timeout...
        
        // Set RTC time.
        RTC.write(tm);

        // Set system time from RTC
        setTime(RTC.get());

        lcd.clear();
        screen = 1;
      }

    }
  
}

void Screen13(uint8_t &screen, unsigned long &screenBlock){
  
    if(encoderPos > 1){
      
      encoderPos = 1;
      
    } else if(encoderPos < 0){
      
      lcd.clear();
      encoderPos = 0;
      
    }
    
    lcd.setCursor(0, 0);
    lcd.print(F("Bew"));
    lcd.print((char)0xE1);
    lcd.print(F("sserungszeiten"));
    lcd.setCursor(0, 1);
    lcd.print(F("einstellen?"));
    lcd.setCursor(0, 2);

    const char *jein[] = {"nein", "ja"};
    lcd.print(jein[encoderPos]);

    debounce3.update();
    if(debounce3.fell()){

      displaybeleuchtung(true); // update display timeout...
      
      if(encoderPos == 0)
        screen = 1;
  
      if(encoderPos == 1){
        
        lcd.clear();
        screen = 14;
        
      }
      
    }
  
}

void Screen14(uint8_t &screen, unsigned long &screenBlock){

    if(encoderPos == 24)
      encoderPos = 0;

    if(encoderPos >= 24){
      
      lcd.clear();
      encoderPos = 23;
      
    }

    lcd.setCursor(0, 0);
    lcd.print(F("Startzeit setzen:"));
    lcd.setCursor(0, 1);
    
    if(encoderPos < 10)
      lcd.print("0");
      
    lcd.print(encoderPos);
    lcd.print(F(" Uhr"));

    debounce3.update();
    if(debounce3.fell()){

      displaybeleuchtung(true); // update display timeout...
      
      setings.startwasser = encoderPos;
      write_EEPROM++;
      lcd.clear();
      screen = 15;
      
    }

  
}

void Screen15(uint8_t &screen, unsigned long &screenBlock){
    
    if(encoderPos == 60){
      
      encoderPos = 0;
      
    } else if(encoderPos == 65535){
      
      lcd.clear();
      encoderPos = 59;
      
    }

    lcd.setCursor(0, 0);
    lcd.print(F("Start Minute:"));
    lcd.setCursor(0, 1);
    
    if(encoderPos < 10)
      lcd.print("0");

    lcd.print(encoderPos);
    lcd.print(F(" Min."));

    debounce3.update();
    if(debounce3.fell()){

      displaybeleuchtung(true); // update display timeout...
      
      setings.startwassermin = encoderPos;
      write_EEPROM++;
      lcd.clear();
      screen = 16;
      
    }
  
}

void Screen16(uint8_t &screen, unsigned long &screenBlock){

      if(encoderPos == 60){
      
      encoderPos = 0;
      
    } else if(encoderPos == 65535){
      
      lcd.clear();
      encoderPos = 59;
      
    }

    lcd.setCursor(0, 0);
    lcd.print(F("Ende Minute:"));
    lcd.setCursor(0, 1);
    
    if(encoderPos < 10)
      lcd.print("0");

    lcd.print(encoderPos);
    lcd.print(F(" Min."));

    debounce3.update();
    if(debounce3.fell()){

      displaybeleuchtung(true); // update display timeout...
      
      setings.auswasser = encoderPos;
      write_EEPROM++;
      lcd.clear();
      screen = 17;
      
    }

}

void Screen17(uint8_t &screen, unsigned long &screenBlock){

    if(encoderPos == 60){
      
      encoderPos = 0;
      
    } else if(encoderPos == 65535){
      
      lcd.clear();
      encoderPos = 59;
      
    }

    lcd.setCursor(0, 0);
    lcd.print(F("Dauer in Sekunden:"));
    lcd.setCursor(0, 1);
    
    if(encoderPos < 10)
      lcd.print("0");
 
    lcd.print(encoderPos);
    lcd.print(F(" Sek."));

    debounce3.update();
    if(debounce3.fell()){

      displaybeleuchtung(true); // update display timeout...

      setings.sekauswasser = encoderPos;
      write_EEPROM++;
      lcd.clear();
      screen = 18;
      
    }
  
}

void Screen18(uint8_t &screen, unsigned long &screenBlock){

    lcd.setCursor(0, 0);
    lcd.print(F("Startzeit:"));
    lcd.setCursor(0, 1);
    
    if(setings.startwasser < 10)
      lcd.print("0");

    lcd.print(setings.startwasser);
    lcd.print(":");

    if(setings.startwassermin < 10)
      lcd.print("0");

    lcd.print(setings.startwassermin);
    lcd.print(":00 Uhr");
    lcd.setCursor(0, 2);
    lcd.print(F("Ende:"));
    lcd.setCursor(0, 3);

    if(setings.startwasser < 10)
      lcd.print("0");

    lcd.print(setings.startwasser);
    lcd.print(":");

    if(setings.auswasser < 10)
      lcd.print("0");

    lcd.print(setings.auswasser);
    lcd.print(":");

    if(setings.sekauswasser < 10)
      lcd.print("0");

    lcd.print(setings.sekauswasser);

    debounce3.update();
    if(debounce3.fell()){

      displaybeleuchtung(true); // update display timeout...

      asm volatile ("jmp 0");
      
    }

}

