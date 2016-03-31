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

#include "Wire.h"                   // https://www.arduino.cc/en/Reference/Wire (included with Arduino IDE)
#include "LiquidCrystal_I2C.h"      // https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library
#include "DS3231.h"                 // https://github.com/bpg/DS3231
#include "EEPROM.h"                 // https://www.arduino.cc/en/Reference/EEPROM
#include "I2CSoilMoistureSensor.h"  // https://github.com/Miceuz/i2c-moisture-sensor
#include "Adafruit_Sensor.h"        // https://github.com/adafruit/Adafruit_Sensor
#include "Adafruit_BME280.h"        // https://github.com/adafruit/Adafruit_BME280_Library
#include "Time.h"                   // https://github.com/PaulStoffregen/Time
#include "TimeAlarms.h"             // https://www.pjrc.com/teensy/td_libs_TimeAlarms.html

/* Festlegen der verschiedenen Lichtprogramme, 0=LSR (Standart), 1, GROW, 2, BLOOM */
enum { LSR, GROW, BLOOM };

/* Change to Uptdate EEPROM to new DEFAULT settings, update the stuct after for change setings. */
const uint32_t MAGIC_NUMBER = 0xAAEBCCDF;

/* Structure hält default einstellungen, Überschrieben on in EEPROM gespeicherter structure */
struct setings_t {

  uint32_t MAGIC_NUMBER   = MAGIC_NUMBER;

  byte lichtmodus     = LSR;   // Speichern des gewaehlten Lichtmodus einstellen
  bool autowasse      = false; // Autobewasserung, on false = disabled
  
  byte bloom_counter  = 0;    // Speichern des bloom Tage counters.
  byte starttag       = 0;    // Speichern des Start Tages
  byte startmonat     = 0;    // Speichern des Start Monats

/* Ab hier Zeit für die Belichtungsmodis einstellen */
  byte lsr_an           = 5;     // Startzeit des LSR Modis
  byte lsr_aus          = 22;    // Endzeit des LSR Modis
  byte grow_licht_an    = 5;     // Startzeit des Grow Modis
  byte grow_licht_aus   = 22;    // Endzeit des Grow Modis
  byte bloom_licht_an   = 5;     // Startzeit des Bloom Modis
  byte bloom_licht_aus  = 16;    // Endzeit des Grow Modis
 
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

} setings_a, setings_b; // wenn sich structure a von b unterscheidet dann schreibe EEPROM neu...

#define DS3231_I2C_ADDRESS 0x68

#define BACKLIGHT_PIN (3)
#define LED_ADDR (0x27)  // might need to be 0x3F, if 0x27 doesn't work
LiquidCrystal_I2C lcd(LED_ADDR, 20, 4);

// GY-30 Lux Meter
int BH1750_address = 0x23;  // I2C Addresse des GY-30
byte buff[2];

//Backlight button
const int buttonPin = 10;
int buttonState = LOW;
byte buttonGedrueckt = 0;
unsigned long buttonZeit = 0;  // Zeit beim drücken des Tasters
//**************************************************************
//Displayfunktionen
byte lcdbereinigen = 0;  // dispaly Clear funktion
byte hintergrund = 1;    // schalte dispaly an menue
//**************************************************************
//Programm modus und resert Taster
const int wechslertPin = A1;  // Pinnummer des Tasters für zum Lichtmodus wechseln und Eprom Reset
int wechslertStatus = LOW;  // aktuelles Signal vom Eingangspin des Wechslertasters
byte wechslertGedrueckt = 0;  // abfragen ob Taster gedrückt wurde
unsigned int entprellZeit = 200;  // Zeit für Entprellung, anpassen!
unsigned long wechslertZeit = 0;  // Zeit beim drücken des Tasters
byte tage_reset = 0;
//**************************************************************
// Verschiedene Variablen
byte speichern = 0;  // Setzt autosave auf aus, erst wenn Lichtmodus gewechselt wird, wird auch gespeichert
byte daycounter_speichern = 0;

byte relay_bloom_switching = 0;
byte relay_grow_switching = 0;
byte relay_lsr_switching = 0;

//**************Anim Icons
unsigned long previousMillis = 0;
unsigned long OnTime1 = 300;
unsigned long OnTime2 = 300;
int ventiicon = LOW;

//**************************************************************
// Ab hier LCD menue fuehrung und taster
byte screen = 1;
const int screenPin = 4;  // Pin für Taster zum umschalten der LCD seite
int screenStatus = LOW;  // aktuelles Signal vom Eingangspin
byte screenGedrueckt = 0;  // abfragen ob Taster gedrückt wurde
unsigned long screenZeit = 0;  // Zeit beim drücken des Tasters

// Variablen für Starttag und bloomcounter
byte letztertag = 0;
byte letztermonat = 0;
byte starttag = 0;
byte startmonat = 0;
byte relay_switching = 0;
byte last_relay_state = 0;
byte bloom_counter = 0;

//****************************Variablen zum einstellen der RTC
byte setsekunde = 0;
byte setminute = 1;
byte setstunde = 1;
byte settag = 6;
byte settagderwoche = 1;
byte setmonat = 3;
byte setjahr = 16;

//****************************BME 280
int ltitemp;
double ltirlf;
int dsplrlf;
Adafruit_BME280 bme; // I2C
unsigned long vorhermillis = millis();
unsigned long vorhermillislti = millis();

//****************************Encoder
#define encoderPinA 2
#define encoderPinB 3
volatile unsigned int encoderPos = 0;  // Encoder counter
unsigned int lastReportedPos = 1;
static boolean rotating = false;
boolean A_set = false;
boolean B_set = false;
byte anaus = 0;
byte temp_bereich = 0;
byte rlf_bereich = 0;
byte zeitstellen = 0;

I2CSoilMoistureSensor bodensensor; // setze Var fuer Bodenfeuchtesensor (chirp)
//*******************************************************************************

DS3231 RTC;
int tempC;

//**************************** Bekanntmachung der Relais
const byte luft_relay = 9;  // luft_relay = LTI
const byte licht_relay_p = 7; // licht_relay = zur Steuerung des Hauptleuchtmittels
const byte lsr_relay_p = 6; // lsr_relay = zur Steuerung der LSR der Jungpflanzen
const byte ventilator = 5; // vetilator = zur steuerung des Relais Umluftventilators
const byte irrigation = 11; // wasser_relay = autobewaesserung

//**************************** GY-30 Luxmeter
void BH1750_Init(int address) {

  Wire.beginTransmission(address);
  Wire.write(0x10);  // 1 [lux] aufloesung
  Wire.endTransmission();
}

byte BH1750_Read(int address) {

  byte i = 0;
  Wire.beginTransmission(address);
  Wire.requestFrom(address, 2);
  while (Wire.available()) {
    buff[i] = Wire.read();
    i++;
  }
  Wire.endTransmission();
  return i;
}

char sendeInhalt = ' ';

//**************************** RTC funktionen

// Convert normal decimal numbers to binary coded decimal
byte decToBcd(byte val)
{
  return ( (val / 10 * 16) + (val % 10) );
}
// Convertiere binaeren dezimal code zu normalen dezimal nummern
byte bcdToDec(byte val)
{
  return ( (val / 16 * 10) + (val % 16) );
}

void readDS3231time(byte *second,
                    byte *minute,
                    byte *hour,
                    byte *dayOfWeek,
                    byte *dayOfMonth,
                    byte *month,
                    byte *year)
{
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0);        // setze DS3231 register pointer zu 00h
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 7);
  *second = bcdToDec(Wire.read() & 0x7f);
  *minute = bcdToDec(Wire.read());
  *hour = bcdToDec(Wire.read() & 0x3f);
  *dayOfWeek = bcdToDec(Wire.read());
  *dayOfMonth = bcdToDec(Wire.read());
  *month = bcdToDec(Wire.read());
  *year = bcdToDec(Wire.read());
}

void setDS3231time(byte second, byte minute, byte hour, byte dayOfWeek, byte
                   dayOfMonth, byte month, byte year)
{
  // sets time and date data to DS3231
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0); // set next input to start at the seconds register
  Wire.write(decToBcd(second)); // set seconds
  Wire.write(decToBcd(minute)); // set minutes
  Wire.write(decToBcd(hour)); // set hours
  Wire.write(decToBcd(dayOfWeek)); // set day of week (1=Sunday, 7=Saturday)
  Wire.write(decToBcd(dayOfMonth)); // set date (1 to 31)
  Wire.write(decToBcd(month)); // set month
  Wire.write(decToBcd(year)); // set year (0 to 99)
  Wire.endTransmission();
}

//****************************hier gehen die einzelnen Funktionen los

void displayTime() // anzeige der Zeit und Datum auf dem Display
{ if (hintergrund == 1) {
    byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
    // hole daten von DS3231
    readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);

    lcd.setCursor(0, 0);
    
    if (hour < 10)
      lcd.print("0");
 
    lcd.print(hour, DEC);
    
    lcd.print(":");
    
    if (minute < 10)
      lcd.print("0");
      
    lcd.print(minute, DEC);
    
    lcd.print(":");
    if (second < 10)
      lcd.print("0");

    lcd.print(second, DEC);
    lcd.print(" ");

    switch (dayOfWeek) {
      case 1:
        lcd.print("So");
        break;
      case 2:
        lcd.print("Mo");
        break;
      case 3:
        lcd.print("Di");
        break;
      case 4:
        lcd.print("Mi");
        break;
      case 5:
        lcd.print("Do");
        break;
      case 6:
        lcd.print("Fr");
        break;
      case 7:
        lcd.print("Sa");
        break;
    }

    lcd.print(" ");
    if (dayOfMonth < 10)
    {
      lcd.print("0");
    }
    lcd.print(dayOfMonth, DEC);
    lcd.print(" ");
    switch (month) {
      case 1:
        lcd.print("Jan");
        break;
      case 2:
        lcd.print("Feb");
        break;
      case 3:
        lcd.print("Mar");
        break;
      case 4:
        lcd.print("Apr");
        break;
      case 5:
        lcd.print("Mai");
        break;
      case 6:
        lcd.print("Jun");
        break;
      case 7:
        lcd.print("Jul");
        break;
      case 8:
        lcd.print("Aug");
        break;
      case 9:
        lcd.print("Sep");
        break;
      case 10:
        lcd.print("Okt");
        break;
      case 11:
        lcd.print("Nov");
        break;
      case 12:
        lcd.print("Dez");
        break;
    }
  }
}

void bme280() // Anzeige der Temp und RLF auf dem Display
{ if (hintergrund == 1) {
    if (millis() - vorhermillis > 3000) {
      vorhermillis = millis();
      ltitemp = bme.readTemperature();
      dsplrlf = bme.readHumidity();

      // DISPLAY DATA
      lcd.setCursor(0, 1);           // setze curserposition
      lcd.write(3);    // zeichne thermometer auf dem Display, siehe auch abschnitt Custom Caracter bzw. void setup
      lcd.print(F(" "));
      lcd.print(ltitemp);
      lcd.print((char)223);
      lcd.print(F("C "));
      lcd.print(F(" "));
      lcd.write(4);    // zeichne Wassertropfen auf dem Display, siehe auch abschnitt Custom Caracter bzw. void setup
      lcd.print(F(" "));
      lcd.print(dsplrlf);
      lcd.print(F("%"));
      lcd.print(F("   "));
    }
  }
}

void DS3231temp()  // hole und zeige auf dem Display die Case Temperatur der RTC
{ if (hintergrund == 1) {
    tempC = RTC.getTemperature();

    lcd.setCursor(0, 3);
    lcd.print(F("Case:"));
    lcd.print(tempC);
    lcd.print((char)223);
    lcd.print(F("C"));
  }
}

void LTI() // die Funtion des Rohrventilators
{ byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
  // hole daten von DS3231
  readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);
  if (millis() - vorhermillislti > 10000)
  {
    vorhermillislti = millis();
    ltitemp = bme.readTemperature();
    ltirlf = bme.readHumidity();
  }
  // Pruefe im LSR oder Grow Modus Temperatur und RLF ist die Temp unter 24 Grad C oder unter RLF unter 55%
  // bleibt Stufentrafo gedimmt (z.B. 80V)
  // ist Temp oder gleich oder höher wird auf hoechste stufe (z.B. 190V) geschaltet.
  if (setings_a.lichtmodus == LSR)
  { if ( ltitemp < setings_a.lsr_temp) {
      digitalWrite(luft_relay, LOW);
    }
    if (ltirlf < setings_a.lsr_rlf) {
      digitalWrite(luft_relay, LOW);
    }
    else {
      digitalWrite(luft_relay, HIGH);
    }
  }
  // Pruefe im LSR oder Grow Modus Temperatur und RLF ist die Temp unter 24 Grad C oder unter RLF unter 55%
  // bleibt Stufentrafo gedimmt (z.B. 80V)
  // ist Temp oder gleich oder höher wird auf hoechste stufe (z.B. 190V) geschaltet.
  if (setings_a.lichtmodus == GROW)
  { if (ltitemp < setings_a.grow_temp) {
      digitalWrite(luft_relay, LOW);
    }
    if (ltirlf < setings_a.grow_rlf) {
      digitalWrite(luft_relay, LOW);
    }
    else {
      digitalWrite(luft_relay, HIGH);
    }
  }

  // Pruefe im Uebergangsmodus Grow>Bloom Temperatur und RLF
  if (setings_a.lichtmodus == BLOOM)
  { if (ltitemp < setings_a.grow_temp) {
      digitalWrite(luft_relay, LOW);
    }
    if (ltirlf < setings_a.grow_rlf) {
      digitalWrite(luft_relay, LOW);
    }
    else {
      digitalWrite(luft_relay, HIGH);
    }
  }

  // Pruefe im Bloom Modus Temperatur und RLF ist die Temp unter 22 Grad C oder unter RLF unter 40%
  // bleibt Stufentrafo gedimmt (z.B. 80V)
  // ist Temp oder RLF gleich oder höher wird auf hoechste stufe (z.B. 190V) geschaltet.
  if (setings_a.lichtmodus == 3)
  { if (ltitemp < setings_a.bloom_temp) {
      digitalWrite(luft_relay, LOW);
    }
    if (ltirlf < setings_a.bloom_rlf) {
      digitalWrite(luft_relay, LOW);
    }
    else {
      digitalWrite(luft_relay, HIGH);
    }
  }
}
void gy30() // Luxmeter
{ if (hintergrund == 1) {
    float valf = 0;
    if (BH1750_Read(BH1750_address) == 2) {

      {
        valf = ((buff[0] << 8) | buff[1]);

        lcd.setCursor(0, 2);
        lcd.print(F("LUX: "));
        if (valf < 1000) lcd.print(" ");
        if (valf < 100) lcd.print(" ");
        if (valf < 10) lcd.print(" ");

        lcd.print(valf, 2);
      }
    }
  }
}

void displaybeleuchtung() // hier wird das Display ein und ausgeschaltet
{ // lese ststus des display buttons und schalte wenn betätigt fuer 30 sek. an
  buttonState = digitalRead(buttonPin);

  // Wenn der Wechseltaster gedrückt ist...
  if (buttonState == HIGH)
  {
    buttonZeit = millis();  // aktualisiere tasterZeit
    buttonGedrueckt = 1;  // speichert, dass Taster gedrückt wurde
  }

  // Wenn Taster gedrückt wurde die gewählte entprellZeit vergangen ist soll Lichtmodi und gespeichert werden ...
  if ((millis() - buttonZeit > entprellZeit) && buttonGedrueckt == 1)
  {
    hintergrund++;  // LCD Seite wird um +1 erhöht
    buttonGedrueckt = 0;  // setzt gedrückten Taster zurück
  }
  {
    if (hintergrund == 1) // display ist an

    {
      lcd.display();
      lcd.setBacklight(255);
    }

    else if (hintergrund == 2) // display ist ganz aus
    { lcd.setBacklight(0);
      lcd.noDisplay();
    }

    else if (hintergrund == 3) // setzt die Funtion wieder auf anfang
    {
      hintergrund = 1;
    }
  }
}

// Custom Caracter
byte moon[8] = {
  0b00000,
  0b01110,
  0b10101,
  0b11111,
  0b10001,
  0b01110,
  0b00000,
  0b00000
};

byte sun[8] = {
  0b00000,
  0b00100,
  0b10101,
  0b01110,
  0b01110,
  0b10101,
  0b00100,
  0b00000
};

byte thermo[8] = {
  0b00100,
  0b01010,
  0b01010,
  0b01110,
  0b01110,
  0b11111,
  0b11111,
  0b01110
};

byte rlf[8] = {
  0b00100,
  0b00100,
  0b01110,
  0b01110,
  0b11111,
  0b11001,
  0b11111,
  0b01110
};

byte water_on[8] = {
  0b11100,
  0b01000,
  0b11110,
  0b11111,
  0b00011,
  0b00011,
  0b00000,
  0b00011
};

byte water_off[8] = {
  0b11100,
  0b01000,
  0b11100,
  0b11110,
  0b00011,
  0b00011,
  0b00000,
  0b00000
};

byte venti_I[8] = {
  0b00100,
  0b01010,
  0b00000,
  0b00100,
  0b10001,
  0b11011,
  0b00000,
  0b00000
};

byte venti_II[8] = {
  0b00000,
  0b11011,
  0b10001,
  0b00100,
  0b00000,
  0b01010,
  0b00100,
  0b00000
};

void tagec()
// bluete Tagecounter
{ relay_switching = digitalRead(licht_relay_p);
  if (relay_switching != last_relay_state) {
    if (relay_switching == LOW)
      setings_a.bloom_counter++;
    //EEPROM.update(addr2, bloom_counter);
  }
  last_relay_state = relay_switching;
}

void doEncoderA()
{
  if ( rotating ) delay (1);  // debounce für Encoder Pin A
  if ( digitalRead(encoderPinA) != A_set ) { // debounce erneut
    A_set = !A_set;
    // stelle counter + 1 im Uhrzeigersinn
    if ( A_set && !B_set )
      encoderPos += 1;
    rotating = false;
  }
}


void doEncoderB() {
  if ( rotating ) delay (1);
  if ( digitalRead(encoderPinB) != B_set ) {
    B_set = !B_set;
    //  stelle counter - 1 gegen Uhrzeigersinn
    if ( B_set && !A_set )
      encoderPos -= 1;
    rotating = false;
  }
}

// Wassernfunktion
void startzeitwassern() {
  digitalWrite(irrigation, LOW);
}

void endzeitwassern() {
  digitalWrite(irrigation, HIGH);
}

void readEEPROM(){
  /* Lese EEPROM structure in speicher*/
  EEPROM.get(0, setings_a);

  if( setings_a.MAGIC_NUMBER != MAGIC_NUMBER ){ // Vergleiche Magic number wenn ungleich schreibe EEPROM mit defaults neu.

    setings_a = setings_b; // settings_a ist b (defaults)
    EEPROM.put(0, setings_b); // Schreibe settings_b, enthält default einstellungen.

  } else {

    setings_b = setings_a; // settings_a ist valide.

  }  
}

//**************************** das Setup
void setup() {

  Serial.begin(9600);

  readEEPROM();

  Wire.begin();
  lcd.begin(); // stelle LCD groesse ein
  bme.begin();

  // Splashscreen
  lcd.setCursor(0, 0);
  lcd.print(F("..:: Gro-Matic ::.."));
  lcd.setCursor(0, 2);
  lcd.print(F("  BME-280 Edition"));
  lcd.setCursor(0, 3);
  lcd.print(F(" V. 0.9.9.9 by zrox"));
  delay (3000);
  lcd.clear();

  BH1750_Init(BH1750_address);
  delay(500);

  byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
  // hole daten von DS3231
  readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);

  letztertag = (dayOfMonth);
  letztermonat = (month);
  setTime(hour, minute, second, dayOfMonth, month, year);

  digitalWrite(licht_relay_p, HIGH);  // alle Relais Pins beim Start auf HIGH setzen und damit ausschalten.
  digitalWrite(lsr_relay_p, HIGH);
  digitalWrite(luft_relay, HIGH);
  digitalWrite(ventilator, HIGH);
  digitalWrite(irrigation, HIGH);
  pinMode(luft_relay,  OUTPUT);  // alle Relais Pins als ausgang setzen
  pinMode(licht_relay_p,  OUTPUT);
  pinMode(lsr_relay_p,  OUTPUT);
  pinMode(ventilator, OUTPUT);
  pinMode(irrigation, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);  // den backlight Taster als Input setzen
  pinMode(wechslertPin, INPUT_PULLUP);  // Modus-Taster Pin wird als Eingang gesetzt
  pinMode(screenPin, INPUT_PULLUP);  // Modus-Taster Pin wird als Eingang gesetzt
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT); ;
  digitalWrite(encoderPinA, HIGH);
  digitalWrite(encoderPinB, HIGH);
  attachInterrupt(0, doEncoderA, CHANGE); // Encoder pin an interrupt 0 (pin 2)
  attachInterrupt(1, doEncoderB, CHANGE); // Encoder pin an interrupt 1 (pin 3)

  // erstelle die Custom character
  lcd.createChar(1, moon);
  lcd.createChar(2, sun);
  lcd.createChar(3, thermo);
  lcd.createChar(4, rlf);
  lcd.createChar(5, water_on);
  lcd.createChar(6, water_off);
  lcd.createChar(7, venti_I);
  lcd.createChar(8, venti_II);

//  Alarm.alarmRepeat(startwasser, startwassermin, 0, startzeitwassern); // 8:30am every day
//  Alarm.alarmRepeat(startwasser, auswasser, sekauswasser, endzeitwassern); // 5:45pm every day

}

//**************************** der Loop
void loop()
{
  //********************************************************************
  LTI();  // ruft die einfache LTI steuerung auf und prueft Temp und RLF und schaltet den Stufentrafo zwischen zwei Stufen.
  displaybeleuchtung();
  ltitemp = bme.readTemperature();
  dsplrlf = bme.readHumidity();
  Alarm.delay(0);
  //********************************************************************
  rotating = true;  // reset the debouncer

  if (lastReportedPos != encoderPos)
  {

    lastReportedPos = encoderPos;
  }

  // ab hier Taster des Encoders
  wechslertStatus = digitalRead(wechslertPin);

  // Wenn der Wechseltaster gedrückt ist...
  if (wechslertStatus == HIGH)
  {
    wechslertZeit = millis();  // aktualisiere tasterZeit
    wechslertGedrueckt = 1;  // speichert, dass Taster gedrückt wurde
  }

  //********************************************************************
  // Prüfe den Starttag und Monat die beim Start des Arduino gesetzt werden und Speichere wenn noetig oder,
  // wenn noch kein Starttag ins Eprom geschrieben wurde ab.
  // Ab hier die Tagezählerfunktion

/* WTF

  if (starttag == 0 && startmonat == 0)
  { starttag = (letztertag);
    startmonat = (letztermonat);
    EEPROM.update (addr4, starttag);
    EEPROM.update (addr5, startmonat);
  }
  else
  { letztertag = EEPROM.read(addr4);
    letztermonat = EEPROM.read(addr5);
  }

*/

  byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
  // hole daten von DS3231
  readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);


  //***********************************************

  if (setings_a.lichtmodus == 0)
  {
    byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
    // hole daten von DS3231
    readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);


    if ((hour >= setings_a.lsr_an) && (hour < setings_a.lsr_aus))
    { digitalWrite(lsr_relay_p, LOW); //schaltet lsr um 5 Uhr an und um 22:59:59 Uhr aus
      digitalWrite(licht_relay_p, HIGH); //schaltet ndl Relais aus sollten sie noch an sein

      // Umluftventilator alle 15 minuten einschalten wenn licht an
      if ( (minute >= 15 && minute <= 29 ) || (minute >= 45 && minute <= 59))
      {
        digitalWrite(ventilator, LOW);
      }
      else
      {
        digitalWrite(ventilator, HIGH);
      }

    }

    else
    { digitalWrite(lsr_relay_p, HIGH);
      if ( (hour >= setings_a.grow_licht_aus) & (hour < setings_a.grow_licht_an) || (hour >= setings_a.bloom_licht_aus) & (hour <= setings_a.bloom_licht_an))
        digitalWrite(licht_relay_p, HIGH);  //schaltet lsr Relais aus sollten sie noch an sein
      if ((minute >= 15) && (minute <= 19)) // schaltet Ventilator im Nachtmodus 1 x jede Stunde fuer 5 Min. an
      {
        digitalWrite(ventilator, LOW);
      }
      else
      {
        digitalWrite(ventilator, HIGH);
      }
    }

  }

  else if (setings_a.lichtmodus == 1)
  {
    byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
    // hole daten von DS3231
    readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);


    if ((hour >= setings_a.grow_licht_an) && (hour < setings_a.grow_licht_aus))
    { digitalWrite(licht_relay_p, LOW); //schaltet ndl im Grow modus 18h licht um 5 Uhr an und um 22:59:59 Uhr aus
      digitalWrite(lsr_relay_p, HIGH);  //schaltet lsr Relais aus sollten sie noch an sein

      // Umluftventilator alle 15 minuten einschalten wenn licht an
      if ( (minute >= 15 && minute <= 29 ) || (minute >= 45 && minute <= 59) )
      {
        digitalWrite(ventilator, LOW);
      }
      else
      {
        digitalWrite(ventilator, HIGH);
      }

    }

    else
    { digitalWrite(licht_relay_p, HIGH);
      if ( (hour >= setings_a.lsr_aus) & (hour < setings_a.lsr_an) )
        digitalWrite(lsr_relay_p, HIGH);  //schaltet lsr Relais aus sollten sie noch an sein
      if ((minute >= 15) && (minute <= 19)) // schaltet Ventilator im Nachtmodus 1 x jede Stunde fuer 5 Min. an
      {
        digitalWrite(ventilator, LOW);
      }
      else
      {
        digitalWrite(ventilator, HIGH);
      }
    }

  }

  else if (setings_a.lichtmodus == 2)
  {
    byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
    // hole daten von DS3231
    readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);

    if ((hour >= setings_a.grow_licht_an) && (hour < setings_a.grow_licht_aus))
    { digitalWrite(licht_relay_p, LOW); //schaltet ndl im Grow modus 18h licht um 5 Uhr an und um 22:59:59 Uhr aus
      digitalWrite(lsr_relay_p, HIGH);  //schaltet lsr Relais aus sollten sie noch an sein
      // Umluftventilator alle 15 minuten einschalten wenn licht an
      if ( (minute >= 15 &&  minute <= 29) || (minute >= 45 && minute <= 59) )
      {
        digitalWrite(ventilator, LOW);
      }
      else
      {
        digitalWrite(ventilator, HIGH);
      }
    }

    else
    { setings_a.lichtmodus = 3;
      speichern = 1;
    }
  }

  else if (setings_a.lichtmodus == 3)
  {
    byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
    // hole daten von DS3231
    readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);
    tagec();

    if ((hour >= setings_a.bloom_licht_an) && (hour < setings_a.bloom_licht_aus))
    { digitalWrite(licht_relay_p, LOW); //schaltet ndl im Bloom modus 12h licht um 5 Uhr an und um 16:59:59 Uhr aus
      digitalWrite(lsr_relay_p, HIGH);  //schaltet lsr Relais aus sollten sie noch an sein

      // Umluftventilator alle 15 minuten einschalten wenn licht an
      if ( (minute >= 15 && minute <= 29 ) || (minute >= 45 && minute <= 59) )
      {
        digitalWrite(ventilator, LOW);
      }
      else
      {
        digitalWrite(ventilator, HIGH);
      }


    }

    else
    { digitalWrite(licht_relay_p, HIGH);
      if ( (hour >= setings_a.grow_licht_aus) & (hour < setings_a.grow_licht_an) || (hour >= setings_a.lsr_aus) & (hour < setings_a.lsr_an) )
        digitalWrite(lsr_relay_p, HIGH);  //schaltet lsr Relais aus sollten sie noch an sein
      if ((minute >= 15) && (minute <= 19)) // schaltet Ventilator im Nachtmodus 1 x jede Stunde fuer 5 Min. an
      {
        digitalWrite(ventilator, LOW);
      }
      else
      {
        digitalWrite(ventilator, HIGH);
      }
    }

  }

  // Lichtmodus Ende
  //*********************************************************************************************

  //**************************** Autobewaesserung



  if (setings_a.autowasser == 1)
  {

  }

  // Autobewaesserung Ende
  //*********************************************************************************************

  // ab hier Taster abfrage fuer LCD menue
  screenStatus = digitalRead(screenPin);

  // Wenn der Wechseltaster gedrückt ist...
  if (screenStatus == HIGH)
  {
    screenZeit = millis();  // aktualisiere tasterZeit
    screenGedrueckt = 1;  // speichert, dass Taster gedrückt wurde
  }

  // Wenn Taster gedrückt wurde die gewählte entprellZeit vergangen ist soll Lichtmodi und gespeichert werden ...
  if ((millis() - screenZeit > entprellZeit) && screenGedrueckt == 1)
  {
    screen++;  // LCD Seite wird um +1 erhöht
    screenGedrueckt = 0;  // setzt gedrückten Taster zurück
    lcdbereinigen = 1;
  }
  if (lcdbereinigen == 1) { // Austosave des Lichtmodis starten
    lcd.clear();
    lcdbereinigen = 0;
  }



  //******************************************************************

  if (screen == 1)
  {
    // Rufe funktionen für Seite 1 auf
    displayTime();        // zeige die RTC Daten auf dem LCD display,
    bme280();          // zeige temp und rlf auf dem LCD display,
    DS3231temp();  // prüfe gehaeuse temp und gib sie auf dem display aus
    gy30();  // Luxmeter

    //GY-30
    byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
    // hole daten von DS3231
    readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);
    if (second >= 0) {
      void BH1750_Init(int address);
    }

    // Wenn Taster gedrückt wurde die gewählte entprellZeit vergangen ist soll Lichtmodi und gespeichert werden ...
    if ((millis() - wechslertZeit > entprellZeit) && wechslertGedrueckt == 1)
    {
      setings_a.lichtmodus++;  // lichtmodus wird um +1 erhöht
//      EEPROM.update(addr, lichtmodus);
      wechslertGedrueckt = 0;  // setzt gedrückten Taster zurück
    }

    //*********Ventilator Icon

    unsigned long currentMillis = millis();
    if (digitalRead(ventilator) == LOW) {
      if ((ventiicon == HIGH) && (currentMillis - previousMillis >= OnTime1)) {
        ventiicon = LOW;
        previousMillis = currentMillis;
        lcd.setCursor(17, 2);
        lcd.write(7);
      }

      if ((ventiicon == LOW) && (currentMillis - previousMillis >= OnTime2)) {
        ventiicon = HIGH;
        previousMillis = currentMillis;
        lcd.setCursor(17, 2);
        lcd.write(8);
      }
    }
    if (digitalRead(ventilator) == HIGH) {
      lcd.setCursor(17, 2);
      lcd.print(F(" "));
    }
    //*************************************Programm-Modis**************************************

    // Wenn Lichtmodus 0 ist, starte im LSR modus
    if (setings_a.lichtmodus == 0)
    {
      lcd.setCursor(10, 3);
      lcd.print(F("  LSR Mode"));
      relay_lsr_switching = digitalRead(lsr_relay_p);
      if (relay_lsr_switching == LOW) {
        lcd.setCursor(19, 2);
        lcd.write(2);
      }
      if (relay_lsr_switching == HIGH)
      { lcd.setCursor(19, 2);
        lcd.write(1);
      }
    }

    else if (setings_a.lichtmodus == 1)
    {
      lcd.setCursor(10, 3);
      lcd.print(F(" Grow Mode"));
      relay_grow_switching = digitalRead(licht_relay_p);
      if (relay_grow_switching == LOW) {
        lcd.setCursor(19, 2);
        lcd.write(2);
      }
      if (relay_grow_switching == HIGH) {
        lcd.setCursor(19, 2);
        lcd.write(1);
      }
    }

    else if (setings_a.lichtmodus == 2)
    {
      lcd.setCursor(10, 3);
      lcd.print(F("Grow>Bloom"));
      relay_bloom_switching = digitalRead(licht_relay_p);
      if (relay_bloom_switching == LOW) {
        lcd.setCursor(19, 2);
        lcd.write(2);
      }
      if (relay_bloom_switching == HIGH) {
        lcd.setCursor(19, 2);
        lcd.write(1);
      }
    }
    else if (setings_a.lichtmodus == 3)
    {
      lcd.setCursor(10, 3);
      lcd.print(F("Bloom Mode"));
      relay_bloom_switching = digitalRead(licht_relay_p);
      if (relay_bloom_switching == LOW) {
        lcd.setCursor(19, 2);
        lcd.write(2);
      }
      if (relay_bloom_switching == HIGH) {
        lcd.setCursor(19, 2);
        lcd.write(1);
      }
    }


    // Wenn der Lichtmodus auf 3 sprngt, setzte ihn wieder zurück auf 0 um von vorne zu beginnen
    else
    {
      setings_a.lichtmodus = 0;
    }
    // Lichtmodus Ende

  }
  else if (screen == 2)
  {

    // Wenn Taster gedrückt wurde die gewählte entprellZeit vergangen ist soll Tagecounter gelöscht werden ...
    if ((millis() - wechslertZeit > entprellZeit) && wechslertGedrueckt == 1)
    {
      wechslertGedrueckt = 0;      // setzt gedrückten Taster zurück
      tage_reset = 1;              // beginne Eprom Reset und starte neu
    }

    if (tage_reset == 1) {
      for (int i = 0; i < 512; i++)
//        EEPROM.write(i, 0);
      asm volatile ("jmp 0");
    }
    lcd.setCursor(0, 0);
    lcd.print(F("Starttag am: "));
    if (letztertag < 10)
    {
      lcd.print(F("0"));
    }
    lcd.print(letztertag);
    lcd.print(F("."));
    if (letztermonat < 10)
    {
      lcd.print(F("0"));
    }
    lcd.print(letztermonat);
    lcd.setCursor(0, 1);
    lcd.print(F("Bl"));
    lcd.print((char)0xF5);
    lcd.print(F("tetag:"));
    lcd.print(setings_a.bloom_counter);
    lcd.setCursor(0, 2);
    lcd.print(F("dr"));
    lcd.print((char)0xF5);
    lcd.print(F("cke Enc.taste zum"));
    lcd.setCursor(0, 3);
    lcd.print(F("Speicher l"));
    lcd.print((char)0xEF);
    lcd.print(F("schen."));

  }

  else if (screen == 3)
  {
    // Wenn Taster gedrückt wurde die gewählte entprellZeit vergangen ist soll Lichtmodi und gespeichert werden ...
    if ((millis() - wechslertZeit > entprellZeit) && wechslertGedrueckt == 1)
    {
      setings_a.autowasser++;
//      EEPROM.update(addr1, autowasser);
      wechslertGedrueckt = 0;  // setzt gedrückten Taster zurück
    }

    //*************************************Programm-Modis**************************************

    // Wenn Lichtmodus 0 ist, starte im LSR modus
    if (setings_a.autowasser == true) {
      
      lcd.setCursor(0, 2);
      lcd.print(F("Autobew"));
      lcd.print((char)0xE1);
      lcd.print(F("sserung:"));
      lcd.setCursor(0, 3);
      lcd.write(6);
      lcd.print(F(" aus"));
      
    } else if( setings_a.autowasser == false) {
      
      lcd.setCursor(0, 2);
      lcd.print(F("Autobew"));
      lcd.print((char)0xE1);
      lcd.print(F("sserung:"));
      lcd.setCursor(0, 3);
      lcd.write(5);
      lcd.print(F(" an "));
      
    } else {
      
      setings_a.autowasser = true;
      
    }


    // Autobewaesserung Ende

    lcd.setCursor(0, 0);
    lcd.print(F("Boden "));
    lcd.write(4);
    lcd.print(F(" "));
    lcd.print(bodensensor.getCapacitance()); //lese bodensensor
    lcd.setCursor(0, 1);
    lcd.print(F("Boden "));
    lcd.write(3);
    lcd.print(F(" "));
    lcd.print(bodensensor.getTemperature() / (float)10); //lese temperatur register des bodensensors
    lcd.print((char)223);
    lcd.print(F("C "));
  }
  else if (screen == 4)
  {

    lcd.setCursor(0, 0);
    lcd.print(F("Schaltzeiten Licht"));
    lcd.setCursor(0, 1);
    lcd.print(F("LSR:  "));
    lcd.print(setings_a.lsr_an);
    lcd.print(F(":00-"));
    lcd.print(setings_a.lsr_aus);
    lcd.print(F(":00 Uhr"));
    lcd.setCursor(0, 2);
    lcd.print(F("Grow: "));
    lcd.print(setings_a.grow_licht_an);
    lcd.print(F(":00-"));
    lcd.print(setings_a.grow_licht_aus);
    lcd.print(F(":00 Uhr"));
    lcd.setCursor(0, 3);
    lcd.print(F("Bloom:"));
    lcd.print(setings_a.bloom_licht_an);
    lcd.print(F(":00-"));
    lcd.print(setings_a.bloom_licht_aus);
    lcd.print(F(":00 Uhr"));
    if ((millis() - wechslertZeit > entprellZeit) && wechslertGedrueckt == 1)
    {
      lcd.clear();
      delay (50);
      screen = 7;
      wechslertGedrueckt = 0;  // setzt gedrückten Taster zurück
    }
  }
  else if (screen == 5)
  {

    lcd.setCursor(0, 0);
    lcd.print(F("eingest. LTI Werte"));
    lcd.setCursor(0, 1);
    lcd.print(F("LSR:  "));
    lcd.print(setings_a.lsr_temp);
    lcd.print((char)223);
    lcd.print(F("C, :"));
    lcd.print(setings_a.lsr_rlf);
    lcd.print(F(" %"));
    lcd.setCursor(0, 2);
    lcd.print(F("Grow: "));
    lcd.print(setings_a.grow_temp);
    lcd.print((char)223);
    lcd.print(F("C, :"));
    lcd.print(setings_a.grow_rlf);
    lcd.print(F(" %"));
    lcd.setCursor(0, 3);
    lcd.print(F("Bloom:"));
    lcd.print(setings_a.bloom_temp);
    lcd.print((char)223);
    lcd.print(F("C, :"));
    lcd.print(setings_a.bloom_rlf);
    lcd.print(F(" %"));
    if ((millis() - wechslertZeit > entprellZeit) && wechslertGedrueckt == 1)
    {
      lcd.clear();
      delay (200);
      temp_bereich = 0;
      rlf_bereich = 0;
      screen = 8;
      wechslertGedrueckt = 0;  // setzt gedrückten Taster zurück

    }
  }
  else if (screen == 6)
  {

    screen = 10; // geht wieder auf seite 1 zurück
  }

  else if (screen == 7)
  {

    if (encoderPos == 24) {

      encoderPos = 0;
    }
    if (encoderPos >= 24) {
      lcd.clear();
      encoderPos = 23;
    }

    if (anaus == 0) {

      lcd.setCursor(0, 0);
      lcd.print(F("LSR an:    "));
      if (encoderPos < 10)
      {
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

      if ((millis() - wechslertZeit > entprellZeit) && wechslertGedrueckt == 1)
      {
        setings_a.lsr_an = encoderPos;
        wechslertGedrueckt = 0;  // setzt gedrückten Taster zurück
//        EEPROM.update (addr6, lsr_an);
        lcd.clear();
        anaus++;
      }
    }

    if (anaus == 1) {
      lcd.setCursor(0, 0);
      lcd.print(F("LSR aus:   "));
      if (encoderPos < 10)
      {
        lcd.print("0");
      }
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
      if ((millis() - wechslertZeit > entprellZeit) && wechslertGedrueckt == 1)
      {
        if (encoderPos == 0) {
          encoderPos = 23;
          setings_a.lsr_aus = encoderPos;
        }
        else {
          setings_a.lsr_aus = encoderPos;
        }
        wechslertGedrueckt = 0;  // setzt gedrückten Taster zurück
//        EEPROM.update (addr7, lsr_aus);
        lcd.clear();
        anaus++;
      }
    }

    if (anaus == 2) {
      lcd.setCursor(0, 0);
      lcd.print(F("Grow an:   "));
      if (encoderPos < 10)
      {
        lcd.print("0");
      }
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
      if ((millis() - wechslertZeit > entprellZeit) && wechslertGedrueckt == 1)
      {
        setings_a.grow_licht_an = encoderPos;
        wechslertGedrueckt = 0;  // setzt gedrückten Taster zurück
//        EEPROM.update (addr8, grow_licht_an);
        lcd.clear();
        anaus++;
      }
    }
    if (anaus == 3) {
      lcd.setCursor(0, 0);
      lcd.print(F("Grow aus:  "));
      if (encoderPos < 10)
      {
        lcd.print("0");
      }
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
      if ((millis() - wechslertZeit > entprellZeit) && wechslertGedrueckt == 1)
      {
        if (encoderPos == 0) {
          encoderPos = 23;
          setings_a.grow_licht_aus = encoderPos;
        }
        else {
          setings_a.grow_licht_aus = encoderPos;
        }
        wechslertGedrueckt = 0;  // setzt gedrückten Taster zurück
//        EEPROM.update (addr9, grow_licht_aus);
        lcd.clear();
        anaus++;
      }
    }
    if (anaus == 4) {
      lcd.setCursor(0, 0);
      lcd.print(F("Bloom an:  "));
      if (encoderPos < 10)
      {
        lcd.print("0");
      }
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
      if ((millis() - wechslertZeit > entprellZeit) && wechslertGedrueckt == 1)
      {
        setings_a.bloom_licht_an = encoderPos;
        wechslertGedrueckt = 0;  // setzt gedrückten Taster zurück
//        EEPROM.update (addr10, bloom_licht_an);
        lcd.clear();
        anaus++;
      }

    }
    if (anaus == 5) {
      lcd.setCursor(0, 0);
      lcd.print(F("Bloom aus: "));
      if (encoderPos < 10)
      {
        lcd.print("0");
      }
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
      if ((millis() - wechslertZeit > entprellZeit) && wechslertGedrueckt == 1)
      {
        if (encoderPos == 0) {
          encoderPos = 23;
          setings_a.bloom_licht_aus = encoderPos;
        }
        else {
          setings_a.bloom_licht_aus = encoderPos;
        }
        wechslertGedrueckt = 0;  // setzt gedrückten Taster zurück
//        EEPROM.update (addr11, bloom_licht_aus);
        lcd.clear();
        delay (100);
        anaus = 0;
        screen = 4;
      }
    }


  }
  else if (screen == 8)
  {


    if (encoderPos >= 29) {
      encoderPos = 17;
    }
    else if (encoderPos <= 16) {
      lcd.clear();
      encoderPos = 28;
    }

    if (temp_bereich == 0) {
      lcd.setCursor(0, 0);
      lcd.print(F("LSR max. "));
      lcd.write(3);
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
      if ((millis() - wechslertZeit > entprellZeit) && wechslertGedrueckt == 1)
      {
        setings_a.lsr_temp = encoderPos;
        wechslertGedrueckt = 0;  // setzt gedrückten Taster zurück
//        EEPROM.update (addr12, lsr_temp);
        lcd.clear();
        temp_bereich++;
      }
    }

    if (temp_bereich == 1) {
      lcd.setCursor(0, 0);
      lcd.print(F("Grow max. "));
      lcd.write(3);
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
      if ((millis() - wechslertZeit > entprellZeit) && wechslertGedrueckt == 1)
      {
        setings_a.grow_temp = encoderPos;
        wechslertGedrueckt = 0;  // setzt gedrückten Taster zurück
//        EEPROM.update (addr13, grow_temp);
        lcd.clear();
        temp_bereich++;
      }
    }

    if (temp_bereich == 2) {
      lcd.setCursor(0, 0);
      lcd.print(F("Bloom max. "));
      lcd.write(3);
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
      if ((millis() - wechslertZeit > entprellZeit) && wechslertGedrueckt == 1)
      {
        setings_a.bloom_temp = encoderPos;
        wechslertGedrueckt = 0;  // setzt gedrückten Taster zurück
//        EEPROM.update (addr14, bloom_temp);
        lcd.clear();
        screen = 9;
      }
    }

  }
  else if (screen == 9)
  {

    if (encoderPos >= 71) {
      encoderPos = 15;
    }
    else if (encoderPos <= 14) {
      lcd.clear();
      encoderPos = 70;
    }

    if (rlf_bereich == 0) {
      lcd.setCursor(0, 0);
      lcd.print(F("LSR max. "));
      lcd.write(4);
      lcd.print(F(" : "));
      lcd.print(encoderPos);
      lcd.print(F(" %"));
      lcd.setCursor(0, 1);
      lcd.print(F("optimaler bereich"));
      lcd.setCursor(0, 2);
      lcd.print(F("zwischen 55 % - 60 %"));
      if ((millis() - wechslertZeit > entprellZeit) && wechslertGedrueckt == 1)
      {
        setings_a.lsr_rlf = (double) encoderPos;
        wechslertGedrueckt = 0;  // setzt gedrückten Taster zurück
//        EEPROM.update (addr15, lsr_rlf);
        lcd.clear();
        rlf_bereich++;
      }
    }
    if (rlf_bereich == 1) {
      lcd.setCursor(0, 0);
      lcd.print(F("Grow max. "));
      lcd.write(4);
      lcd.print(F(" : "));
      lcd.print(encoderPos);
      lcd.print(F(" %"));
      lcd.setCursor(0, 1);
      lcd.print(F("optimaler bereich"));
      lcd.setCursor(0, 2);
      lcd.print(F("zwischen 50 % - 55 %"));
      if ((millis() - wechslertZeit > entprellZeit) && wechslertGedrueckt == 1)
      {
        setings_a.grow_rlf = (float) encoderPos;
        wechslertGedrueckt = 0;  // setzt gedrückten Taster zurück
//        EEPROM.update (addr16, grow_rlf);
        lcd.clear();
        rlf_bereich++;
      }
    }
    if (rlf_bereich == 2) {
      lcd.setCursor(0, 0);
      lcd.print(F("Bloom max. "));
      lcd.write(4);
      lcd.print(F(" : "));
      lcd.print(encoderPos);
      lcd.print(F(" %"));
      lcd.setCursor(0, 1);
      lcd.print(F("optimal nicht "));
      lcd.print((char)0xF5);
      lcd.print(F("ber"));
      lcd.setCursor(0, 2);
      lcd.print(F("40 % RLF"));
      if ((millis() - wechslertZeit > entprellZeit) && wechslertGedrueckt == 1)
      {
        setings_a.bloom_rlf = (double) encoderPos;
        wechslertGedrueckt = 0;  // setzt gedrückten Taster zurück
//        EEPROM.update (addr17, bloom_rlf);
        lcd.clear();
        rlf_bereich++;
      }
    }
    if (rlf_bereich == 3) {
      temp_bereich = 0;
      rlf_bereich = 0;
      screen = 5;
    }

  }

  else if (screen == 10) {

    if (encoderPos > 1) {
      encoderPos = 1;
    }
    else if (encoderPos < 0) {
      lcd.clear();
      encoderPos = 0;
    }
    lcd.setCursor(0, 0);
    lcd.print(F("RTC einstellen?"));
    lcd.setCursor(0, 1);
    switch (encoderPos) {
      case 0:
        lcd.print("nein");
        break;
      case 1:
        lcd.print("ja  ");
        break;
    }
    if ((millis() - wechslertZeit > entprellZeit) && wechslertGedrueckt == 1)
    {
      wechslertGedrueckt = 0;
      if (encoderPos == 0) {
        screen = 13;
      }
      if (encoderPos == 1) {
        screen = 12;
      }
    }
  }
  else if (screen == 11) {
    screen = 1;
  }



  else if (screen == 12)
  {

    if (zeitstellen == 0) {
      if (encoderPos >= 32) {
        encoderPos = 1;
      }
      else if (encoderPos <= 0) {
        lcd.clear();
        encoderPos = 31;
      }

      lcd.setCursor(0, 0);
      lcd.print(F("Tag einstellen:"));
      lcd.setCursor(0, 1);
      if (encoderPos < 10)
      {
        lcd.print("0");
      }
      lcd.print(encoderPos);
      lcd.print(F("."));
      if (setmonat < 10)
      {
        lcd.print("0");
      }
      lcd.print(setmonat);
      lcd.print(F("."));
      lcd.print(2000 + setjahr);

      if ((millis() - wechslertZeit > entprellZeit) && wechslertGedrueckt == 1)
      {
        wechslertGedrueckt = 0;  // setzt gedrückten Taster zurück
        settag = encoderPos;
        lcd.clear();
        zeitstellen++;
      }

    }

    if (zeitstellen == 1) {
      if (encoderPos >= 13) {
        encoderPos = 1;
      }
      else if (encoderPos <= 0) {
        lcd.clear();
        encoderPos = 12;
      }

      lcd.setCursor(0, 0);
      lcd.print(F("Monat einstellen:"));
      lcd.setCursor(0, 1);
      if (settag < 10)
      {
        lcd.print("0");
      }
      lcd.print(settag);
      lcd.print(F("."));
      if (encoderPos < 10)
      {
        lcd.print("0");
      }
      lcd.print(encoderPos);
      lcd.print(F("."));
      lcd.print(2000 + setjahr);


      if ((millis() - wechslertZeit > entprellZeit) && wechslertGedrueckt == 1)
      {
        wechslertGedrueckt = 0;  // setzt gedrückten Taster zurück
        setmonat = encoderPos;
        lcd.clear();
        zeitstellen++;
      }

    }

    if (zeitstellen == 2) {
      byte encoderPos = 16;
      if (encoderPos <= 15) {
        lcd.clear();
        encoderPos = 50;
      }
      else if (encoderPos >= 51) {
        encoderPos = 16;
      }

      lcd.setCursor(0, 0);
      lcd.print(F("Jahr einstellen:"));
      lcd.setCursor(0, 1);
      if (settag < 10)
      {
        lcd.print("0");
      }
      lcd.print(settag);
      lcd.print(F("."));
      if (setmonat < 10)
      {
        lcd.print("0");
      }
      lcd.print(setmonat);
      lcd.print(F("."));
      lcd.print(2000 + encoderPos);

      if ((millis() - wechslertZeit > entprellZeit) && wechslertGedrueckt == 1)
      {
        wechslertGedrueckt = 0;  // setzt gedrückten Taster zurück
        setjahr = encoderPos;
        lcd.clear();
        zeitstellen++;
      }

    }

    if (zeitstellen == 3) {
      if (encoderPos >= 8) {
        encoderPos = 1;
      }
      else if (encoderPos <= 0) {
        lcd.clear();
        encoderPos = 7;
      }
      lcd.setCursor(0, 0);
      lcd.print(F("Tag der Woche"));
      lcd.setCursor(0, 1);
      switch (encoderPos) {
        case 1:
          lcd.print("Sonntag   ");
          break;
        case 2:
          lcd.print("Montag    ");
          break;
        case 3:
          lcd.print("Dienstag  ");
          break;
        case 4:
          lcd.print("Mittwoch  ");
          break;
        case 5:
          lcd.print("Donnerstag");
          break;
        case 6:
          lcd.print("Freitag   ");
          break;
        case 7:
          lcd.print("Samstag   ");
          break;
      }

      if ((millis() - wechslertZeit > entprellZeit) && wechslertGedrueckt == 1)
      {
        wechslertGedrueckt = 0;  // setzt gedrückten Taster zurück
        settagderwoche = encoderPos;
        lcd.clear();
        zeitstellen++;
      }

    }

    if (zeitstellen == 4) {
      if (encoderPos == 24) {
        encoderPos = 0;
      }
      if (encoderPos >= 24) {
        lcd.clear();
        encoderPos = 23;
      }

      lcd.setCursor(0, 0);
      lcd.print(F("Stunde einstellen:"));
      lcd.setCursor(0, 1);
      if (encoderPos < 10)
      {
        lcd.print("0");
      }
      lcd.print(encoderPos);
      lcd.print(F(":"));
      if (setminute < 10)
      {
        lcd.print("0");
      }
      lcd.print(setminute);
      lcd.print(F(":"));
      if (setsekunde < 10)
      {
        lcd.print("0");
      }
      lcd.print(setsekunde);

      if ((millis() - wechslertZeit > entprellZeit) && wechslertGedrueckt == 1)
      {
        wechslertGedrueckt = 0;  // setzt gedrückten Taster zurück
        setstunde = encoderPos;
        lcd.clear();
        zeitstellen++;
      }

    }

    if (zeitstellen == 5) {
      if (encoderPos == 60) {
        encoderPos = 0;
      }
      else if (encoderPos == 65535) {
        lcd.clear();
        encoderPos = 59;
      }

      lcd.setCursor(0, 0);
      lcd.print(F("Minute einstellen:"));
      lcd.setCursor(0, 1);
      if (setstunde < 10)
      {
        lcd.print("0");
      }
      lcd.print(setstunde);
      lcd.print(F(":"));
      if (encoderPos < 10)
      {
        lcd.print("0");
      }
      lcd.print(encoderPos);
      lcd.print(F(":"));
      if (setsekunde < 10)
      {
        lcd.print("0");
      }
      lcd.print(setsekunde);

      if ((millis() - wechslertZeit > entprellZeit) && wechslertGedrueckt == 1)
      {
        wechslertGedrueckt = 0;  // setzt gedrückten Taster zurück
        setminute = encoderPos;
        lcd.clear();
        zeitstellen++;
      }

    }

    if (zeitstellen == 6) {
      lcd.setCursor(0, 0);
      switch (settagderwoche) {
        case 1:
          lcd.print("Sonntag");
          break;
        case 2:
          lcd.print("Montag");
          break;
        case 3:
          lcd.print("Dienstag");
          break;
        case 4:
          lcd.print("Mittwoch");
          break;
        case 5:
          lcd.print("Donnerstag");
          break;
        case 6:
          lcd.print("Freitag");
          break;
        case 7:
          lcd.print("Samstag");
          break;
      }
      lcd.setCursor(0, 1);
      if (setstunde < 10)
      {
        lcd.print("0");
      }
      lcd.print(setstunde);
      lcd.print(F(":"));
      if (setminute < 10)
      {
        lcd.print("0");
      }
      lcd.print(setminute);
      lcd.print(F(":"));
      if (setsekunde < 10)
      {
        lcd.print("0");
      }
      lcd.print(setsekunde);
      lcd.print(F(" "));
      if (settag < 10)
      {
        lcd.print("0");
      }
      lcd.print(settag);
      lcd.print(F("."));
      if (setmonat < 10)
      {
        lcd.print("0");
      }
      lcd.print(setmonat);
      lcd.print(F("."));
      lcd.print(2000 + setjahr);

      lcd.setCursor(0, 2);
      lcd.print("best");
      lcd.print((char)0xE1);
      lcd.print("tigen um Datum,");
      lcd.setCursor(0, 3);
      lcd.print("Zeit zu setzen.");

      if ((millis() - wechslertZeit > entprellZeit) && wechslertGedrueckt == 1)
      {
        wechslertGedrueckt = 0;  // setzt gedrückten Taster zurück
        setDS3231time(setsekunde, setminute, setstunde, settagderwoche, settag, setmonat, setjahr);
        lcd.clear();
        screen = 1;
      }

    }
  }

  else if (screen == 13)
  {
    if (encoderPos > 1) {
      encoderPos = 1;
    }
    else if (encoderPos < 0) {
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
    switch (encoderPos) {
      case 0:
        lcd.print("nein");
        break;
      case 1:
        lcd.print("ja  ");
        break;
    }
    if ((millis() - wechslertZeit > entprellZeit) && wechslertGedrueckt == 1)
    {
      wechslertGedrueckt = 0;
      if (encoderPos == 0) {
        screen = 1;
      }
      if (encoderPos == 1) {
        lcd.clear();
        screen = 14;
      }
    }
  }


  else if (screen == 14)
  {
    if (encoderPos == 24) {
      encoderPos = 0;
    }
    if (encoderPos >= 24) {
      lcd.clear();
      encoderPos = 23;
    }

    lcd.setCursor(0, 0);
    lcd.print(F("Startzeit setzen:"));
    lcd.setCursor(0, 1);
    if (encoderPos < 10)
    {
      lcd.print("0");
    }
    lcd.print(encoderPos);
    lcd.print(F(" Uhr"));
    if ((millis() - wechslertZeit > entprellZeit) && wechslertGedrueckt == 1)
    {
      setings_a.startwasser = encoderPos;
      wechslertGedrueckt = 0;  // setzt gedrückten Taster zurück
      //EEPROM.update (addr18, startwasser);
      lcd.clear();
      screen = 15;
    }
  }

  else if (screen == 15)
  {
    if (encoderPos == 60) {
      encoderPos = 0;
    }
    else if (encoderPos == 65535) {
      lcd.clear();
      encoderPos = 59;
    }

    lcd.setCursor(0, 0);
    lcd.print(F("Start Minute:"));
    lcd.setCursor(0, 1);
    if (encoderPos < 10)
    {
      lcd.print("0");
    }
    lcd.print(encoderPos);
    lcd.print(F(" Min."));
    if ((millis() - wechslertZeit > entprellZeit) && wechslertGedrueckt == 1)
    {
      setings_a.startwassermin = encoderPos;
      wechslertGedrueckt = 0;  // setzt gedrückten Taster zurück
      //EEPROM.update (addr20, startwassermin);
      lcd.clear();
      screen = 16;
    }
  }

  else if (screen == 16)
  {
    if (encoderPos == 60) {
      encoderPos = 0;
    }
    else if (encoderPos == 65535) {
      lcd.clear();
      encoderPos = 59;
    }

    lcd.setCursor(0, 0);
    lcd.print(F("Ende Minute:"));
    lcd.setCursor(0, 1);
    if (encoderPos < 10)
    {
      lcd.print("0");
    }
    lcd.print(encoderPos);
    lcd.print(F(" Min."));
    if ((millis() - wechslertZeit > entprellZeit) && wechslertGedrueckt == 1)
    {
      setings_a.auswasser = encoderPos;
      wechslertGedrueckt = 0;  // setzt gedrückten Taster zurück
      //EEPROM.update (addr19, auswasser);
      lcd.clear();
      screen = 17;
    }
  }

  else if (screen == 17) {
    if (encoderPos == 60) {
      encoderPos = 0;
    }
    else if (encoderPos == 65535) {
      lcd.clear();
      encoderPos = 59;
    }

    lcd.setCursor(0, 0);
    lcd.print(F("Dauer in Sekunden:"));
    lcd.setCursor(0, 1);
    if (encoderPos < 10)
    {
      lcd.print("0");
    }
    lcd.print(encoderPos);
    lcd.print(F(" Sek."));
    if ((millis() - wechslertZeit > entprellZeit) && wechslertGedrueckt == 1)
    {
      setings_a.sekauswasser = encoderPos;
      wechslertGedrueckt = 0;  // setzt gedrückten Taster zurück
//      EEPROM.update (addr27, sekauswasser);
      lcd.clear();
      screen = 18;
    }
  }

  else if (screen == 18) {

    lcd.setCursor(0, 0);
    lcd.print(F("Startzeit:"));
    lcd.setCursor(0, 1);
    if (setings_a.startwasser < 10)
    {
      lcd.print("0");
    }
    lcd.print(setings_a.startwasser);
    lcd.print(":");
    if (setings_a.startwassermin < 10)
    {
      lcd.print("0");
    }
    lcd.print(setings_a.startwassermin);
    lcd.print(":00 Uhr");
    lcd.setCursor(0, 2);
    lcd.print(F("Ende:"));
    lcd.setCursor(0, 3);
    if (setings_a.startwasser < 10)
    {
      lcd.print("0");
    }
    lcd.print(setings_a.startwasser);
    lcd.print(":");
    if (setings_a.auswasser < 10)
    {
      lcd.print("0");
    }
    lcd.print(setings_a.auswasser);
    lcd.print(":");
    if (setings_a.sekauswasser < 10)
    {
      lcd.print("0");
    }
    lcd.print(setings_a.sekauswasser);

    if ((millis() - wechslertZeit > entprellZeit) && wechslertGedrueckt == 1)
    {
      wechslertGedrueckt = 0;
      asm volatile ("jmp 0");
    }
  }
}

