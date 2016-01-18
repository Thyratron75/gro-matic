//
//    FILE: gro-matic
//  AUTHOR: zrox
// VERSION: 0.9.6
//
// 2004 I2C LCD (bei UNO, SDA = Pin A4, SCL = Pin A5)
// DS3231 RTC (bei UNO, SDA = Pin A4, SCL = Pin A5)
// DHT22 Thermometer, Hydrometer an Pin8
// GY-30 Lux Meter (bei UNO, SDA = Pin A4, SCL = Pin A5)
//
// Taster mit 10K OHM Wiederstand an Pin2 zum einschalten der Display hintergrundbelaeuchtung.
// Taster mit INTERNAL_PULLUP Pin3 zum wechseln des Lichtmodus, durch verwendung des INTERNAL_PULLUP sind umschaltzeiten schneller.
//
// 8er Relais Modul
// Relay 1, an Pin 9 als Wechsler für Stufentrafo
// Relay 2, an Pin 7 zur NDL (P braunes Kabel) Steuerung
// Relay 3, an Pin 6 zur NDL (N blaues Kabel) Steuerung
// Relay 4, an Pin 10 zur LSR (P braunes Kabel) Steuerung
// Relay 5, an Pin 11 zur LSR (N blaues Kabel) Steuerung
// Relay 6, an Pin 12 zur Umluftventi steuerung


// Relais 1 Anschluss
//  
// Pin Links z.B. an 80V anschluss vom Stufentrafo, Pin in der Mitte P (braunes Kabel) an LTI und Pin Rechts an z.B. 190V vom Stufentrafo.
// Anschüsse zum Stufentrafo  80V  P  190V
//                             |   |   |
//                             ---------
//                             |o  o  o|
//                             | Relais|
//                             ---------
//
// Anschluss Relais 2 -5
// Stromzufuhr (von der Steckdose) immer in der Mitte und anschluss zur NDL oder LSR rechts.

#include "Wire.h"
#include "dht.h"
#include "LiquidCrystal_I2C.h"
#include "DS3231.h"
#include "EEPROM.h"

#define DS3231_I2C_ADDRESS 0x68
#define DHT22_PIN 8
dht DHT;

// GY-30 Lux Meter
int BH1750_address = 0x23; // i2c Addresse 
byte buff[2];

//Backlight button
const int buttonPin = 2;
int buttonState = 0;
const int wechslertPin = 3;
int addr = 0; // Eprom adresse einstellen
int wechslertStatus = LOW;           // aktuelles Signal vom Eingangspin
int wechslertGedrueckt = 0;          // abfragen ob Taster gedrückt war
int lichtmodus = 0;               // festlegen der verschiedenen Lichtprogramme
int entprellZeit = 200;           // Zeit für Entprellung, anpassen!
int speichern = 0;                // Setzt autosave auf aus, erst wenn Modus gewechselt wird, wird gespeichert

// ab hier Zeit für die Belichtungsmodis einstellen
int lsr_an = 5;      // LSR um 5:00 Uhr einschalten
int lsr_aus = 22;    // LSR um 22:59:59 Uhr ausschalten
int grow_licht_an = 5;   // NDL im Grow-Modi um 5 Uhr einschalten
int grow_licht_aus = 22; // NDL im Grow-Modi um 22:59:59 Uhr ausschalten
int bloom_licht_an = 5;  // NDL im Bloom-Modi um 5 Uhr einschalten
int bloom_licht_aus = 16;  // NDL im Bloom-Modi um 16:59:59 Uhr ausschalten

int counter = 0;

unsigned long wechslertZeit = 0;     // Zeit beim drücken des Tasters

// initialisiere die LCD_I2C library 
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);


DS3231 RTC;
int tempC;

int luft_relay = 9;
int licht_relay_p = 7;
int licht_relay_n = 6;
int lsr_relay_p = 10;
int lsr_relay_n = 11;
int ventilator = 12;

// Convertiere binaeren dezimal code zu normalen dezimal nummern
byte bcdToDec(byte val)
{
  return( (val/16*10) + (val%16) );
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
  Wire.write(0); 					// setze DS3231 register pointer zu 00h
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



void displayTime()
{
  byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
  // hole daten von DS3231
  readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);

  lcd.setCursor(0, 0);

  lcd.print(hour, DEC);
  lcd.print(":");
  if (minute<10)
  {
    lcd.print("0");
  }
  lcd.print(minute, DEC);
  lcd.print(":");
  if (second<10)
  {
    lcd.print("0");
  }
  lcd.print(second, DEC);
  lcd.print(" ");

  switch(dayOfWeek){
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
  lcd.print(dayOfMonth, DEC);
  lcd.print(" ");
  switch(month){
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

void dht22()
{
 // lese DHT DATA
 int chk = DHT.read22(DHT22_PIN);	
 // DISPLAY DATA
 lcd.setCursor(0, 1);					// setze curserposition
 lcd.print(DHT.temperature, 1);				
 lcd.print((char)223);
 lcd.print("C ");
 lcd.print(" ");
 lcd.print(DHT.humidity, 1);
 lcd.print("%");
 lcd.print(" RLF");
}

void BH1750_Init(int address){
  
  Wire.beginTransmission(address);
  Wire.write(0x10); // 1 [lux] aufloesung
  Wire.endTransmission();
}
 
byte BH1750_Read(int address){
  
  byte i=0;
  Wire.beginTransmission(address);
  Wire.requestFrom(address, 2);
  while(Wire.available()){
    buff[i] = Wire.read(); 
    i++;
  }
  Wire.endTransmission();  
  return i;
}
 
char sendeInhalt = ' '; 

void DS3231temp()
{
  tempC = RTC.getTemperature();
  
    lcd.setCursor(0, 3);
    lcd.print("Case:");
    lcd.print(tempC);
    lcd.print((char)223);
    lcd.print("C");
}

void setup()
{
  
  
    
  Wire.begin();
  lcd.begin(20,4);					// stelle LCD groesse ein
 
  lcd.setCursor(0,0); 
  lcd.print("..:: Gro-Matic ::..");
  lcd.setCursor(0,2);
  lcd.print("  V. 0.9.6 by zrox");
  
  delay (3000);
  
  lcd.clear();
  
  BH1750_Init(BH1750_address);
  
  delay(500);
  Serial.begin(9600);
  digitalWrite(licht_relay_p, HIGH); // alle Relais Pins beim Start auf HIGH setzen und damit ausschalten.
  digitalWrite(licht_relay_n, HIGH); 
  digitalWrite(lsr_relay_p, HIGH); 
  digitalWrite(lsr_relay_n, HIGH); 
  digitalWrite(luft_relay, HIGH);
  digitalWrite(ventilator, HIGH);
  pinMode(luft_relay,  OUTPUT);  // alle Relais Pins als ausgang setzen
  pinMode(licht_relay_p,  OUTPUT);
  pinMode(licht_relay_n,  OUTPUT);
  pinMode(lsr_relay_p,  OUTPUT);
  pinMode(lsr_relay_n,  OUTPUT);
  pinMode(ventilator, OUTPUT);
  pinMode(buttonPin, INPUT);       // init. den backlight Taster
  pinMode(wechslertPin, INPUT_PULLUP);    // Taster Pin wird als Eingang gesetzt
  lichtmodus = EEPROM.read(lichtmodus); // Eprom auslesen
  Serial.print(lichtmodus);
}


void loop()
{
  buttonState = digitalRead(buttonPin); // lese ststus des display buttons und schalte wenn betätigt fuer 30 sek. an
  
    if (buttonState == HIGH) {
    counter = 30;        
    // schalte LCD an:
    lcd.setBacklight(255);
    }
    if (buttonState == LOW) {
    if (counter > 0) {
    counter--;
    }
    }
    if (counter <= 0) {
    // schalte LCD nach ablauf der 30 sek. aus:
    lcd.setBacklight(0);
  }

  displayTime(); 			// zeige die RTC Daten auf dem LCD display,
  dht22(); 				// zeige temp und rlf auf dem LCD display,
  DS3231temp();
  
  void BH1750_Init(int address);

  delay(1000); 				// jede Sekunde
    float valf=0;
 
  if(BH1750_Read(BH1750_address)==2){
    
    {
    valf = ((buff[0]<<8)|buff[1]);       
    
    lcd.setCursor(0, 2);
    lcd.print("LUX: ");
    if (valf<1000) lcd.print(" ");
    if (valf<100) lcd.print(" ");
    if (valf<10) lcd.print(" ");
    
    lcd.print(valf,2);
    }
     
  }
  
  
  // Pruefe Temperatur, ist sie unter 22 Grad C bleibt Stufentrafo gedimmt (z.B. 80V) 
  // ist Temp gleich oder höher wird auf hoechste stufe (z.B. 190V) geschalten.
  if (DHT.temperature <= 22) {
  digitalWrite(luft_relay, HIGH);}
  else {
  digitalWrite(luft_relay, LOW);}
  
  wechslertStatus = digitalRead(wechslertPin);
  
    // Wenn der Wechseltaster gedrückt ist...
  if (wechslertStatus == HIGH)
  {
    wechslertZeit = millis();      // aktualisiere tasterZeit
    wechslertGedrueckt = 1;        // speichert, dass Taster gedrückt wurde
  }
 
  // Wenn Taster gedrückt wurde die gewählte entprellZeit vergangen ist soll Lichtmodi und gespeichert werden ...
  if ((millis() - wechslertZeit > entprellZeit) && wechslertGedrueckt == 1)
    {
      lichtmodus++;             // lichtmodus wird um +1 erhöht
      wechslertGedrueckt = 0;      // setzt gedrückten Taster zurück
      speichern=1;
    }
    
     if(speichern==1) {            // Austosave des Lichtmodis starten
        EEPROM.write(addr, lichtmodus);
    speichern = 0;                // Austosave des Lichtmodis wieder beenden, sonst wird permanent das EEprom im loop beschieben..
  }
    
  //*************************************Programm-Modis**************************************
   
  // Wenn Lichtmodus 0 ist, starte bei LSR modus
  if (lichtmodus == 0)
  {
      lcd.setCursor(10, 3);
      lcd.print("  LSR mode");
      

  byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
  // hole daten von DS3231
  readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);
 

 if ((hour >= lsr_an) && (hour <= lsr_aus))
 {digitalWrite(lsr_relay_p, LOW); //schaltet lsr um 5 Uhr an und um 22:59:59 Uhr aus
 digitalWrite(lsr_relay_n, LOW);
 digitalWrite(licht_relay_p, HIGH); //schaltet ndl Relais aus sollten sie noch an sein
 digitalWrite(licht_relay_n, HIGH);
 // Umluftventilator alle 15 minuten einschalten wenn licht an
  if (minute == 15)
  {digitalWrite(ventilator, LOW);}
  else if (minute == 30)
  {digitalWrite(ventilator, HIGH);}
  else if (minute == 45)
  {digitalWrite(ventilator, LOW);}
  else if (minute == 59)
  {digitalWrite(ventilator, HIGH);}
 }
 
  else
 {digitalWrite(lsr_relay_p, HIGH);
  digitalWrite(lsr_relay_n, HIGH);
  if ((hour >= grow_licht_aus) && (hour <= grow_licht_an) || (hour >= bloom_licht_aus) && (hour <= bloom_licht_an))
  digitalWrite(licht_relay_p, HIGH);  //schaltet lsr Relais aus sollten sie noch an sein
  digitalWrite(licht_relay_n, HIGH);
  if (minute == 15) // schaltet Ventilator im Nachtmodus 1 x jede Stunde fuer 5 Min. an
  {digitalWrite(ventilator, LOW);}
  else if (minute == 20)
  {digitalWrite(ventilator, HIGH);}
}
 
  }
 
  else if (lichtmodus == 1)
  {
      lcd.setCursor(10, 3);
      lcd.print(" Grow mode");
      
  byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
  // hole daten von DS3231
  readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);
  

 if ((hour >= grow_licht_an) && (hour <= grow_licht_aus))
 {digitalWrite(licht_relay_p, LOW); //schaltet ndl im Grow modus 18h licht um 5 Uhr an und um 22:59:59 Uhr aus
 digitalWrite(licht_relay_n, LOW);
 digitalWrite(lsr_relay_p, HIGH);  //schaltet lsr Relais aus sollten sie noch an sein
 digitalWrite(lsr_relay_n, HIGH); 
 
 // Umluftventilator alle 15 minuten einschalten wenn licht an
  if (minute == 15)
  {digitalWrite(ventilator, LOW);}
  else if (minute == 30)
  {digitalWrite(ventilator, HIGH);}
  else if (minute == 45)
  {digitalWrite(ventilator, LOW);}
  else if (minute == 59)
  {digitalWrite(ventilator, HIGH);}
}
 
 else
 {digitalWrite(licht_relay_p, HIGH);
  digitalWrite(licht_relay_n, HIGH);  
  if ((hour >= lsr_aus) && (hour <= lsr_an))
  digitalWrite(lsr_relay_p, HIGH);  //schaltet lsr Relais aus sollten sie noch an sein  
  digitalWrite(lsr_relay_n, HIGH); 
  if (minute == 15) // schaltet Ventilator im Nachtmodus 1 x jede Stunde fuer 5 Min. an
  {digitalWrite(ventilator, LOW);}
  else if (minute == 20)
  {digitalWrite(ventilator, HIGH);}
}

  }
 
   else if (lichtmodus == 2)
  {
      lcd.setCursor(10, 3);
      lcd.print("Bloom mode");
            
  byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
  // hole daten von DS3231
  readDS3231time(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);
  

 if ((hour >= bloom_licht_an) && (hour <= bloom_licht_aus))
 {digitalWrite(licht_relay_p, LOW); //schaltet ndl im Bloom modus 12h licht um 5 Uhr an und um 16:59:59 Uhr aus
 digitalWrite(licht_relay_n, LOW);
 digitalWrite(lsr_relay_p, HIGH);  //schaltet lsr Relais aus sollten sie noch an sein
 digitalWrite(lsr_relay_n, HIGH); 
 
 // Umluftventilator alle 15 minuten einschalten wenn licht an
  if (minute == 15)
  {digitalWrite(ventilator, LOW);}
  else if (minute == 30)
  {digitalWrite(ventilator, HIGH);}
  else if (minute == 45)
  {digitalWrite(ventilator, LOW);}
  else if (minute == 59)
  {digitalWrite(ventilator, HIGH);}
}
 
 else
 {digitalWrite(licht_relay_p, HIGH);
  digitalWrite(licht_relay_n, HIGH);  
  if ((hour >= grow_licht_aus) && (hour <= grow_licht_an) || (hour >= lsr_aus) && (hour <= lsr_an))
  digitalWrite(lsr_relay_p, HIGH);  //schaltet lsr Relais aus sollten sie noch an sein  
  digitalWrite(lsr_relay_n, HIGH);
  if (minute == 15) // schaltet Ventilator im Nachtmodus 1 x jede Stunde fuer 5 Min. an
  {digitalWrite(ventilator, LOW);}
  else if (minute == 20)
  {digitalWrite(ventilator, HIGH);}
}

  }
 
  // Wenn der Lichtmodus auf 3 sprngt, setzte ihn wieder zurück auf 0 um von vorne zu beginnen
  else
  {
    lichtmodus = 0;
  }


}

