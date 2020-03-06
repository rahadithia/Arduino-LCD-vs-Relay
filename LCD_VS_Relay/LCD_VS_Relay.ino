#include <OneWire.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27 ,2,1,0,4,5,6,7,3, POSITIVE);
float teg[10];
double tds, konduktifitas;
float rata_rata_teg;
const int phSensorPin = A0;
float Po              = 0;
#define RELAYASAM 7 //CW is defined as pin #7//
#define RELAYBASA 8 //CCW is defined as pin #8//
#define BUZZER 9

void setup(){
  pinMode(phSensorPin, INPUT);
  pinMode(RELAYASAM, OUTPUT); //Set CW as an output//
  pinMode(RELAYBASA, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);
  lcd.begin (16,2); //LCD untuk ukuran 16x2
  lcd.setCursor(3, 2); //baris kedua   
  lcd.print("Rahadithia");
  
  lcd.setCursor(1,1);
  lcd.print("Volt : ");
  lcd.setCursor(1,2);
  lcd.print("pH : ");
  }

  
void loop(){ 
  int nilaiPengukuranPh = analogRead(phSensorPin);
  Serial.print("nilai ADC Ph:");
  Serial.println(nilaiPengukuranPh);
  lcd.setCursor(3, 0);
  lcd.print(nilaiPengukuranPh);
  double TeganganPh = 5 / 1024.0 * nilaiPengukuranPh;
  Serial.print("Tegangan Ph:");Serial.println(TeganganPh, 3);
  lcd.setCursor(1,1);
  lcd.print(TeganganPh);
  lcd.print(" volt");
  lcd.setCursor(2,2);
  delay(3000);

  if(nilaiPengukuranPh>20)
  {
  lcd.print("Pompa 1 On");
  lcd.setCursor(0,1);
  lcd.print("Cairan Basa Mengalir");
  lcd.setCursor(0,2);
  digitalWrite(RELAYBASA, HIGH);//Motor runs counter-clockwise// 
  delay(3000);            //For 3 second//
  digitalWrite(RELAYBASA, LOW); //Motor stops//  
  lcd.clear();
  }
  
  else if(nilaiPengukuranPh>20 && nilaiPengukuranPh<=100)
  {
  digitalWrite(BUZZER,HIGH); //Buzzer run//
  lcd.print("Pompa 2 On");
  lcd.setCursor(1,1);
  lcd.print("Cairan Asam Mengalir");
  lcd.setCursor(2,0);
  digitalWrite(RELAYASAM,HIGH); //Motor runs clockwise// 
  delay(3000);  //for 3 second//
  digitalWrite(RELAYASAM, LOW); //Motor stops//
  digitalWrite(BUZZER,LOW); //Buzzer Stop//
  lcd.clear();

  }
  
  for ( int i=0; i<10; i++){
  int val = analogRead(A0);
  teg[i] = val * (5.0/1023);
  }
  rata_rata_teg = (teg[0] + teg[1] + teg[2] + teg[3] + teg[4] + teg[5] + teg[6] + teg[7] + teg[8] + teg[9])/10 ;
  delay(1000);
  lcd.clear();
  tds = (211.2254 * rata_rata_teg) - 144.1466;
  konduktifitas = (0.3442 * rata_rata_teg) - 0.253;
  lcd.setCursor(0,0);
  lcd.print(rata_rata_teg);
  lcd.setCursor(0,1);
  lcd.print(tds);
  lcd.setCursor(8,1);
  lcd.print(konduktifitas);
  delay(1000);
}

  
