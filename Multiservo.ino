#include <SoftwareSerial.h>   //library til kommunikation med bluetooth modulet
#include <Time.h>   //library til tidsstyring

#include <Servo.h>    //library til brug af servo
Servo fServo, bServo;   //navn af servo (fServo er den første del af armen, bServo er underarmen, hhv. variablen der styrer ømrører motoren direkt

int bluetoothTx = 1;   //bluetooth pins
int bluetoothRx = 0;
unsigned int realservo;   //varibel til opbevaring af modtaget data fra bluetooth enheden
SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);

//switch pins
int startSwitch = 8;
int up = 7;
int down = 5;

//Led pin
int statusLED = 6;

//Motor til omrøring, pin
int stirMotor = 9;

//Servoer pins
int fArm = 14;
int bArm = 15;

//Luftkøling, pin
int fan = 3;

//motorhastighed udgangsposition, skal kalibreres
int motorStart = 122;

//værdi til start, der styrer at programmet kommer fra det ene trin til det næste
int start = 0;

//positionsvariabler til servo
int fPos, bPos;

//variabler til motorhastighed og LED
long prevMillis;
long interval = 1000;
int ledState = LOW;
int motorCur;

//variabler til tidsregulering
long stanTime;
long chanTime;
long stirTime;
long curTime;
int next; //skift fra tid til kør

//alarm
long alarm;

void setup()
{
  fServo.attach(fArm);   //initialisering af de to servoer
  bServo.attach(bArm);
  //initialisering af kommunikation med copmuteren
  Serial.begin(9600);

  //initialisering af kommunikationen med en anden bluetooth enhed (i dette tilfælde en mobiltelefon inkl. app)
  bluetooth.begin(9600);
  
  //definering af de forskellige pins
  pinMode(fan, OUTPUT);
  pinMode(startSwitch, INPUT);
  pinMode(stirMotor, OUTPUT);
  pinMode(up, INPUT);
  pinMode(statusLED, OUTPUT);
  pinMode(down, INPUT);

}

void loop() {
    //modtager informationer via bluetooth og printer dem på computer skærmen - bl.a. til debugging
  if(bluetooth.available()>= 2 ){    //checker om den modtager et 2 byte nummer
    unsigned int servopos = bluetooth.read(); //det første byte ,unsigned for at øge den positive kapacitet - den mulige negative værdi bliver tilsat som positiv
    unsigned int servopos1 = bluetooth.read();     //det andet byte
    realservo = (servopos1 *256) + servopos;     //sætter tallene sammen, således at vi får det samme tal som vi arbejder med i appen
  }
    
    if (start == 0){ //position af armen
      upDown();   //funktion der styrer positionen af armen
    }
    if (start == 1){
      rotationSpeed();   //funktion der styrer rotationshastigheden af omrøringsmotoren
    }
    if(start == 3){
      timeManagement();   //funktion der styrer tiden, hhv. i hvor lang tid den skal røre rundt
    }
    if (realservo == 4000 || start == 4){ 
      stir();   //funktion der styrer den reelle omrøringsproces
    }
    if(start == 4){
      alarmBlink();   //funktion der indikerer, hvornår den er færdig med at omrøre
   }
 }
  
  void upDown(){
    int top = 0;
    
      if (digitalRead(up) == HIGH){
         fPos = fServo.read();
         fPos++;    //skal kalibreres i forhold til hastighed og i forhold til positionen af servo b
         fServo.write(fPos);

         bPos = bServo.read();
         bPos--;    //skal kalibreres i forhold til hastighed og i forhold til positionen af servo b
         bServo.write(bPos);
        }
      if (digitalRead(down) == HIGH){
         fPos = fServo.read();
         fPos--;    //skal kalibreres i forhold til hastighed og i forhold til positionen af servo b
         fServo.write(fPos);

         bPos = bServo.read();
         bPos++;    //skal kalibreres i forhold til hastighed og i forhold til positionen af servo b
         bServo.write(bPos);
        }
      if (top == 0 && fServo.read() >= 90){
        for(int i = bServo.read();i < 90;i++){
          bServo.write(i);
          delay(20);
        }
        top = 1;
      }
      if (top == 1 && fServo.read() < 90){
        for(int i = bServo.read();i < 45;i++){
          bServo.write(i);
          delay(20);
        }
        top = 0;
      }
      if (realservo >= 1000 && realservo <1090){
        int servo1 = realservo;
        servo1 = map(servo1, 1000,1090,0,90);
        fServo.write(servo1);
        if(servo1 >= 90 && top == 0){
          for(bPos = bServo.read(); bPos < 90; bPos++){ 
            bServo.write(bPos);
            delay(20);
            }
            top = 1;
          }
        if(servo1 < 90 && top == 1){
          for(bPos = bServo.read(); bPos < 20; bPos--){ 
            bServo.write(bPos);
            delay(20);
            }
            top = 0;
          }
        }
    if (realservo == 4000 || digitalRead(startSwitch) == HIGH){
      start += 1;
      delay(200);
      }
  }

  void rotationSpeed(){
      if(interval >= 0 && interval <= 3000){    //Interval variablen styrer motorhastigheden og blink frekvensen 
        if(digitalRead(up) == HIGH){    //hvis knappen trykkes opad
          interval -= 100;
          }
        }
      if(interval >= 0){ 
        if(digitalRead(down) == HIGH){ //hvis knappen trykkes nedad
          interval += 100;
          }
        }
      if (realservo >= 2000 && realservo <2500){    //konverterer data fra bluetooth kommunikationen til enheder fra 0 til 3000, hhv. omrøringshastigheden
        interval = map(realservo,2000,20500,0,3000);
      }
      //LED blink hastighed, der indikerer motorhastigheden
      long curMillis = millis();    //vi benytter os af den allerede tilstedeværende timer i arduinoen for at holde styr på blink frekvensen i forhold til motoren
      if (curMillis - prevMillis >= interval) {
        prevMillis = curMillis;
        if (ledState == LOW) {
          ledState = HIGH;
          } else {
            ledState = LOW;
          }
        digitalWrite(statusLED, ledState);
      }
      digitalWrite(statusLED, HIGH);
      motorCur = map(interval,3000,0,0,255);    //selve motorhastigheden
      if (realservo == 4000 || digitalRead(startSwitch) == HIGH){    //hvis start knappen trykkes igen skifter den over til det næste skridt
        start += 1;
        delay(200);
      }
  }

 void timeManagement(){    //med bluetooth ellers er der en standard til på 90 minutter
  if (realservo >= 3002 && realservo <3180){    //bluetooth - minutter fra 0 til 180
        chanTime = realservo;
        chanTime = map(chanTime, 3000, 3090, 0, 90);    //koverterer data til antal minutter
        stirTime = chanTime*60000;    //siden at vi arbejder med millisekunder er vi nød til at gange antal minutter med 60.000 for at få antal minutter
      } else {
        stirTime = stanTime*60000;    //standart tid
        }
        if (realservo == 4000 || digitalRead(startSwitch) == HIGH){    //enten via bluetooth eller via start knap kan der skiftes over til det næste skridt
          start += 1;
          delay(200);
      }
 }

 void stir(){    //selve omøringsfunktionen
    if(next == 0){
        curTime = millis()+stirTime;
        next ++;
      }
    if(next == 1 && millis() <= curTime){    //motoren kører i det tidsrum, der blev bestemt i tidsfunktionen og med den hastighed, der blev bestemt i hastighedsfunktionen
      digitalWrite(stirMotor, motorCur);
      }
    if(realservo == 5000 || digitalRead(startSwitch) == HIGH){
        next = 0;
        start ++;
      }
 }

 void alarmBlink(){    //funktionen der styrer alarmen
      long curMillis = millis();
      if (curMillis - prevMillis >= 200) {
        prevMillis = curMillis;
        if (ledState == LOW) {
          ledState = HIGH;
          } else {
            ledState = LOW;
          }
        digitalWrite(statusLED, ledState);
      }
      if(next == 0){    //blinker i tre minutter
        long alarm = millis()+3*60000;
        next ++;
      }
      if(next == 1){
        if(alarm >= millis()){
          next = 0;
          start = 0;
        }
      }
      if(digitalRead(startSwitch) == HIGH){    //går til sidst tilbage til udgangspositionen
        next = 0;
        start = 0;
      }
    }

