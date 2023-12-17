#include "Wheels.h"
#include <IRremote.h>
#include <Servo.h>
#include <LiquidCrystal_I2C.h>
#include <TaskScheduler.h>

//odległość od przeszkody
#define maxDistanceToObstacle 30
// biały
#define EnA 6
// szary
#define IN1 11
// fioletowy
#define IN2 10
// niebieski
#define IN3 8
// zielony
#define IN4 7
// zolty
#define EnB 5
// szary
#define PIN_OUT1 2
// zielony
#define PIN_OUT2 13
// niebieski
#define PIN_OUT3 12
// fioletowy
#define PIN_OUT4 4
// brązowy
#define PIN_OUT5 3
// niebieski
//#define RECV_PIN 2
// lewo
#define IR_BUTTON_1 8
// prawo
#define IR_BUTTON_2 90
// dół
#define IR_BUTTON_3 82
// góra
#define IR_BUTTON_4 24
// stop
#define IR_BUTTON_5 28
// pin kontroli serwo (musi być PWM), pomarańczowy 
#define SERVO 9

#define READ_DIGITAL()   IRvalue1 = digitalRead(PIN_OUT1);\
                         IRvalue2 = digitalRead(PIN_OUT2);\ 
                         IRvalue3 = digitalRead(PIN_OUT3);\ 
                         IRvalue4 = digitalRead(PIN_OUT4);\ 
                         IRvalue5 = digitalRead(PIN_OUT5);

// piny dla sonaru (HC-SR04)
//fioletowy
#define TRIG A2
//zielony
#define ECHO A3

#define INTINPUT0 A0
#define INTINPUT1 A1

byte LCDAddress = 0x27;

float pi = 3.14;
int cntL = 0;
int cntR = 0;
int prevCntL;
int prevCntR;
boolean prevL;
boolean prevR;
int IRvalue1 = 0;
int IRvalue2 = 0;
int IRvalue3 = 0;
int IRvalue4 = 0;
int IRvalue5 = 0;

volatile int lastDistance;

unsigned long startTime = 0;
const int duration = 100;  
const int duration1 = 400;
const int speed = 250;

uint8_t arrowRight[8] =
{
    0b01000,
    0b01100,
    0b00110,
    0b11111,
    0b11111,
    0b00110,
    0b01100,
    0b01000
};

Wheels w;
Servo serwo;
LiquidCrystal_I2C lcd(LCDAddress, 16, 2);
//Scheduler scheduler;

//Task taskMeasureDistance(100, TASK_FOREVER, &tellDistance);
//Task taskRemote(50, TASK_FOREVER, &remote);

void setup() {
  w.cnt0 = 0;
  w.cnt1 = 1;
  pinMode(TRIG, OUTPUT);    // TRIG startuje sonar
  pinMode(ECHO, INPUT);     // ECHO odbiera powracający impuls
  pinMode(PIN_OUT1,INPUT);
  pinMode(PIN_OUT2,INPUT);
  pinMode(PIN_OUT3,INPUT);
  pinMode(PIN_OUT4,INPUT);
  pinMode(PIN_OUT5,INPUT);
  w.attach(IN1, IN2, EnA, IN3, IN4, EnB);
  w.setSpeed(speed);
  pinMode(INTINPUT0, INPUT);
  pinMode(INTINPUT1, INPUT);
  Serial.begin(9600);
  //IrReceiver.begin(RECV_PIN);
  serwo.attach(SERVO);
  //attachInterrupt(digitalPinToInterrupt(ECHO), tellDistance, CHANGE);
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, arrowRight);
  /*
  scheduler.init();
  scheduler.addTask(taskMeasureDistance);
  scheduler.addTask(taskRemote);
  taskMeasureDistance.enable();
  taskRemote.enable();
  */
}

void loop() {
  lineFollower();
  //avoidObstacles();
  //useLCD();
  //remote();  
  //scheduler.execute();
}

void lineFollower() {  
  READ_DIGITAL();

  if ((IRvalue1 == 1 && IRvalue2 == 1 && IRvalue3 == 1 && IRvalue4 == 1 && IRvalue5 == 1) || (IRvalue2 == 1 && IRvalue3 == 1 && IRvalue4 == 1)) {
    w.setSpeed(150);
    while ((IRvalue1 == 1 && IRvalue2 == 1 && IRvalue3 == 1 && IRvalue4 == 1 && IRvalue5 == 1) || (IRvalue2 == 1 && IRvalue3 == 1 && IRvalue4 == 1)) {
      w.forward();
      READ_DIGITAL();
    }
    w.stop();
  } else if ( IsCond2True() ) {
    w.setSpeed(250);
    while (IsCond2True() ) {
      w.forwardRight();
      READ_DIGITAL();
    }
    w.stop();
  } else if (IsCond3True() ) {
      w.setSpeed(250);
      while (IsCond3True() ) {
        w.forwardLeft();
        READ_DIGITAL();
      }
      w.stop();
  }
}

bool IsCond2True() {
   return ((IRvalue1 == 0 && IRvalue2 == 0 && IRvalue3 == 1 && IRvalue4 == 1 && IRvalue5 == 1) ||
           (IRvalue1 == 0 && IRvalue2 == 1 && IRvalue3 == 1 && IRvalue4 == 1 && IRvalue5 == 1) ||
           (IRvalue1 == 0 && IRvalue2 == 0 && IRvalue3 == 1 && IRvalue4 == 1 && IRvalue5 == 1) ||
           (IRvalue1 == 0 && IRvalue2 == 0 && IRvalue3 == 0 && IRvalue4 == 1 && IRvalue5 == 1) ||
           (IRvalue1 == 0 && IRvalue2 == 0 && IRvalue3 == 0 && IRvalue4 == 0 && IRvalue5 == 1));
}

bool IsCond3True() {
  return ((IRvalue1 == 1 && IRvalue2 == 1 && IRvalue3 == 0 && IRvalue4 == 0 && IRvalue5 == 0) ||
          (IRvalue1 == 1 && IRvalue2 == 0 && IRvalue3 == 0 && IRvalue4 == 0 && IRvalue5 == 0) ||
          (IRvalue1 == 1 && IRvalue2 == 1 && IRvalue3 == 0 && IRvalue4 == 0 && IRvalue5 == 0) ||
          (IRvalue1 == 1 && IRvalue2 == 1 && IRvalue3 == 1 && IRvalue4 == 0 && IRvalue5 == 0) ||
          (IRvalue1 == 1 && IRvalue2 == 1 && IRvalue3 == 1 && IRvalue4 == 1 && IRvalue5 == 0));
}

void useLCD() {
  lcd.setCursor(0,1);
  lcd.print("L");
  lcd.setCursor(1,1);
  lcd.print(digitalRead(INTINPUT0));
  lcd.print(readSpeedL());
  lcd.print(cntL);
  lcd.setCursor(3,1);
  lcd.print("R");
  lcd.setCursor(4,1);
  lcd.print(cntR);
  lcd.print(digitalRead(A1));
  lcd.print(readSpeedR());
}

int servo() {
  /* rozejrzyj się w zakresie od 0 stopni (patrz na jedną burtę)
  *  do 180 stopni (patrz na prawą burtę). Wydrukuj na konsoli
  *  kierunek patrzenia i najbliżej widziany obiekt (pojedynczy pomiar)
  */
 int maxDistance = 0;
 int angleToGo;
 int distance;

  for(byte angle = 0; angle < 180; angle+= 20) {
    serwo.write(angle);
    Serial.print("Patrzę w kącie ");
    Serial.print(angle);
    distance = lastDistance;
    Serial.print(": widzę coś w odległości ");
    Serial.println(distance);
    if(distance>maxDistance){
      angleToGo = angle;
      maxDistance = distance;
    }
    delay(500);

  }
  serwo.write(90);
  return angleToGo;
}

bool condition(int distanceToObstacle, int arg) {
    if (distanceToObstacle < arg) {
        return 0;
    } else {
        return 1;
    }
}

void remote() {
 if (IrReceiver.decode()) {
    IrReceiver.resume();
    //Serial.println(IrReceiver.decodedIRData.command);
    int command = IrReceiver.decodedIRData.command;
    switch (command) {
      case IR_BUTTON_1: {
        startTime = millis();
        while (millis() - startTime <= duration) {
          w.forwardLeft();
        }
        w.stop();
        break;
      }
      case IR_BUTTON_2: {
        startTime = millis();
        while (millis() - startTime <= duration) {
          w.forwardRight();
        }
        w.stop();
        break;
      }
      case IR_BUTTON_3: {
        startTime = millis();
        while (millis() - startTime <= duration) {
          w.back();
        }
        w.stop();
        break;
      }
      case IR_BUTTON_4: {
        startTime = millis();
        //tellDistance();
        //Serial.println(lastDistance);
        while (millis() - startTime <= duration /*&& lastDistance > maxDistanceToObstacle*/) {
          w.forward();
          //Serial.println(readSpeedR());
        }
        w.stop();
        break;
      }
      case IR_BUTTON_5: {
        w.stop();
        break;
      }
      default: {
        Serial.println("Button not recognized");
      }
    }
  }
}

void tellDistance() {
  unsigned long tot;      // czas powrotu (time-of-travel)
  unsigned int distance;

  /* uruchamia sonar (puls 10 ms na `TRIGGER')
  * oczekuje na powrotny sygnał i aktualizuje
  */
  digitalWrite(TRIG, HIGH);
  delay(10);
  digitalWrite(TRIG, LOW);
  tot = pulseIn(ECHO, HIGH);

  /* prędkość dźwięku = 340m/s => 1 cm w 29 mikrosekund
  * droga tam i z powrotem, zatem:
  */
  distance = tot/58;

  lastDistance = distance;
}

void avoidObstacles() {
  tellDistance();
  int cond = 1;
  int angle = 0;
  while (true) {
    switch (cond) {
      case 0: {
        w.stop();
        tellDistance();
        //lcd.setCursor(0,0);
        //lcd.print("Dist to obst:");
        //lcd.print(lastDistance);
        if (condition(lastDistance, maxDistanceToObstacle) == 1) {
          cond = 1;
        }
        else {
          cond = 2;
        }
        break;
      }
      case 1: {
        w.forward();
        tellDistance();
        //lcd.setCursor(0,0);
        //lcd.print("Dist to obst:");
        //lcd.print(lastDistance);
        cond = condition(lastDistance, maxDistanceToObstacle);
        break;
      }
      case 2: {
        w.stop();
        angle = 90 - servo();
        Serial.println(angle);
        if (condition(lastDistance, maxDistanceToObstacle) == 1) {
          cond = 1;
        }
        else {
          cond = 3;
        }
        break;
      }
      case 3: {
        w.turn(angle);
        if (condition(lastDistance, maxDistanceToObstacle) == 0) {
          cond = 1;
        }
        else {
          cond = 0;
        }
        break;
      }
    }
  }
}

int readSpeedL(){
  int diff2 = cntL - prevCntL;
  int deg2 = diff2 * 9;
  
  float f2 = ((deg2/50.00)*1000)/(2*pi*3);
    
  prevCntL = cntL;
  prevCntR = cntR;

  return f2;
}

int readSpeedR(){

  int diff = cntR - prevCntR;
  int deg = diff * 9;

  float f = ((deg/50.00)*1000)/(2*pi*3);
    
  prevCntL = cntL;
  prevCntR = cntR;

  return f;
}

static void increment() {
  if(digitalRead(A0)==HIGH )
  {
    if(prevL==false){
    ++cntL;
    prevL=true;
    }
  }
  if(digitalRead(A0)==LOW){
    prevL=false;
  }
  if(digitalRead(A1)==HIGH )
  {
    if(prevR==false){
    ++cntR;
    prevR=true;
    }
  }
  if(digitalRead(A1)==LOW){
    prevR=false;
  }
}