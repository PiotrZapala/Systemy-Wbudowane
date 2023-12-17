#include <Arduino.h>
#include "TimerOne.h"
#include "Wheels.h"

#define SET_MOVEMENT(side,f,b) digitalWrite( side[0], f);\
                               digitalWrite( side[1], b)

Wheels::Wheels() 
{}

void Wheels::attachRight(int pF, int pB, int pS)
{
    pinMode(pF, OUTPUT);
    pinMode(pB, OUTPUT);
    pinMode(pS, OUTPUT);
    this->pinsRight[0] = pF;
    this->pinsRight[1] = pB;
    this->pinsRight[2] = pS;
}


void Wheels::attachLeft(int pF, int pB, int pS)
{
    pinMode(pF, OUTPUT);
    pinMode(pB, OUTPUT);
    pinMode(pS, OUTPUT);
    this->pinsLeft[0] = pF;
    this->pinsLeft[1] = pB;
    this->pinsLeft[2] = pS;
}

void Wheels::setSpeedRight(uint8_t s)
{
    analogWrite(this->pinsRight[2], s);
}

void Wheels::setSpeedLeft(uint8_t s)
{
    analogWrite(this->pinsLeft[2], s);
}

void Wheels::setSpeed(uint8_t s)
{
    setSpeedLeft(s);
    setSpeedRight(s);
}

void Wheels::attach(int pRF, int pRB, int pRS, int pLF, int pLB, int pLS)
{
    this->attachRight(pRF, pRB, pRS);
    this->attachLeft(pLF, pLB, pLS);
}

void Wheels::forwardLeft() 
{
    SET_MOVEMENT(pinsLeft, LOW, HIGH);
}

void Wheels::forwardRight() 
{
    SET_MOVEMENT(pinsRight, HIGH, LOW);
}

void Wheels::backLeft()
{
    SET_MOVEMENT(pinsLeft, HIGH, LOW);
}

void Wheels::backRight()
{
    SET_MOVEMENT(pinsRight, LOW, HIGH);
}

void Wheels::forward()
{
    this->forwardLeft();
    this->forwardRight();
}

void Wheels::back()
{
    this->backLeft();
    this->backRight();
}

void Wheels::stopLeft()
{
    SET_MOVEMENT(pinsLeft, LOW, LOW);
}

void Wheels::stopRight()
{
    SET_MOVEMENT(pinsRight, LOW, LOW);
}

void Wheels::stop()
{
    this->stopLeft();
    this->stopRight();
}

void Wheels::goForward(double cm) 
{
    double timeToTravel = (cm*1000.0/60.0);
    int delayTime = (int)(timeToTravel < 0 ? (timeToTravel - 0.5) : (timeToTravel + 0.5));
    this->forward();
    delay(delayTime);
    this->stop();
    delay(2000);
}

void Wheels::goBack(double cm)
{
    double timeToTravel = (cm*1000.0/60.0);
    int delayTime = (int)(timeToTravel < 0 ? (timeToTravel - 0.5) : (timeToTravel + 0.5));
    this->back();
    delay(delayTime);
    this->stop();
    delay(2000);
}

void Wheels::moveForward(double cm) {
  int cnt0_start = cnt0;
  int cnt1_start = cnt1;
  this->forward();
  while (true) {
    if (cnt0 >= cnt0_start + (cm * 2.0) && cnt1 >= cnt1_start + (cm * 2.0)) {
      this->stop();
      break;
    }
  }
}

void Wheels::moveBackward(double cm) {
  int cnt0_start = cnt0;
  int cnt1_start = cnt1;
  this->back();
  while (true) {
    if (cnt0 >= cnt0_start + (cm * 2.0) && cnt1 >= cnt1_start + (cm * 2.0)) {
      this->stop();
      break;
    }
  }
}

void Wheels::turn(int angle){
  if(angle > 0) {
    forwardRight();
    delay(angle*11);
  } else {
    forwardLeft();
    delay(-angle*11);
  }
}
