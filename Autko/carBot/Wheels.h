#include <Arduino.h>

#ifndef Wheels_h
#define Wheels_h


class Wheels {
    public: 
        Wheels();
        void attachRight(int pinForward, int pinBack, int pinSpeed);
        void attachLeft(int pinForward, int pinBack, int pinSpeed);
        void attach(int pinRightForward, int pinRightBack, int pinRightSpeed,
                    int pinLeftForward, int pinLeftBack, int pinLeftSpeed);
        void forward();
        void forwardLeft();
        void forwardRight();
        void back();
        void backLeft();
        void backRight();
        void stop();
        void stopLeft();
        void stopRight();
        void setSpeed(uint8_t);
        void setSpeedRight(uint8_t);
        void setSpeedLeft(uint8_t);
        void goForward(double cm);
        void goBack(double cm);
        void moveForward(double cm);
        void moveBackward(double cm);
        void turn(int angle);

        double cnt0;
        double cnt1;

    private: 
        int pinsRight[3];
        int pinsLeft[3];
};

#endif
