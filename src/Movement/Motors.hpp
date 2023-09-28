#include <Arduino.h>

#ifndef _MotorSystem
#define _MotorSystem

#define LEFT 1
#define RIGHT 0

#define ADDED 11

class Motor {
    public:
        HardwareTimer* tim;
        int channle;
        uint16_t enable_pin;

        int current_speed = 0;
        int enable = 0;

        Motor(HardwareTimer* tim, int channle, uint16_t enable_pin);

        void setSpeed(const int speed, const int enable);
};

class Driver {
    public: 
        Driver(Motor* motor1, Motor* motor2, Motor* motor3, Motor* motor4);

        void gotoPoint(int theta, int speed);
        void Rotate(int dir, int speed);

        void Brake();

    protected:
        Motor* m1;
        Motor* m2;
        Motor* m3;
        Motor* m4;
};

#endif
