#include <Arduino.h>

#ifndef _MotorSystem
#define _MotorSystem

#define LEFT 1
#define RIGHT 0

class Motor {
    public:
        HardwareTimer* tim;
        int channle;
        uint16_t enable_pin;
        bool reverse;
        uint32_t added;

        int current_speed = 0;
        int enable = 0;

        Motor(HardwareTimer* tim, int channle, uint16_t enable_pin, bool r, uint32_t added);

        void setSpeed(const int speed, const int enable);
};

class Driver {
    public: 
        Driver(Motor* motor1, Motor* motor2, Motor* motor3, Motor* motor4);

        // set the motor speeds to move the robot in the theta angle
        void gotoPoint(int theta, int speed);
        void Rotate(int dir, int speed);

        // Braking the motors
        void Brake();

    protected:
        Motor* m1;
        Motor* m2;
        Motor* m3;
        Motor* m4;
};

#endif
