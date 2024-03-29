#include <Arduino.h>

#ifndef _MotorSystem
#define _MotorSystem

#define LEFT 1
#define RIGHT 0

#define MAXSPEED 55

class Motor {
    public:
        HardwareTimer* tim;
        int channle;
        uint16_t enable_pin;
        bool reverse;
        float added;

        int current_speed = 0;
        int enable = 0;

        Motor(HardwareTimer* tim, int channle, uint16_t enable_pin, bool r, float added);

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

        Motor* m1;
        Motor* m2;
        Motor* m3;
        Motor* m4;

        int current_theta = 0;
        bool isMoving = false;
};

#endif
