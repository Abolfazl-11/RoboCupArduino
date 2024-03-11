#include <Arduino.h>
#include "./Motors.hpp"

#define Kp 0.84
#define Ki 1.26
#define Kd 1.07

#define TIME 0.200

#define MAXROTATESPEED 20
#define MOVEINGPIDMULT .55

#define ZONEDISTH 55

#define GETBALLANGLETH 10

#define ROTTH 4

#define GOAL_TH 15

typedef enum Zones {FAR, CLOSE, INGETBALL,  NA} Zones;

class Moves{
    public:
        Moves(Driver* driver);

        void GetBall(int r, int theta, uint16_t speed, Zones* zone, int& goal, int&backingToGoal);

        void RotateToZero(int e);

        void Attack(uint32_t speed);

        void BackToGoal(uint32_t speed);

        double sr1, sr2;
    private:
        Driver* driver;
        float pve = 0;
};
