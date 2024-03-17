#include <Arduino.h>
#include "./Motors.hpp"

#define Kp 0.57
#define Ki 0.8
#define Kd 0.3

#define TIME 0.200

#define MAXROTATESPEED 30
#define MOVEINGPIDMULT 1.4

#define ZONEDISTH 65
#define GETBALLZONE_TH 40

#define FGETBALLANGLETH 14
#define CGETBALLANGLETH 8

#define FB_TH 35

#define CLOSEF_OFF 40
#define CLOSEB_OFF 55

#define ROTTH 2.5

#define GOAL_TH 15

typedef enum Zones {FAR, CLOSEFRONT, CLOSEREAR, FGETBALL, INGETBALL,  NA} Zones;

class Moves{
    public:
        Moves(Driver* driver);

        void GetBall(int r, int theta, uint16_t speed, Zones* zone, int& goal, int& backingToGoal);

        void RotateToZero(double e);

        void Attack(uint32_t speed);

        void BackToGoal(uint32_t speed);

		void CenterInGoal(uint32_t speed, bool& centering);

        double sr1, sr2, sr3;
    private:
        Driver* driver;
        float pve = 0;
};
