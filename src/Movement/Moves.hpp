#include <Arduino.h>
#include "./Motors.hpp"

#define Kp 0.84
#define Ki 1.26
#define Kd 1.07

#define TIME 0.200

#define MAXROTATESPEED 20
#define MOVEINGPIDMULT .55

#define ZONEDISTH 50
#define GETBALLANGLETH 15

#define ROTTH 4

typedef enum Zones {FAR, CLOSE, INGETBALL,  NA} Zones;

class Moves{
    public:
        Moves(Driver* driver);

        void GetBall(int r, int theta, uint16_t speed, Zones* zone);

        void RotateToZero(int e);
    private:
        Driver* driver;
        float pve = 0;
};
