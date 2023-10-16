#include <Arduino.h>

#ifndef _Pixy
#define _Pixy

#define ZERO_X 162
#define ZERO_Y 104
#define WIDTH 211 
#define HEIGHT 207

typedef struct BallTransform {
    int x;
    int y;
} BallTransform;

class Pixy {
    public:
        BallTransform* transform;
        int detected = 0;

        void getBallPos();
};

#endif
