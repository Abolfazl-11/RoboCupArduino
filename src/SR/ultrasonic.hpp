#include <Arduino.h>
#include <HCSR04.h>

#ifndef _UltraSonic
#define _UltraSonic

class SR {
    public:
        byte trigPin;
        byte echoPin;

        SR(byte trigPin, byte echoPin);

        int distance;

        void measureDis();
};

class UltraSonic {
    public:
        SR* sr1;
        SR* sr2;
        SR* sr3;
        SR* sr4;

        UltraSonic();
};

#endif
