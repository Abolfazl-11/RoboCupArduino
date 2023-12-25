#include <HCSR04.h>
#include <Arduino.h>

class SR {
    public:
        SR(int trigpin, int echopin);
        double read();
        int trigpin;
        int echopin;
};