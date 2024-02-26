#include "sr.hpp"

SR::SR(int trigpin, int echopin) {
    this->trigpin = trigpin;
    this->echopin = echopin;
    pinMode(trigpin,OUTPUT);
};
double SR::read() {
    digitalWrite(trigpin,HIGH);
    delayMicroseconds(10);
    digitalWrite(trigpin,LOW);
    int t = 0;
    while(pulseIn(echopin,HIGH)){
        t++;
        delayMicroseconds(1);
    }

    return (double)t / 34;
    
};