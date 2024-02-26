#include "HCSR04.hpp"

SR::SR(uint16_t trigpin, uint16_t echopin) {
    this->trigpin = trigpin;
    this->echopin = echopin;
    pinMode(trigpin, OUTPUT);
    pinMode(echopin, INPUT);
}

double SR::readsr() {
    digitalWrite(trigpin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigpin,HIGH);
    delayMicroseconds(10);
    digitalWrite(trigpin,LOW);
    int t = 0;
    while(!digitalRead(echopin));
    while(digitalRead(echopin)) {
        t++;
        delayMicroseconds(1);
        if (t > TIMEOUT) break;
    }

    return (double)t / 25;
}

SR_Controller::SR_Controller(SR* sr1, SR* sr2, SR* sr3, SR* sr4, Position* position) {
    this->sr1=sr1;
    this->sr2=sr2;
    this->sr3=sr3;
    this->sr4=sr4;
    this->position = position;
}

void SR_Controller::ReadAllSRs() {
    double s1 = sr1->readsr();
    double s2 = sr2->readsr();
    //double s3 = sr3->readsr();
    //double s4 = sr4->readsr();
    double s3, s4;

    if (s4 + s3 >= FIELD_LENGHT) {
        position->x = max(s1, s3) - 170;
    }

    if (s2 + s1 >= FIELD_WIDTH) {
        position->y = max(s2, s4) - 90;
    }
}
