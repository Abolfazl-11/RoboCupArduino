#include "./Motors.hpp"

Motor::Motor(HardwareTimer* tim, int channle, uint16_t enable_pin) {
    this->tim = tim;
    this->channle = channle;
    this->enable_pin = enable_pin;
    pinMode(enable_pin, OUTPUT);
    digitalWrite(enable_pin, LOW);
}

void Motor::setSpeed(const int speed, const int enable) {
    if (enable) {
        digitalWrite(enable_pin, HIGH);
        tim->setCaptureCompare(channle, 100 - speed, PERCENT_COMPARE_FORMAT);
    }
    else {
        digitalWrite(enable_pin, LOW);
        tim->setCaptureCompare(channle, speed, PERCENT_COMPARE_FORMAT);
    }
}

Driver::Driver(Motor* motor1, Motor* motor2, Motor* motor3, Motor* motor4) {
    this->m1 = motor1;
    this->m2 = motor2;
    this->m3 = motor3;
    this->m4 = motor4;
}

void Driver::gotoPoint(int theta, int speed) {
    theta -= 45;
    
    double x = sin(theta * DEG_TO_RAD);
    double y = cos(theta * DEG_TO_RAD);

    m1->setSpeed((int)(abs(y) * speed), (y >= 0 ? 0 : 1));
    m2->setSpeed((int)(abs(x) * speed), (x >= 0 ? 1 : 0));
    m3->setSpeed((int)(abs(y) * speed + abs(y) * ADDED), (y >= 0 ? 0 : 1));
    m4->setSpeed((int)(abs(x) * speed + abs(y) * ADDED), (x >= 0 ? 1 : 0));
}

void Driver::Rotate(int dir, int speed) {
    if (dir) {
        m1->setSpeed(speed, 0);
        m2->setSpeed(speed, 0);
        m3->setSpeed(speed, 1);
        m4->setSpeed(speed, 1);
    }
    else {
        m1->setSpeed(speed, 1);
        m2->setSpeed(speed, 1);
        m3->setSpeed(speed, 0);
        m4->setSpeed(speed, 0);
    }
}

void Driver::Brake() {
    m1->tim->setCaptureCompare(m1->channle, 100, PERCENT_COMPARE_FORMAT);
    digitalWrite(m1->enable_pin, HIGH);
    m2->tim->setCaptureCompare(m2->channle, 100, PERCENT_COMPARE_FORMAT);
    digitalWrite(m2->enable_pin, HIGH);
    m3->tim->setCaptureCompare(m3->channle, 100, PERCENT_COMPARE_FORMAT);
    digitalWrite(m3->enable_pin, HIGH);
    m4->tim->setCaptureCompare(m4->channle, 100, PERCENT_COMPARE_FORMAT);
    digitalWrite(m4->enable_pin, HIGH);
}
