#include "./Motors.hpp"

Motor::Motor(HardwareTimer* tim, int channle, uint16_t enable_pin, bool r, float added) {
    this->tim = tim;
    this->channle = channle;
    this->enable_pin = enable_pin;
    this->reverse = r;
    this->enable = reverse;
    this->added = added;
    pinMode(enable_pin, OUTPUT);
    digitalWrite(enable_pin, LOW);
}

void Motor::setSpeed(const int speed, const int enable) {
    if (enable) {
        digitalWrite(enable_pin, HIGH);
        tim->setCaptureCompare(channle, 100 - ((speed <= MAXSPEED ? speed : MAXSPEED) * added), PERCENT_COMPARE_FORMAT);
    }
    else {
        digitalWrite(enable_pin, LOW);
        tim->setCaptureCompare(channle, ((speed <= MAXSPEED ? speed : MAXSPEED) * added), PERCENT_COMPARE_FORMAT);
    }
    this->enable = enable;
    this->current_speed = speed;
}

Driver::Driver(Motor* motor1, Motor* motor2, Motor* motor3, Motor* motor4) {
    this->m1 = motor1;
    this->m2 = motor2;
    this->m3 = motor3;
    this->m4 = motor4;
}

void Driver::gotoPoint(int theta, int speed) {
    current_theta = theta;
    theta -= 45;
    
    double x = sin(theta * DEG_TO_RAD);
    double y = cos(theta * DEG_TO_RAD);

    m1->setSpeed((int)(abs(y) * speed), (y >= 0 ? m1->reverse : !m1->reverse));
    m2->setSpeed((int)(abs(x) * speed), (x >= 0 ? !m2->reverse : m2->reverse));
    m3->setSpeed((int)(abs(y) * speed), (y >= 0 ? m3->reverse : !m3->reverse));
    m4->setSpeed((int)(abs(x) * speed), (x >= 0 ? !m4->reverse : m4->reverse));

    isMoving = true;
}

void Driver::Rotate(int dir, int speed) {
    if (dir) {
        // Motor 1
        if (m1->enable == m1->reverse) {
            m1->setSpeed(m1->current_speed + speed, m1->enable);
            m1->current_speed -= speed;
        }
        else {
            if (m1->current_speed >= speed) {
                m1->setSpeed(m1->current_speed - speed, m1->enable);
                m1->current_speed += speed;
            }
            else {
                m1->setSpeed(abs(m1->current_speed - speed), !m1->enable);
                m1->current_speed = speed - m1->current_speed;
                m1->enable = !m1->enable;
            }
        }

        // Motor 2
        if (m2->enable == m2->reverse) {
            m2->setSpeed(m2->current_speed + speed, m2->enable);
            m2->current_speed -= speed;
        }
        else {
            if (m2->current_speed >= speed) {
                m2->setSpeed(m2->current_speed - speed, m2->enable);
                m2->current_speed += speed;
            }
            else {
                m2->setSpeed(abs(m2->current_speed - speed), !m2->enable);
                m2->current_speed = speed - m2->current_speed;
                m2->enable = !m2->enable;
            }
        }

        // Motor 3
        if (m3->enable == m3->reverse) {
            if (m3->current_speed >= speed) {
                m3->setSpeed(m3->current_speed - speed, m3->enable);
                m3->current_speed += speed;
            }
            else {
                m3->setSpeed(abs(m3->current_speed - speed), !m3->enable);
                m3->current_speed = speed - m3->current_speed;
                m3->enable = !m3->enable;
            }
        }
        else {
            m3->setSpeed(m3->current_speed + speed, m3->enable);
            m3->current_speed -= speed;
        }

        // Motor 4
        if (m4->enable == m4->reverse) {
            if (m4->current_speed >= speed) {
                m4->setSpeed(m4->current_speed - speed, m4->enable);
                m4->current_speed += speed;
            }
            else {
                m4->setSpeed(abs(m4->current_speed - speed), !m4->enable);
                m4->current_speed = speed - m4->current_speed;
                m4->enable = !m4->enable;
            }
        }
        else {
            m4->setSpeed(m4->current_speed + speed, m4->enable);
            m4->current_speed -= speed;
        }
    }
    else {
        // Motor 1
        if (m1->enable == m1->reverse) {
            if (m1->current_speed >= speed) {
                m1->setSpeed(m1->current_speed - speed, m1->enable);
                m1->current_speed += speed;
            }
            else {
                m1->setSpeed(abs(m1->current_speed - speed), !m1->enable);
                m1->current_speed = speed - m1->current_speed;
                m1->enable = !m1->enable;
            }
        }
        else {
            m1->setSpeed(m1->current_speed + speed, m1->enable);
            m1->current_speed -= speed;
        }

        // Motor 2
        if (m2->enable == m2->reverse) {
            if (m2->current_speed >= speed) {
                m2->setSpeed(m2->current_speed - speed, m2->enable);
                m2->current_speed += speed;
            }
            else {
                m2->setSpeed(abs(m2->current_speed - speed), !m2->enable);
                m2->current_speed = speed - m2->current_speed;
                m2->enable = !m2->enable;
            }
        }
        else {
            m2->setSpeed(m2->current_speed + speed, m2->enable);
            m2->current_speed -= speed;
        }

        // Motor 3
        if (m3->enable == m3->reverse) {
            m3->setSpeed(m3->current_speed + speed, m3->enable);
            m3->current_speed -= speed;
        }
        else {
            if (m3->current_speed >= speed) {
                m3->setSpeed(m3->current_speed - speed, m3->enable);
                m3->current_speed += speed;
            }
            else {
                m3->setSpeed(abs(m3->current_speed - speed), !m3->enable);
                m3->current_speed = speed - m3->current_speed;
                m3->enable = !m3->enable;
            }
        }

        // Motor 4
        if (m4->enable == m4->reverse) {
            m4->setSpeed(m4->current_speed + speed, m4->enable);
            m4->current_speed -= speed;
        }
        else {
            if (m4->current_speed >= speed) {
                m4->setSpeed(m4->current_speed - speed, m4->enable);
                m4->current_speed += speed;
            }
            else {
                m4->setSpeed(abs(m4->current_speed - speed), !m4->enable);
                m4->current_speed = speed - m4->current_speed;
                m4->enable = !m4->enable;
            }
        }
    }
}

void Driver::Brake() {
    m1->tim->setCaptureCompare(m1->channle, 100, PERCENT_COMPARE_FORMAT);
    digitalWrite(m1->enable_pin, HIGH);
    m1->current_speed = 0;
    m1->enable = m1->reverse;

    m2->tim->setCaptureCompare(m2->channle, 100, PERCENT_COMPARE_FORMAT);
    digitalWrite(m2->enable_pin, HIGH);
    m2->current_speed = 0;
    m2->enable = m2->reverse;

    m3->tim->setCaptureCompare(m3->channle, 100, PERCENT_COMPARE_FORMAT);
    digitalWrite(m3->enable_pin, HIGH);
    m3->current_speed = 0;
    m3->enable = m3->reverse;

    m4->tim->setCaptureCompare(m4->channle, 100, PERCENT_COMPARE_FORMAT);
    digitalWrite(m4->enable_pin, HIGH);
    m4->current_speed = 0;
    m4->enable = m4->reverse;

    isMoving = false;
    current_theta = -3600;
}
