#include "Moves.hpp"

Moves::Moves(Driver* driver) {
    this->driver = driver;
}

void Moves::GetBall(int r, int theta, uint16_t speed, Zones* zone) {
    if (r >= ZONEDISTH) {
        *zone = FAR;
    }
    else if (r < ZONEDISTH) {
        if (abs(theta) <= GETBALLANGLETH) {
            *zone = INGETBALL;
        }
        else {
            *zone = CLOSE;
        }
    }
    else {
        *zone = NA;
    }

    switch (*zone) {
        case FAR:
            driver->gotoPoint(theta, speed);
            break;
        case CLOSE:
            if (theta > 0) {
                driver->gotoPoint(theta + 30, speed);
            }
            else {
                driver->gotoPoint(theta - 30, speed);
            }
            break;
        case INGETBALL:
            driver->gotoPoint(0, speed);
            break;
        case NA:
            break;
    }
}

void Moves::RotateToZero(int e) {
    if (abs(e) < ROTTH) {
        driver->m1->setSpeed(driver->m1->current_speed, driver->m1->enable);
        driver->m2->setSpeed(driver->m2->current_speed, driver->m2->enable);
        driver->m3->setSpeed(driver->m3->current_speed, driver->m3->enable);
        driver->m4->setSpeed(driver->m4->current_speed, driver->m4->enable);
        return;
    }
    int d = e > 0 ? LEFT : RIGHT;
    e = abs(e);
    uint32_t u = abs(Kp * e + Ki * e * TIME) + Kd * (e - abs(pve));

    if (u > MAXROTATESPEED) u = MAXROTATESPEED;

    if (driver->m1->current_speed > 0 || driver->m2->current_speed > 0 || driver->m2->current_speed > 0 || driver->m2->current_speed > 0) {
        u *= MOVEINGPIDMULT;
    }

    driver->Rotate(d, u);

    pve = e;
}
