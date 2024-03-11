// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "../I2Cdev/I2Cdev.h"

#include "./MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#ifndef MPU6050DMP6_
#define MPU6050DMP6_

typedef struct DMP_DATA {
    double yaw;
    double pitch;
    double roll;
} DMP_DATA;

int setupMPU6050DMP(int citer);

void getDMPData(DMP_DATA* d);

#endif
