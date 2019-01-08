
#ifndef Accel_GPU_Control
#define Accel_GPU_Control


#include <MPU6050_tockn.h>
#include <Wire.h>

#include <PID_v1.h>

#include <stdio.h>
#include <Servo.h>
//#include "Calibration.h"
//#include "AngleCalculation.h"

extern double GyroX, GyroY, GyroZ;

extern double AccelX, AccelY, AccelZ;

void ReadSensors();

#endif
