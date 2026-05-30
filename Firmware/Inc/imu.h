#ifndef IMU_H
#define IMU_H

#include "stm32f4xx_hal.h"

void MPU6050_Init(void);

void MPU6050_Read_All(float* ax,
                      float* ay,
                      float* az,
                      float* gx,
                      float* gy,
                      float* gz);

void IMU_ComputeAngles(float ax,
                       float ay,
                       float az,
                       float gx,
                       float gy,
                       float gz,
                       float dt,
                       float* roll,
                       float* pitch,
                       float* yaw);

#endif
