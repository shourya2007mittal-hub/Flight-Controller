#include "main.h"
#include "imu.h"
#include "pid.h"
#include "motors.h"
#include "receiver.h"

float roll;
float pitch;
float yaw;

float accelX;
float accelY;
float accelZ;

float gyroX;
float gyroY;
float gyroZ;

PID_t pidRoll;
PID_t pidPitch;
PID_t pidYaw;

float throttle;

uint32_t previousTime;
float dt;

int main(void)
{
    HAL_Init();

    Motors_Init();

    MPU6050_Init();

    PID_Init(&pidRoll, 1.4f, 0.02f, 0.04f);
    PID_Init(&pidPitch, 1.4f, 0.02f, 0.04f);
    PID_Init(&pidYaw, 2.0f, 0.00f, 0.00f);

    previousTime = HAL_GetTick();

    while(1)
    {
        uint32_t currentTime = HAL_GetTick();

        dt = (currentTime - previousTime) / 1000.0f;

        previousTime = currentTime;

        Receiver_Read();

        throttle = rxThrottle;

        MPU6050_Read_All(&accelX,
                         &accelY,
                         &accelZ,
                         &gyroX,
                         &gyroY,
                         &gyroZ);

        IMU_ComputeAngles(accelX,
                          accelY,
                          accelZ,
                          gyroX,
                          gyroY,
                          gyroZ,
                          dt,
                          &roll,
                          &pitch,
                          &yaw);

        float rollError = rxRoll - roll;
        float pitchError = rxPitch - pitch;
        float yawError = rxYaw - gyroZ;

        float rollPID = PID_Compute(&pidRoll,
                                    rollError,
                                    dt);

        float pitchPID = PID_Compute(&pidPitch,
                                     pitchError,
                                     dt);

        float yawPID = PID_Compute(&pidYaw,
                                   yawError,
                                   dt);

        float motor1 = throttle + pitchPID + rollPID - yawPID;
        float motor2 = throttle + pitchPID - rollPID + yawPID;
        float motor3 = throttle - pitchPID + rollPID + yawPID;
        float motor4 = throttle - pitchPID - rollPID - yawPID;

        Motors_Write(0, motor1);
        Motors_Write(1, motor2);
        Motors_Write(2, motor3);
        Motors_Write(3, motor4);

        HAL_Delay(1);
    }
}
