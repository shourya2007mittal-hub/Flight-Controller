#include "imu.h"
#include "math.h"

extern I2C_HandleTypeDef hi2c1;

#define MPU6050_ADDR (0x68 << 1)
#define PWR_MGMT_1   0x6B
#define ACCEL_XOUT_H 0x3B

static float gyroRoll;
static float gyroPitch;
static float gyroYaw;

void MPU6050_Init(void)
{
    uint8_t data = 0;

    HAL_I2C_Mem_Write(&hi2c1,
                      MPU6050_ADDR,
                      PWR_MGMT_1,
                      1,
                      &data,
                      1,
                      100);
}

void MPU6050_Read_All(float* ax,
                      float* ay,
                      float* az,
                      float* gx,
                      float* gy,
                      float* gz)
{
    uint8_t buffer[14];

    HAL_I2C_Mem_Read(&hi2c1,
                     MPU6050_ADDR,
                     ACCEL_XOUT_H,
                     1,
                     buffer,
                     14,
                     100);

    int16_t rawAx = buffer[0] << 8 | buffer[1];
    int16_t rawAy = buffer[2] << 8 | buffer[3];
    int16_t rawAz = buffer[4] << 8 | buffer[5];

    int16_t rawGx = buffer[8] << 8 | buffer[9];
    int16_t rawGy = buffer[10] << 8 | buffer[11];
    int16_t rawGz = buffer[12] << 8 | buffer[13];

    *ax = rawAx / 16384.0f;
    *ay = rawAy / 16384.0f;
    *az = rawAz / 16384.0f;

    *gx = rawGx / 131.0f;
    *gy = rawGy / 131.0f;
    *gz = rawGz / 131.0f;
}

void IMU_ComputeAngles(float ax,
                       float ay,
                       float az,
                       float gx,
                       float gy,
                       float gz,
                       float dt,
                       float* roll,
                       float* pitch,
                       float* yaw)
{
    float accelRoll;
    float accelPitch;

    accelRoll = atan2f(ay, az) * 57.2958f;

    accelPitch = atan2f(-ax,
                        sqrtf(ay * ay + az * az)) * 57.2958f;

    gyroRoll += gx * dt;
    gyroPitch += gy * dt;
    gyroYaw += gz * dt;

    *roll = 0.98f * gyroRoll + 0.02f * accelRoll;
    *pitch = 0.98f * gyroPitch + 0.02f * accelPitch;
    *yaw = gyroYaw;
}
