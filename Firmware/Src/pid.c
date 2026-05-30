#include "pid.h"

void PID_Init(PID_t* pid,
              float kp,
              float ki,
              float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->integral = 0;
    pid->previousError = 0;
}

float PID_Compute(PID_t* pid,
                  float error,
                  float dt)
{
    pid->integral += error * dt;

    float derivative = (error - pid->previousError) / dt;

    float output = pid->kp * error +
                   pid->ki * pid->integral +
                   pid->kd * derivative;

    pid->previousError = error;

    return output;
}
