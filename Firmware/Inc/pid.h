#ifndef PID_H
#define PID_H

typedef struct
{
    float kp;
    float ki;
    float kd;

    float integral;
    float previousError;

} PID_t;

void PID_Init(PID_t* pid,
              float kp,
              float ki,
              float kd);

float PID_Compute(PID_t* pid,
                  float error,
                  float dt);

#endif
