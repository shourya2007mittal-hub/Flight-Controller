#include "motors.h"

extern TIM_HandleTypeDef htim1;

void Motors_Init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

void Motors_Write(uint8_t motor, float value)
{
    if(value < 1000)
        value = 1000;

    if(value > 2000)
        value = 2000;

    switch(motor)
    {
        case 0:
            __HAL_TIM_SET_COMPARE(&htim1,
                                  TIM_CHANNEL_1,
                                  value);
            break;

        case 1:
            __HAL_TIM_SET_COMPARE(&htim1,
                                  TIM_CHANNEL_2,
                                  value);
            break;

        case 2:
            __HAL_TIM_SET_COMPARE(&htim1,
                                  TIM_CHANNEL_3,
                                  value);
            break;

        case 3:
            __HAL_TIM_SET_COMPARE(&htim1,
                                  TIM_CHANNEL_4,
                                  value);
            break;
    }
}
