#ifndef MOTORS_H
#define MOTORS_H

#include "stm32f4xx_hal.h"

void Motors_Init(void);
void Motors_Write(uint8_t motor, float value);

#endif
