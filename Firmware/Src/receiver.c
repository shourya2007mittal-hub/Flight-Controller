#include "receiver.h"

float rxThrottle = 1200;
float rxRoll = 0;
float rxPitch = 0;
float rxYaw = 0;

void Receiver_Read(void)
{
    rxThrottle = 1300;

    rxRoll = 0;
    rxPitch = 0;
    rxYaw = 0;
}
