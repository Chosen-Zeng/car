#include "usr.h"

void State_Chassis(void)
{

    // control
    {
        // todo: lock wheel after certain time
        for (unsigned char cnt = 0; cnt < 4; ++cnt)
        {
            if (wheel[cnt].spd_pct > 0)
            {
                wheel[cnt].port_N->ODR &= ~wheel[cnt].pin_N;
                wheel[cnt].port_P->ODR |= wheel[cnt].pin_P;
            }
            else if (wheel[cnt].spd_pct < 0)
            {
                wheel[cnt].port_N->ODR |= wheel[cnt].pin_N;
                wheel[cnt].port_P->ODR &= ~wheel[cnt].pin_P;
            }
            else
            {
                wheel[cnt].port_N->ODR &= ~wheel[cnt].pin_N;
                wheel[cnt].port_P->ODR &= ~wheel[cnt].pin_P;
            }

            *wheel[cnt].channel = (TIM_CHASSIS->ARR + 1) * wheel[cnt].spd_pct;
        }
    }
}

void State_RoboticArm(void)
{
}