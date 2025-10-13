#include "usr.h"

#define Err_LED_PORT GPIOC
#define Err_LED_PIN GPIO_ODR_ODR13

#define REFRESH_PERIOD_ms 10
#define BRIGHTNESS_LEVEL 10

void Err(unsigned char breath_period)
{
    static unsigned char refresh_cnt, brightness_level, repetition;
    static bool descend;

    if (!breath_period)
    {
        Err_LED_PORT->ODR ^= Err_LED_PIN;

        repetition = brightness_level = refresh_cnt = 0;
        descend = false;
    }
    else
    {
        if (++refresh_cnt == REFRESH_PERIOD_ms)
        {
            refresh_cnt = 0;

            if (++repetition == breath_period * (1000 / REFRESH_PERIOD_ms) / REFRESH_PERIOD_ms / 2)
            {
                repetition = 0;

                descend ? --brightness_level
                        : ++brightness_level;

                if (brightness_level == BRIGHTNESS_LEVEL)
                    descend = true;
                else if (!brightness_level)
                    descend = false;
            }
        }

        refresh_cnt < brightness_level ? (Err_LED_PORT->ODR |= Err_LED_PIN)
                                       : (Err_LED_PORT->ODR &= ~Err_LED_PIN);
    }
}