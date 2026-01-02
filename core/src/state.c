#include "stm32f103xb.h"
#include "usr.h"

#include "TIM.h"
#include "algorithm.h"
#include <math.h>

#define TBD 0

#define TIM_CHASSIS    TIM4
#define WHEEL_LF_arrID 0
#define WHEEL_RF_arrID 1
#define WHEEL_RR_arrID 2
#define WHEEL_LR_arrID 3

#define REACH_DIST_CM     TBD
#define REACH_DEADBAND_CM TBD

void State_Chassis(void) {
    static enum state_t state_last_chassis;

    static TIMsw_t runtime_chassis;

    static struct wheel_t {
        GPIO_TypeDef *const port_N, *const port_P;
        const unsigned pin_N, pin_P;
        volatile unsigned *const channel;
        float spd_pct;
    } wheel[4] = {
        [WHEEL_LF_arrID] = {.port_P = GPIOB, .pin_P = GPIO_ODR_ODR5, .port_N = GPIOB, .pin_N = GPIO_ODR_ODR4, .channel = &TIM_CHASSIS->CCR1},
        [WHEEL_RF_arrID] = {.port_P = GPIOB, .pin_P = GPIO_ODR_ODR3, .port_N = GPIOA, .pin_N = GPIO_ODR_ODR15, .channel = &TIM_CHASSIS->CCR2},
        [WHEEL_RR_arrID] = {.port_P = GPIOA, .pin_P = GPIO_ODR_ODR8, .port_N = GPIOB, .pin_N = GPIO_ODR_ODR15, .channel = &TIM_CHASSIS->CCR3},
        [WHEEL_LR_arrID] = {.port_P = GPIOB, .pin_P = GPIO_ODR_ODR14, .port_N = GPIOB, .pin_N = GPIO_ODR_ODR13, .channel = &TIM_CHASSIS->CCR4},
    };

    static struct spd_t {
        float x, y, z;
    } spd_pct;

    switch (state_chassis) {
    case IDLE: {
        if (state_last_chassis != state_chassis) {
            if (state_last_chassis != SEARCH) {
                TIMsw_Clear(&runtime_chassis);

                state_last_chassis = state_chassis;
            }
        }

        spd_pct.z = spd_pct.y = spd_pct.x = 0;

        if (state_last_chassis != SEARCH && TIMsw_CheckTimeout(&runtime_chassis, 3)) {
            state_chassis = SEARCH;
        }

        break;
    }
    case SEARCH: {
        if (state_last_chassis != state_chassis) {
            TIMsw_Clear(&runtime_chassis);

            state_last_chassis = state_chassis;
        }

        if (state_W.obj_detect) {
            state_chassis = REACH;
            break;
        }

        if (TIMsw_CheckTimeout(&runtime_chassis, 3))
            state_chassis = IDLE;

        spd_pct.y = spd_pct.x = 0;
        spd_pct.z = 0.2;

        break;
    }
    case REACH: {
        if (state_last_chassis != state_chassis) {

            state_last_chassis = state_chassis;
        }

        if (!state_W.obj_detect) {
            state_chassis = SEARCH;
            break;
        }

        spd_pct.x = (MovAvgFltr_GetData(&obj.dist_cm_x_fltr) - REACH_DIST_CM) * 0.1;
        spd_pct.y = MovAvgFltr_GetData(&obj.dist_cm_y_fltr) * 0.1;
        spd_pct.z = -MovAvgFltr_GetData(&obj.dist_cm_x_fltr) * 0.1;

        break;
    }
    }

    // calculation
    {
        wheel[WHEEL_LF_arrID].spd_pct = spd_pct.x + spd_pct.y + spd_pct.z;
        wheel[WHEEL_RF_arrID].spd_pct = -spd_pct.x + spd_pct.y - spd_pct.z;
        wheel[WHEEL_RR_arrID].spd_pct = spd_pct.x + spd_pct.y - spd_pct.z;
        wheel[WHEEL_LR_arrID].spd_pct = -spd_pct.x + spd_pct.y + spd_pct.z;
    }

    // control
    {
        for (unsigned char cnt = 0; cnt < 4; ++cnt) {
            if (wheel[cnt].spd_pct > 0) {
                wheel[cnt].port_P->ODR |= wheel[cnt].pin_P;
                wheel[cnt].port_N->ODR &= ~wheel[cnt].pin_N;
            } else if (wheel[cnt].spd_pct < 0) {
                wheel[cnt].port_P->ODR &= ~wheel[cnt].pin_P;
                wheel[cnt].port_N->ODR |= wheel[cnt].pin_N;
            } else // if (!wheel[cnt].spd_pct), spd == 0, set both pin low
            {
                wheel[cnt].port_P->ODR &= ~wheel[cnt].pin_P;
                wheel[cnt].port_N->ODR &= ~wheel[cnt].pin_N;
            }

            *wheel[cnt].channel = (TIM_CHASSIS->ARR + 1) * ABS(wheel[cnt].spd_pct);
        }
    }
}

#define TIM_ROBOTICARM TIM3

#define ROBOTICARM_HEIGHT_cm       15
#define ROBOTICARM_FETCH_HEIGHT_cm 2.5

static struct joint_t {
    volatile unsigned *const channel;
    const float arm_len_cm;
    float angle;
} joint[4] = {
    {.channel = &TIM_ROBOTICARM->CCR1, .arm_len_cm = 8.5},
    {.channel = &TIM_ROBOTICARM->CCR2, .arm_len_cm = 6},
    {.channel = &TIM_ROBOTICARM->CCR3, .arm_len_cm = 13.5},
    {.channel = &TIM_ROBOTICARM->CCR4},
};

#define JOINT_ANGLE_CLAMP   20
#define JOINT_ANGLE_UNCLAMP 90

#define JOINT_ANGLE_PRESET_IDLE_0 -75
#define JOINT_ANGLE_PRESET_IDLE_1 150
#define JOINT_ANGLE_PRESET_IDLE_2 -15

#define JOINT_ANGLE_PRESET_PLACE_0 0
#define JOINT_ANGLE_PRESET_PLACE_1 -45
#define JOINT_ANGLE_PRESET_PLACE_2 90

void State_RoboticArm(void) {

    static const char joint_angle_offset[4] = {-15, 0, 0, 0};

    static enum state_t state_last_roboticarm;

    static TIMsw_t runtime_roboticarm;

    switch (state_roboticarm) {
    case IDLE: {
        if (state_last_roboticarm != state_roboticarm) {

            state_last_roboticarm = state_roboticarm;
        }

        // set angle
        joint[0].angle = JOINT_ANGLE_PRESET_IDLE_0 * 2,
        joint[1].angle = JOINT_ANGLE_PRESET_IDLE_1,
        joint[2].angle = JOINT_ANGLE_PRESET_IDLE_2,
        joint[3].angle = JOINT_ANGLE_UNCLAMP;

        if (state_W.obj_detect)
            state_roboticarm = FETCH;

        break;
    }
    case FETCH: {
        if (state_last_roboticarm != state_roboticarm) {
            TIMsw_Clear(&runtime_roboticarm);

            state_last_roboticarm = state_roboticarm;
        }

        if (!state_W.obj_detect) {
            state_roboticarm = IDLE;
            break;
        }

        if (obj.dist_cm < 10 && MovAvgFltr_GetStatus(&obj.dist_cm_x_fltr, TBD) && MovAvgFltr_GetStatus(&obj.dist_cm_y_fltr, TBD)) {

            float dist_0_2_cm = hypot(joint[2].arm_len_cm + ROBOTICARM_FETCH_HEIGHT_cm - ROBOTICARM_HEIGHT_cm, obj.dist_cm);

            joint[0].angle = (acos(obj.dist_cm / dist_0_2_cm)
                              + acos((pow(dist_0_2_cm, 2) + pow(joint[0].arm_len_cm, 2) - pow(joint[1].arm_len_cm, 2)) / (2 * dist_0_2_cm * joint[0].arm_len_cm)))
                           * R2D * 2;
            joint[1].angle = acos((pow(joint[0].arm_len_cm, 2) + pow(joint[1].arm_len_cm, 2) - pow(dist_0_2_cm, 2)) / (2 * joint[0].arm_len_cm * joint[1].arm_len_cm)) * R2D;
            joint[2].angle = (asin(obj.dist_cm / dist_0_2_cm)
                              + acos((pow(joint[1].arm_len_cm, 2) + pow(dist_0_2_cm, 2) - pow(joint[0].arm_len_cm, 2)) / (2 * joint[1].arm_len_cm * dist_0_2_cm)))
                               * R2D
                           - 180;
            joint[3].angle = TIMsw_CheckTimeout(&runtime_roboticarm, 1.5) ? JOINT_ANGLE_CLAMP
                                                                          : JOINT_ANGLE_UNCLAMP;

            if (TIMsw_CheckTimeout(&runtime_roboticarm, 3)) {
                state_roboticarm = PLACE;
            }
        } else
            TIMsw_Clear(&runtime_roboticarm);

        break;
    }
    case PLACE: {
        if (state_last_roboticarm != state_roboticarm) {
            TIMsw_Clear(&runtime_roboticarm);
            state_W.obj_detect = false;

            state_last_roboticarm = state_roboticarm;
        }

        joint[0].angle = JOINT_ANGLE_PRESET_PLACE_0 * 2,
        joint[1].angle = JOINT_ANGLE_PRESET_PLACE_1,
        joint[2].angle = JOINT_ANGLE_PRESET_PLACE_2,
        joint[3].angle = TIMsw_CheckTimeout(&runtime_roboticarm, 1.5) ? JOINT_ANGLE_UNCLAMP
                                                                      : JOINT_ANGLE_CLAMP;

        if (TIMsw_CheckTimeout(&runtime_roboticarm, 3)) {
            state_roboticarm = IDLE;
        }

        break;
    }
    }

    // control
    {
        for (unsigned char cnt = 0; cnt < 4; ++cnt)
            *joint[cnt].channel = (TIM_ROBOTICARM->ARR + 1) * ((joint[cnt].angle + joint_angle_offset[cnt] + 180) / 360 / 10 + 0.025);
    }
}
