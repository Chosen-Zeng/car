#ifndef __USER_H
#define __USER_H

extern void BitSet(unsigned *reg_addr, unsigned data);
extern void BitReset(unsigned *reg_addr, unsigned data);

#include <stdbool.h>
#include <math.h>

#include "stm32f1xx.h"

#include "algorithm.h"
#include "fltr.h"

#define TIMsw TIM2
#include "TIM.h"
#include "USART.h"

extern unsigned short task_intvl_ms_State;

extern unsigned char RxData[8];

enum state_t
{
    IDLE,
    SEARCH,
    REACH,
    FETCH,
    PLACE,
};
extern enum state_t state_chassis, state_roboticarm;

struct obj_t
{
    float dist_cm, dist_cm_x, dist_cm_y;
    MovAvgFltr_t dist_cm_y_fltr, dist_cm_x_fltr;
};
extern struct obj_t obj;

struct state_W_t
{
    unsigned char obj_detect : 1;
};
extern struct state_W_t state_W;

// declare task function
void State_Chassis(void);
void State_RoboticArm(void);
void Err(unsigned char breath_period);
void Comm(void);

extern unsigned short task_cnt_ms_State;
extern unsigned short task_cnt_ms_Err;

#endif