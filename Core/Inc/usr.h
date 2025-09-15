#ifndef __USER_H
#define __USER_H

#include <stdbool.h>
#include "stm32f1xx.h"

#define TIMsw TIM2
#include "TIM.h"
#include "USART.h"

extern unsigned short task_intvl_ms_State;

extern unsigned char RxData[8];

extern USART_handle_t USART1_handle;

typedef struct
{
    GPIO_TypeDef *port_N, *const port_P;
    const unsigned pin_N, pin_P;
    volatile unsigned *const channel;
    float spd_pct;
} wheel_t;
extern wheel_t wheel[4];

#define TIM_CHASSIS TIM4
#define WHEEL_LF_arrID 0
#define WHEEL_LR_arrID 1
#define WHEEL_RF_arrID 2
#define WHEEL_RR_arrID 3

// call in main.c
void PeriphInit(void);
void Scheduler(void);

// declare task function
void State_Chassis(void);
void State_RoboticArm(void);
void Err(void);
void Comm(void);

// define task period
#define TASK_INTVL_ms_State 1

extern unsigned short task_intvl_ms_State;

#endif