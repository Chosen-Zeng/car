#include "usr.h"

#define RxData_LEN 8

enum state_t state_chassis, state_roboticarm;

unsigned char RxData[RxData_LEN];

struct obj_t obj;

struct state_W_t state_W;

void TIM4_Init(void)
{
    TIM4->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
    TIM4->CR1 |= TIM_CR1_CEN;
}

void TIM3_Init(void)
{
    TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
    TIM3->CR1 |= TIM_CR1_CEN;
}

void TIMsw_Init(void)
{
    TIM2->CR1 |= TIM_CR1_CEN;
}

void OS_Tick_Init(void)
{
    TIM1->DIER |= TIM_DIER_UIE;
    TIM1->CR1 |= TIM_CR1_CEN;
}

void USART1_Init(void)
{
    USART1->CR3 |= USART_CR3_DMAR;
    DMA1_Channel5->CCR |= DMA_CCR_TCIE;
    DMA1_Channel5->CNDTR = RxData_LEN;
    DMA1_Channel5->CMAR = RxData;
    DMA1_Channel5->CPAR = &USART1->DR;
}

void PeriphInit(void)
{
    TIM4_Init();
    TIM3_Init();
    TIMsw_Init();
    USART1_Init();
}

// task period/frequency
#define TASK_PERIOD_ms_State 1
#define TASK_FREQ_Err_OBJ_UNDETECT 1
#define TASK_FREQ_Err_OBJ_DETECT 2

unsigned short task_cnt_ms_State, task_cnt_ms_Err;

void Scheduler(void)
{
    OS_Tick_Init();

    while (1)
    {
        // state
        if (task_cnt_ms_State >= TASK_PERIOD_ms_State)
        {
            task_cnt_ms_State = 0;

            State_Chassis();
            State_RoboticArm();
        }

        // error
        static unsigned short task_freq_Err = TASK_FREQ_Err_OBJ_UNDETECT;
        if (state_chassis == IDLE && state_roboticarm == IDLE && task_cnt_ms_Err >= 1) // 1000Hz
        {
            task_cnt_ms_Err = 0;

            Err(3);
        }
        else
        {
            task_freq_Err = state_W.obj_detect ? TASK_FREQ_Err_OBJ_DETECT
                                               : TASK_FREQ_Err_OBJ_UNDETECT;

            if (task_cnt_ms_Err >= 1000 / task_freq_Err / 2)
            {
                task_cnt_ms_Err = 0;

                Err(0);
            }
        }
    }
}