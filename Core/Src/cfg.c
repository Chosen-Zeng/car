#include "usr.h"

unsigned short task_intvl_ms_State;

unsigned char RxData[8];

USART_handle_t USART1_handle = {.USART_handle = USART1, .DMA_handle = DMA1, .DMA_subhandle = DMA1_Channel5, .DMA_ID = 5};

wheel_t wheel[4] =
    {
        [WHEEL_LF_arrID] = {.port_N = GPIOB, .port_P = GPIOB, .pin_N = GPIO_PIN_5, .pin_P = GPIO_PIN_4, .channel = &TIM_CHASSIS->CCR1},
        [WHEEL_LR_arrID] = {.port_N = GPIOB, .port_P = GPIOA, .pin_N = GPIO_PIN_3, .pin_P = GPIO_PIN_15, .channel = &TIM_CHASSIS->CCR2},
        [WHEEL_RF_arrID] = {.port_N = GPIOA, .port_P = GPIOA, .pin_N = GPIO_PIN_12, .pin_P = GPIO_PIN_11, .channel = &TIM_CHASSIS->CCR3},
        [WHEEL_RR_arrID] = {.port_N = GPIOA, .port_P = GPIOB, .pin_N = GPIO_PIN_8, .pin_P = GPIO_PIN_15, .channel = &TIM_CHASSIS->CCR4},
};

void TIM4_Init(void)
{
    TIM4->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
    TIM4->CR1 |= TIM_CR1_CEN;
}

void TIM3_Init(void)
{
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
}

void PeriphInit(void)
{
    TIM4_Init();
    TIM3_Init();
    TIMsw_Init();
    USART1_Init();
}

void Scheduler(void)
{
    OS_Tick_Init();

    while (1)
    {
        // state
        if (task_intvl_ms_State >= TASK_INTVL_ms_State)
        {
            task_intvl_ms_State = 0;

            State_Chassis();
            State_RoboticArm();
        }
    }
}