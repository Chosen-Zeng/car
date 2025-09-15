#include "usr.h"

void TIM1_UP_IRQHandler(void)
{
    if (TIM1->SR & TIM_SR_UIF)
    {
        TIM1->SR &= ~TIM_SR_UIF;

        ++task_intvl_ms_State;
    }
}

void DMA1_Channel5_IRQHandler(void)
{
    if (DMA1->ISR & DMA_ISR_TCIF5)
    {
        DMA1->IFCR |= DMA_IFCR_CTCIF5;

        // data process
    }
}