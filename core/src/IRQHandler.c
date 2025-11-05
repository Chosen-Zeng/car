#include "usr.h"

void SysTick_Handler(void) {
    ++task_cnt_ms_State;
    ++task_cnt_ms_Err;
}

void DMA1_Channel5_IRQHandler(void) {
    if (DMA1->ISR & DMA_ISR_TCIF5) {
        DMA1->IFCR |= DMA_IFCR_CTCIF5;

        obj.dist_cm_x = *(float *)RxData;
        obj.dist_cm_y = *(float *)&RxData[4];
        obj.dist_cm = hypot(MovAvgFltr(&obj.dist_cm_x_fltr, obj.dist_cm_x), MovAvgFltr(&obj.dist_cm_y_fltr, obj.dist_cm_y));

        state_W.obj_detect = true;
    }
}