#include "stm32f103xb.h"
#include "usr.h"

#define RxData_LEN 8

enum state_t state_chassis, state_roboticarm;

unsigned char RxData[RxData_LEN];

struct obj_t obj;

struct state_W_t state_W;

// task period/frequency
#define TASK_PERIOD_ms_State       1
#define TASK_FREQ_Err_OBJ_UNDETECT 1
#define TASK_FREQ_Err_OBJ_DETECT   2

unsigned short task_cnt_ms_State, task_cnt_ms_Err;

int mian(void) {
    BitSet(&FLASH->ACR, FLASH_ACR_LATENCY_2); // increase wait states for higher CPU frequency

    // clock control
    {
        // HSE
        BitSet(&RCC->CR, RCC_CR_HSEON);    // enable HSE
        while (!(RCC->CR & RCC_CR_HSERDY)) // wait till HSE ready
            ;

        // PLL
        BitSet(&RCC->CFGR, RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL9); // configure main PLL clock source and multiplication factor
        BitSet(&RCC->CR, RCC_CR_PLLON);                          // enable main PLL
        while (!(RCC->CR & RCC_CR_PLLRDY))                       // wait till PLL ready
            ;

        // clock divider
        BitSet(&RCC->CFGR, RCC_CFGR_PPRE2_DIV1 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_HPRE_DIV1);

        // system clock source
        BitSet(&RCC->CFGR, RCC_CFGR_SW_PLL);
        while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
            ;
        SystemCoreClockUpdate();

        // HSI
        BitReset(&RCC->CR, RCC_CR_HSION); // disable HSI
        while (RCC->CR & RCC_CR_HSIRDY)   // wait till HSI disabled
            ;
    }

    // DMA
    {
        RCC->AHBENR |= RCC_AHBENR_DMA1EN;

        // USART1 RX
        DMA1_Channel5->CCR |= DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_TCIE | DMA_CCR_EN;
        DMA1_Channel5->CNDTR = RxData_LEN;
        DMA1_Channel5->CMAR = (unsigned)RxData;
        DMA1_Channel5->CPAR = (unsigned)&USART1->DR;
    }

    // GPIO & AFIO
    {
        // SW-DP
        RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
        AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;

        RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_IOPDEN;

        GPIOC->ODR |= GPIO_ODR_ODR13;

        // output push-pull
        GPIOA->CRH = (GPIOA->CRH & ~(GPIO_CRH_CNF15 | GPIO_CRH_MODE15 | GPIO_CRH_CNF8 | GPIO_CRH_MODE8))
                   | (GPIO_CRH_MODE15_1 | GPIO_CRH_MODE8_1);
        GPIOB->CRL = (GPIOB->CRL & ~(GPIO_CRL_CNF5 | GPIO_CRL_MODE5 | GPIO_CRL_CNF4 | GPIO_CRL_MODE4 | GPIO_CRL_CNF3 | GPIO_CRL_MODE3))
                   | GPIO_CRL_MODE5_1 | GPIO_CRL_MODE4_1 | GPIO_CRL_MODE3_1;
        GPIOB->CRH = (GPIOB->CRH & ~(GPIO_CRH_CNF15 | GPIO_CRH_MODE15 | GPIO_CRH_CNF14 | GPIO_CRH_MODE14 | GPIO_CRH_CNF13 | GPIO_CRH_MODE13))
                   | GPIO_CRH_MODE15_1 | GPIO_CRH_MODE14_1 | GPIO_CRH_MODE13_1;
        GPIOC->CRH = (GPIOC->CRH & ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13))
                   | GPIO_CRH_MODE13_1;

        // TIM3 OC, alternate function output push-pull
        GPIOA->CRL = (GPIOA->CRL & ~(GPIO_CRL_CNF7 | GPIO_CRL_MODE7 | GPIO_CRL_CNF6 | GPIO_CRL_MODE6))
                   | GPIO_CRL_CNF7_1 | GPIO_CRL_MODE7_1 | GPIO_CRL_MODE7_0 | GPIO_CRL_CNF6_1 | GPIO_CRL_MODE6_1 | GPIO_CRL_MODE6_0;
        GPIOB->CRL = (GPIOB->CRL & ~(GPIO_CRL_CNF1 | GPIO_CRL_MODE1 | GPIO_CRL_CNF0 | GPIO_CRL_MODE0))
                   | GPIO_CRL_CNF1_1 | GPIO_CRL_MODE1_1 | GPIO_CRL_MODE1_0 | GPIO_CRL_CNF0_1 | GPIO_CRL_MODE0_1 | GPIO_CRL_MODE0_0;

        // TIM4 OC, alternate function output push-pull
        GPIOB->CRL = (GPIOB->CRL & ~(GPIO_CRL_CNF7 | GPIO_CRL_MODE7 | GPIO_CRL_CNF6 | GPIO_CRL_MODE6))
                   | GPIO_CRL_CNF7_1 | GPIO_CRL_MODE7_1 | GPIO_CRL_MODE7_0 | GPIO_CRL_CNF6_1 | GPIO_CRL_MODE6_1 | GPIO_CRL_MODE6_0;
        GPIOB->CRH = (GPIOB->CRH & ~(GPIO_CRH_CNF9 | GPIO_CRH_MODE9 | GPIO_CRH_CNF8 | GPIO_CRH_MODE8))
                   | GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9_1 | GPIO_CRH_MODE9_0 | GPIO_CRH_CNF8_1 | GPIO_CRH_MODE8_1 | GPIO_CRH_MODE8_0;

        // USART1 RX, alternate function output push-pull
        GPIOA->CRH = (GPIOA->CRH & ~(GPIO_CRH_CNF10 | GPIO_CRH_MODE10))
                   | GPIO_CRH_CNF10_1 | GPIO_CRH_MODE10_1 | GPIO_CRH_MODE10_0;

        // analog mode
        GPIOA->CRL &= ~(GPIO_CRL_CNF5 | GPIO_CRL_CNF4 | GPIO_CRL_CNF3 | GPIO_CRL_CNF2 | GPIO_CRL_CNF1 | GPIO_CRL_CNF0);
        GPIOA->CRH &= ~GPIO_CRH_CNF9;
        GPIOB->CRL &= ~GPIO_CRL_CNF2;
        GPIOB->CRH &= ~(GPIO_CRH_CNF12 | GPIO_CRH_CNF11 | GPIO_CRH_CNF10);
        GPIOC->CRL &= ~(GPIO_CRL_CNF7 | GPIO_CRL_CNF6 | GPIO_CRL_CNF5 | GPIO_CRL_CNF4 | GPIO_CRL_CNF3 | GPIO_CRL_CNF2 | GPIO_CRL_CNF1 | GPIO_CRL_CNF0);
        GPIOC->CRH &= ~(GPIO_CRH_CNF15 | GPIO_CRH_CNF14 | GPIO_CRH_CNF12 | GPIO_CRH_CNF11 | GPIO_CRH_CNF10 | GPIO_CRH_CNF9 | GPIO_CRH_CNF8);
        GPIOD->CRL &= ~(GPIO_CRL_CNF7 | GPIO_CRL_CNF6 | GPIO_CRL_CNF5 | GPIO_CRL_CNF4 | GPIO_CRL_CNF3 | GPIO_CRL_CNF2 | GPIO_CRL_CNF1 | GPIO_CRL_CNF0);
        GPIOD->CRH &= ~(GPIO_CRH_CNF15 | GPIO_CRH_CNF14 | GPIO_CRH_CNF13 | GPIO_CRH_CNF12 | GPIO_CRH_CNF11 | GPIO_CRH_CNF10 | GPIO_CRH_CNF9 | GPIO_CRH_CNF8);
        GPIOE->CRL &= ~(GPIO_CRL_CNF7 | GPIO_CRL_CNF6 | GPIO_CRL_CNF5 | GPIO_CRL_CNF4 | GPIO_CRL_CNF3 | GPIO_CRL_CNF2 | GPIO_CRL_CNF1 | GPIO_CRL_CNF0);
        GPIOE->CRH &= ~(GPIO_CRH_CNF15 | GPIO_CRH_CNF14 | GPIO_CRH_CNF13 | GPIO_CRH_CNF12 | GPIO_CRH_CNF11 | GPIO_CRH_CNF10 | GPIO_CRH_CNF9 | GPIO_CRH_CNF8);
    }

    // TIM2
    {
        RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

        TIM2->CR1 |= TIM_CR1_ARPE;
        TIM2->PSC = 1152 - 1;
        TIM2->ARR = 62500 - 1;
        TIM2->EGR |= TIM_EGR_UG;

        TIM2->CR1 |= TIM_CR1_CEN;
    }

    // TIM3
    {
        RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

        TIM3->CR1 |= TIM_CR1_ARPE;
        TIM3->PSC = 24 - 1;
        TIM3->ARR = 60000 - 1;
        TIM3->EGR |= TIM_EGR_UG;

        TIM3->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE | TIM_CCMR1_OC2FE | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE | TIM_CCMR1_OC1FE;
        TIM3->CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4PE | TIM_CCMR2_OC4FE | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3PE | TIM_CCMR2_OC3FE;
        TIM3->CCER |= TIM_CCER_CC4E | TIM_CCER_CC3E | TIM_CCER_CC2E | TIM_CCER_CC1E;

        TIM3->CR1 |= TIM_CR1_CEN;
    }

    // TIM4
    {
        RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

        TIM4->CR1 |= TIM_CR1_ARPE;
        TIM4->PSC = 72 - 1;
        TIM4->ARR = 10000 - 1;
        TIM4->EGR |= TIM_EGR_UG;

        TIM4->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE | TIM_CCMR1_OC2FE | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE | TIM_CCMR1_OC1FE;
        TIM4->CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4PE | TIM_CCMR2_OC4FE | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3PE | TIM_CCMR2_OC3FE;
        TIM4->CCER |= TIM_CCER_CC4E | TIM_CCER_CC3E | TIM_CCER_CC2E | TIM_CCER_CC1E;

        TIM4->CR1 |= TIM_CR1_CEN;
    }

    // USART1
    {
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

        USART1->CR1 |= USART_CR1_RE;
        USART1->BRR = 2 << USART_BRR_DIV_Mantissa_Pos | 0b100;

        USART1->CR3 |= USART_CR3_DMAR;

        USART1->CR1 |= USART_CR1_UE;
    }

    // USB
    {
        RCC->APB1ENR |= RCC_APB1ENR_USBEN;

        USB->CNTR &= ~USB_CNTR_PDWN;

        // wait for defined USB transceiver startup time
        SysTick->LOAD = SystemCoreClock / 1e6;
        SysTick->VAL = 0;
        SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
        while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk))
            ;
        SysTick->CTRL = 0;

        USB->CNTR &= ~USB_CNTR_FRES;

        USB->DADDR |= USB_DADDR_EF;
    }

    SysTick_Config(SystemCoreClock / 1000); // 1ms SysTick interrupt

    while (1) {

        // state
        if (task_cnt_ms_State >= TASK_PERIOD_ms_State) {
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
        } else {
            task_freq_Err = state_W.obj_detect ? TASK_FREQ_Err_OBJ_DETECT
                                               : TASK_FREQ_Err_OBJ_UNDETECT;

            if (task_cnt_ms_Err >= 1000 / task_freq_Err / 2) {
                task_cnt_ms_Err = 0;

                Err(0);
            }
        }
    }
}
