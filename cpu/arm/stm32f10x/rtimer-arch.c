#include "sys/energest.h"
#include "sys/rtimer.h"
#include "stm32f10x.h"
#include "stm32f10x_it.h"

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

void rtimer_arch_comp_isr(void) 
{
	TIM_ITConfig(TIM2, TIM_IT_CC1, DISABLE); // Disable the next compare interrupt

	PRINTF("\r\nCompare event %4x\r\n", (unsigned int)TIM2->CNT);
	ENERGEST_ON(ENERGEST_TYPE_IRQ);
	rtimer_run_next();
	ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}

void rtimer_arch_init(void) 
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	uint16_t PrescalerValue = 0;

	// Compute the prescaler value
	// TIM2 clock enable
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	PrescalerValue = (uint16_t) ((SystemCoreClock) / RTIMER_ARCH_SECOND) - 1;

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	// Prescaler configuration
	TIM_PrescalerConfig(TIM2, PrescalerValue, TIM_PSCReloadMode_Immediate);

	// Output Compare Timing Mode configuration: Channel1
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0xFFFF;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Disable);

	// Interrupt generation
    TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);

	// TIM2 enable counter
	TIM_Cmd(TIM2, ENABLE);

	// NVIC
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	PRINTF("rtimer_arch_init done\r\n");
}

rtimer_clock_t rtimer_arch_now(void) 
{
	return (rtimer_clock_t)TIM2->CNT;
}

void rtimer_arch_schedule(rtimer_clock_t t) 
{
    TIM_SetCompare1(TIM2, (uint16_t)t);
}

void TIM2_IRQHandler(void) 
{
//	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) 
//    {   // Overflow event. 
//		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
//		//rtimer_arch_ovf_isr();
//	} 
    if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET) 
    {
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
        
        ENERGEST_ON(ENERGEST_TYPE_IRQ);
        rtimer_run_next();
        ENERGEST_OFF(ENERGEST_TYPE_IRQ);
	}
}
