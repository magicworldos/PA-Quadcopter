/*
 * pwm_out.c
 *
 *  Created on: Jun 25, 2017
 *      Author: lidq
 */

#include <pwm_out.h>

u16 pwm[4] = { 0, 0, 0, 0 };

void pwm_out_init()
{
	pwm_out_gpio_config();

	pwm_out_mode_config();

	pwm_out_set_value();
}

void pwm_out_gpio_config()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	//output
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
}

void pwm_out_mode_config()
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	TIM_TimeBaseStructure.TIM_Period = TIM_PERIOD;
	TIM_TimeBaseStructure.TIM_Prescaler = TIM_PRESCALER;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = TIM_PULSE;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	//output Channel1
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	//output Channel2
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
	//output Channel3
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
	//output Channel4
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
	//enable TIM4
	TIM_ARRPreloadConfig(TIM4, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
}

void pwm_out_set_value()
{
	TIM_SetCompare1(TIM4, pwm[0]);
	TIM_SetCompare2(TIM4, pwm[1]);
	TIM_SetCompare3(TIM4, pwm[2]);
	TIM_SetCompare4(TIM4, pwm[3]);
}
