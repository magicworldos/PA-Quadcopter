#include <pwm.h>

void pwm_init()
{
	pwm_gpio_config_out();
//	pwm_gpio_config_in();
	pwm_mode_config_out();
//	pwm_mode_config_in();
}

void pwm_gpio_config_out()
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

//void pwm_gpio_config_in()
//{
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
//
//	GPIO_InitTypeDef GPIO_InitStructure;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//
//	//input
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
//
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
//
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
//}

void pwm_mode_config_out()
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	u16 CCR_Val = 1000;

	TIM_TimeBaseStructure.TIM_Period = 2000;
	TIM_TimeBaseStructure.TIM_Prescaler = 72;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = CCR_Val;
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

//void pwm_mode_config_in()
//{
//	TIM_ICInitTypeDef TIM_ICInitStructure;
//	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
//
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
//
//	TIM_TimeBaseStructure.TIM_Period = 2000;
//	TIM_TimeBaseStructure.TIM_Prescaler = 72;
//	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
//
//	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
//	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
//	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
//	TIM_ICInitStructure.TIM_ICFilter = 0;
//
//	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
//	TIM_PWMIConfig(TIM3, &TIM_ICInitStructure);
//
//	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
//	TIM_PWMIConfig(TIM3, &TIM_ICInitStructure);
//
//	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
//	TIM_PWMIConfig(TIM3, &TIM_ICInitStructure);
//
//	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
//	TIM_PWMIConfig(TIM3, &TIM_ICInitStructure);
//
//	TIM_SelectInputTrigger(TIM3, TIM_TS_TI2FP2);
//	TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);
//	TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Enable);
//
//	TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);
//	TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);
//	TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE);
//	TIM_ITConfig(TIM3, TIM_IT_CC4, ENABLE);
//
//	TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
//	TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
//	TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
//	TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);
//
//	TIM_Cmd(TIM3, ENABLE);
//
//	NVIC_InitTypeDef NVIC_InitStructure;
//	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//}

//__IO u32 pwm1 = 0;
//__IO u32 pwm2 = 0;
//__IO u32 pwm3 = 0;
//__IO u32 pwm4 = 0;
//
//void TIM3_IRQHandler(void)
//{
//	if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
//	{
//		pwm1 = TIM_GetCapture1(TIM3);
//		//IC2Value = TIM_GetCapture2(TIM3);
//		//Frequency = 72000000 / IC2Value;
//	}
//	if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)
//	{
//		pwm2 = TIM_GetCapture2(TIM3);
//	}
//	if (TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET)
//	{
//		pwm3 = TIM_GetCapture3(TIM3);
//	}
//	if (TIM_GetITStatus(TIM3, TIM_IT_CC4) != RESET)
//	{
//		pwm4 = TIM_GetCapture4(TIM3);
//	}
//
//	TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
//	TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
//	TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
//	TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);
//}
