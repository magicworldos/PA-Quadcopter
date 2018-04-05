#include "../include/pwm_in.h"

u16 pwm_in[8] =
{ 0, 0, 0, 0, 0, 0, 0, 0 };
u16 pwm_rise[8] =
{ 0, 0, 0, 0, 0, 0, 0, 0 };
u16 pwm_drop[8] =
{ 0, 0, 0, 0, 0, 0, 0, 0 };

__IO u32 pwm_in_error_count = 0;

void pwm_in_init(void)
{
	tim5_irq_init();

	tim4_irq_init();

	tim5_gpio_init();

	tim4_gpio_init();

	tim5_capture_init();

	tim4_capture_init();
}

void tim5_irq_init()
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void tim4_irq_init()
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void tim5_gpio_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);
}

void tim4_gpio_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_ResetBits(GPIOB, GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9);
}

void tim5_capture_init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM5_ICInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	//初始化定时器2 TIM5 设定计数器自动重装值
	TIM_TimeBaseStructure.TIM_Period = 0xffff;
	//预分频器
	TIM_TimeBaseStructure.TIM_Prescaler = 35;
	//设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_ICPSC_DIV1;
	//TIM向上计数模式
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	//根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

	//初始化TIM5输入捕获参数
	//CC1S=01 	选择输入端 IC1映射到TI1上
	TIM5_ICInitStructure.TIM_Channel = TIM_Channel_1;
	//上升沿捕获
	TIM5_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	//映射到TI1上
	TIM5_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	//配置输入分频,不分频
	TIM5_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	//IC1F=0000 配置输入滤波器 不滤波
	TIM5_ICInitStructure.TIM_ICFilter = 0x00;
	TIM_ICInit(TIM5, &TIM5_ICInitStructure);

	TIM5_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM5_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM5_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM5_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM5_ICInitStructure.TIM_ICFilter = 0x00;
	TIM_ICInit(TIM5, &TIM5_ICInitStructure);

	TIM5_ICInitStructure.TIM_Channel = TIM_Channel_3;
	TIM5_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM5_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM5_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM5_ICInitStructure.TIM_ICFilter = 0x00;
	TIM_ICInit(TIM5, &TIM5_ICInitStructure);

	TIM5_ICInitStructure.TIM_Channel = TIM_Channel_4;
	TIM5_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM5_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM5_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM5_ICInitStructure.TIM_ICFilter = 0x00;
	TIM_ICInit(TIM5, &TIM5_ICInitStructure);

	TIM_Cmd(TIM5, ENABLE);

	//允许更新中断 ,允许CC1IE捕获中断
	TIM_ITConfig(TIM5, TIM_IT_CC1, ENABLE);
	TIM_ITConfig(TIM5, TIM_IT_CC2, ENABLE);
	TIM_ITConfig(TIM5, TIM_IT_CC3, ENABLE);
	TIM_ITConfig(TIM5, TIM_IT_CC4, ENABLE);

}

void tim4_capture_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM4_ICInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	//初始化定时器4 TIM4
	TIM_TimeBaseStructure.TIM_Period = 0xffff;
	TIM_TimeBaseStructure.TIM_Prescaler = 35;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_ICPSC_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	//初始化TIM4输入捕获参数
	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM4_ICInitStructure.TIM_ICFilter = 0x00;
	TIM_ICInit(TIM4, &TIM4_ICInitStructure);

	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM4_ICInitStructure.TIM_ICFilter = 0x00;
	TIM_ICInit(TIM4, &TIM4_ICInitStructure);

	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_3;
	TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM4_ICInitStructure.TIM_ICFilter = 0x00;
	TIM_ICInit(TIM4, &TIM4_ICInitStructure);

	TIM4_ICInitStructure.TIM_Channel = TIM_Channel_4;
	TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM4_ICInitStructure.TIM_ICFilter = 0x00;
	TIM_ICInit(TIM4, &TIM4_ICInitStructure);

	TIM_Cmd(TIM4, ENABLE);

	TIM_ITConfig(TIM4, TIM_IT_CC1, ENABLE);
	TIM_ITConfig(TIM4, TIM_IT_CC2, ENABLE);
	TIM_ITConfig(TIM4, TIM_IT_CC3, ENABLE);
	TIM_ITConfig(TIM4, TIM_IT_CC4, ENABLE);

}

void TIM5_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM5, TIM_IT_CC1) != RESET)
	{
		TIM_ClearITPendingBit(TIM5, TIM_IT_CC1);
		if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == 1)
		{
			//CC1P=2 设置为下降沿捕获
			TIM_OC1PolarityConfig(TIM5, TIM_ICPolarity_Falling);
			pwm_rise[0] = TIM_GetCapture1(TIM5);
		}
		else
		{
			//CC1P=0 设置为上升沿捕获
			TIM_OC1PolarityConfig(TIM5, TIM_ICPolarity_Rising);
			pwm_drop[0] = TIM_GetCapture1(TIM5);
			if (pwm_rise[0] > pwm_drop[0])
			{
				pwm_in[0] = 65535 - pwm_rise[0] + pwm_drop[0];
			}
			else
			{
				pwm_in[0] = pwm_drop[0] - pwm_rise[0];
			}
		}
	}

	if (TIM_GetITStatus(TIM5, TIM_IT_CC2) != RESET)
	{
		TIM_ClearITPendingBit(TIM5, TIM_IT_CC2);
		if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == 1)
		{
			TIM_OC2PolarityConfig(TIM5, TIM_ICPolarity_Falling);
			pwm_rise[1] = TIM_GetCapture2(TIM5);
		}
		else
		{
			TIM_OC2PolarityConfig(TIM5, TIM_ICPolarity_Rising);
			pwm_drop[1] = TIM_GetCapture2(TIM5);
			if (pwm_rise[1] > pwm_drop[1])
			{
				pwm_in[1] = 65535 - pwm_rise[1] + pwm_drop[1];
			}
			else
			{
				pwm_in[1] = pwm_drop[1] - pwm_rise[1];
			}
		}
	}

	if (TIM_GetITStatus(TIM5, TIM_IT_CC3) != RESET)
	{
		TIM_ClearITPendingBit(TIM5, TIM_IT_CC3);
		if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2) == 1)
		{
			TIM_OC3PolarityConfig(TIM5, TIM_ICPolarity_Falling);
			pwm_rise[2] = TIM_GetCapture3(TIM5);
		}
		else
		{
			TIM_OC3PolarityConfig(TIM5, TIM_ICPolarity_Rising);
			pwm_drop[2] = TIM_GetCapture3(TIM5);
			if (pwm_rise[2] > pwm_drop[2])
			{
				pwm_in[2] = 65535 - pwm_rise[2] + pwm_drop[2];
			}
			else
			{
				pwm_in[2] = pwm_drop[2] - pwm_rise[2];
			}
		}
	}

	if (TIM_GetITStatus(TIM5, TIM_IT_CC4) != RESET)
	{
		TIM_ClearITPendingBit(TIM5, TIM_IT_CC4);
		if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3) == 1)
		{
			TIM_OC4PolarityConfig(TIM5, TIM_ICPolarity_Falling);
			pwm_rise[3] = TIM_GetCapture4(TIM5);
		}
		else
		{
			TIM_OC4PolarityConfig(TIM5, TIM_ICPolarity_Rising);
			pwm_drop[3] = TIM_GetCapture4(TIM5);
			if (pwm_rise[3] > pwm_drop[3])
			{
				pwm_in[3] = 65535 - pwm_rise[3] + pwm_drop[3];
			}
			else
			{
				pwm_in[3] = pwm_drop[3] - pwm_rise[3];
			}
			pwm_in_error_count = 0;
		}
	}
}

void TIM4_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET)   //捕获1发生捕获事件
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);
		if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6) == 1)
		{
			TIM_OC1PolarityConfig(TIM4, TIM_ICPolarity_Falling);
			pwm_rise[4] = TIM_GetCapture1(TIM4);
		}
		else
		{
			TIM_OC1PolarityConfig(TIM4, TIM_ICPolarity_Rising);
			pwm_drop[4] = TIM_GetCapture1(TIM4);
			if (pwm_rise[4] > pwm_drop[4])
			{
				pwm_in[4] = 65535 - pwm_rise[4] + pwm_drop[4];
			}
			else
			{
				pwm_in[4] = pwm_drop[4] - pwm_rise[4];
			}
		}
	}

	if (TIM_GetITStatus(TIM4, TIM_IT_CC2) != RESET)
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);
		if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7) == 1)
		{
			TIM_OC2PolarityConfig(TIM4, TIM_ICPolarity_Falling);
			pwm_rise[5] = TIM_GetCapture2(TIM4);
		}
		else
		{
			TIM_OC2PolarityConfig(TIM4, TIM_ICPolarity_Rising);
			pwm_drop[5] = TIM_GetCapture2(TIM4);
			if (pwm_rise[5] > pwm_drop[5])
			{
				pwm_in[5] = 65535 - pwm_rise[5] + pwm_drop[5];
			}
			else
			{
				pwm_in[5] = pwm_drop[5] - pwm_rise[5];
			}
		}
	}

	if (TIM_GetITStatus(TIM4, TIM_IT_CC3) != RESET)
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC3);
		if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8) == 1)
		{
			TIM_OC3PolarityConfig(TIM4, TIM_ICPolarity_Falling);
			pwm_rise[6] = TIM_GetCapture3(TIM4);
		}
		else
		{
			TIM_OC3PolarityConfig(TIM4, TIM_ICPolarity_Rising);
			pwm_drop[6] = TIM_GetCapture3(TIM4);
			if (pwm_rise[6] > pwm_drop[6])
			{
				pwm_in[6] = 65535 - pwm_rise[6] + pwm_drop[6];
			}
			else
			{
				pwm_in[6] = pwm_drop[6] - pwm_rise[6];
			}
		}
	}

	if (TIM_GetITStatus(TIM4, TIM_IT_CC4) != RESET)
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC4);
		if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9) == 1)
		{
			TIM_OC4PolarityConfig(TIM4, TIM_ICPolarity_Falling);
			pwm_rise[7] = TIM_GetCapture4(TIM4);
		}
		else
		{
			TIM_OC4PolarityConfig(TIM4, TIM_ICPolarity_Rising);
			pwm_drop[7] = TIM_GetCapture4(TIM4);
			if (pwm_rise[7] > pwm_drop[7])
			{
				pwm_in[7] = 65535 - pwm_rise[7] + pwm_drop[7];
			}
			else
			{
				pwm_in[7] = pwm_drop[7] - pwm_rise[7];
			}
		}
	}
}
