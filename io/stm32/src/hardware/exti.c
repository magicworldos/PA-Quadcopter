#include <exti.h>

void EXTIX_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD | GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD | GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource6);

	EXTI_InitStructure.EXTI_Line = EXTI_Line6;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource7);

	EXTI_InitStructure.EXTI_Line = EXTI_Line7;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);

	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);

	EXTI_InitStructure.EXTI_Line = EXTI_Line1;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

extern u32 timer_tick;

u32 _start1 = 0;
u32 _start2 = 0;
u32 _start3 = 0;
u32 _start4 = 0;

u32 pwm1 = 0;
u32 pwm2 = 0;
u32 pwm3 = 0;
u32 pwm4 = 0;

void EXTI9_5_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line6) != RESET)
	{
		if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6) == Bit_SET)
		{
			_start1 = timer_tick;
		}
		else if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6) == Bit_RESET)
		{
			u32 _end = timer_tick;
			if (_end < _start1)
			{
				pwm1 = 0xffffffff - _start1 + _end;
			}
			else
			{
				pwm1 = _end - _start1;
			}
		}
	}
	EXTI_ClearITPendingBit(EXTI_Line6);

	if (EXTI_GetITStatus(EXTI_Line7) != RESET)
	{
		if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7) == Bit_SET)
		{
			_start2 = timer_tick;
		}
		else if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7) == Bit_RESET)
		{
			u32 _end = timer_tick;
			if (_end < _start2)
			{
				pwm2 = 0xffffffff - _start2 + _end;
			}
			else
			{
				pwm2 = _end - _start2;
			}
		}

	}
	EXTI_ClearITPendingBit(EXTI_Line7);
}

void EXTI0_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
		if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) == Bit_SET)
		{
			_start3 = timer_tick;
		}
		else if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) == Bit_RESET)
		{
			u32 _end = timer_tick;
			if (_end < _start3)
			{
				pwm3 = 0xffffffff - _start3 + _end;
			}
			else
			{
				pwm3 = _end - _start3;
			}
		}
	}
	EXTI_ClearITPendingBit(EXTI_Line0);
}

void EXTI1_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line1) != RESET)
	{
		if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == Bit_SET)
		{
			_start4 = timer_tick;
		}
		else if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == Bit_RESET)
		{
			u32 _end = timer_tick;
			if (_end < _start4)
			{
				pwm4 = 0xffffffff - _start4 + _end;
			}
			else
			{
				pwm4 = _end - _start4;
			}
		}
	}
	EXTI_ClearITPendingBit(EXTI_Line1);
}
