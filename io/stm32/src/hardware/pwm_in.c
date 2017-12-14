#include "../include/pwm_in.h"

u16 pwm_in[8] =
{ 0, 0, 0, 0, 0, 0 };
u16 Rise[8], Drop[8];

void pwm_in_init(void)
{
	TIM2_Cap_Init();
	TIM3_Cap_Init();
}

void TIM2_Cap_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;            //TIM2ÖÐ¶Ï
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //ÏÈÕŒÓÅÏÈŒ¶1Œ¶
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;         //ŽÓÓÅÏÈŒ¶0Œ¶
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            //IRQÍšµÀ±»Ê¹ÄÜ
	NVIC_Init(&NVIC_InitStructure);

	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM2_ICInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);	 //使能TIM2时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //使能GPIOB时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);  //使能AFIO功能的时钟
//	GPIO_PinRemapConfig(GPIO_Remap_TIM2, ENABLE);  //进行重映射

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_ResetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);

	//初始化定时器4 TIM2
	TIM_TimeBaseStructure.TIM_Period = 0XFFFF;                   //设定计数器自动重装值
	TIM_TimeBaseStructure.TIM_Prescaler = 71; 	                   //预分频器
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;      //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);              //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

	//初始化TIM2输入捕获参数
	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_1;                //CC1S=01 	选择输入端 IC1映射到TI1上
	TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	   //上升沿捕获
	TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
	TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	         //配置输入分频,不分频
	TIM2_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 配置输入滤波器 不滤波
	TIM_ICInit(TIM2, &TIM2_ICInitStructure);

	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_2;                //CC1S=01 	选择输入端 IC1映射到TI1上
	TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	   //上升沿捕获
	TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
	TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	         //配置输入分频,不分频
	TIM2_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 配置输入滤波器 不滤波
	TIM_ICInit(TIM2, &TIM2_ICInitStructure);

	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_3;                //CC1S=01 	选择输入端 IC1映射到TI1上
	TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	   //上升沿捕获
	TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
	TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	         //配置输入分频,不分频
	TIM2_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 配置输入滤波器 不滤波
	TIM_ICInit(TIM2, &TIM2_ICInitStructure);

	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_4;                //CC1S=01 	选择输入端 IC1映射到TI1上
	TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	   //上升沿捕获
	TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
	TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	         //配置输入分频,不分频
	TIM2_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 配置输入滤波器 不滤波
	TIM_ICInit(TIM2, &TIM2_ICInitStructure);

	TIM_Cmd(TIM2, ENABLE);

	TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);        //允许更新中断 ,允许CC1IE捕获中断
	TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);
	TIM_ITConfig(TIM2, TIM_IT_CC3, ENABLE);
	TIM_ITConfig(TIM2, TIM_IT_CC4, ENABLE);

}

void TIM3_Cap_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;            //TIM3ÖÐ¶Ï
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //ÏÈÕŒÓÅÏÈŒ¶1Œ¶
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;         //ŽÓÓÅÏÈŒ¶0Œ¶
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            //IRQÍšµÀ±»Ê¹ÄÜ
	NVIC_Init(&NVIC_InitStructure);

	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM3_ICInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	 //使能TIM3时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //使能GPIOB时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  //使能GPIOB时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);  //使能AFIO功能的时钟
//	GPIO_PinRemapConfig(GPIO_Remap_TIM3, ENABLE);  //进行重映射

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_ResetBits(GPIOA, GPIO_Pin_6 | GPIO_Pin_7);
	GPIO_ResetBits(GPIOB, GPIO_Pin_0 | GPIO_Pin_1);

	//初始化定时器4 TIM3
	TIM_TimeBaseStructure.TIM_Period = 0XFFFF;                   //设定计数器自动重装值
	TIM_TimeBaseStructure.TIM_Prescaler = 71; 	                   //预分频器
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;      //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);              //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

	//初始化TIM3输入捕获参数
	TIM3_ICInitStructure.TIM_Channel = TIM_Channel_1;                //CC1S=01 	选择输入端 IC1映射到TI1上
	TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	   //上升沿捕获
	TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
	TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	         //配置输入分频,不分频
	TIM3_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 配置输入滤波器 不滤波
	TIM_ICInit(TIM3, &TIM3_ICInitStructure);

	TIM3_ICInitStructure.TIM_Channel = TIM_Channel_2;                //CC1S=01 	选择输入端 IC1映射到TI1上
	TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	   //上升沿捕获
	TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
	TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	         //配置输入分频,不分频
	TIM3_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 配置输入滤波器 不滤波
	TIM_ICInit(TIM3, &TIM3_ICInitStructure);

	TIM3_ICInitStructure.TIM_Channel = TIM_Channel_3;                //CC1S=01 	选择输入端 IC1映射到TI1上
	TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	   //上升沿捕获
	TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
	TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	         //配置输入分频,不分频
	TIM3_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 配置输入滤波器 不滤波
	TIM_ICInit(TIM3, &TIM3_ICInitStructure);

	TIM3_ICInitStructure.TIM_Channel = TIM_Channel_4;                //CC1S=01 	选择输入端 IC1映射到TI1上
	TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	   //上升沿捕获
	TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
	TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	         //配置输入分频,不分频
	TIM3_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 配置输入滤波器 不滤波
	TIM_ICInit(TIM3, &TIM3_ICInitStructure);

	TIM_Cmd(TIM3, ENABLE);

	TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);        //允许更新中断 ,允许CC1IE捕获中断
	TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);
	TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE);
	TIM_ITConfig(TIM3, TIM_IT_CC4, ENABLE);

}

void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)   //捕获1发生捕获事件
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC1); //清除中断标志位
		if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == 1)
		{
			TIM_OC1PolarityConfig(TIM2, TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
			Rise[0] = TIM_GetCapture1(TIM2);
		}
		else
		{
			TIM_OC1PolarityConfig(TIM2, TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
			Drop[0] = TIM_GetCapture1(TIM2);
			if (Rise[0] > Drop[0])
				pwm_in[0] = 65535 - Rise[0] + Drop[0];
			else
				pwm_in[0] = Drop[0] - Rise[0];
		}
	}

	if (TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET)            //捕获1发生捕获事件
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC2); //清除中断标志位
		if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == 1)
		{
			TIM_OC2PolarityConfig(TIM2, TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
			Rise[1] = TIM_GetCapture2(TIM2);
		}
		else
		{
			TIM_OC2PolarityConfig(TIM2, TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
			Drop[1] = TIM_GetCapture2(TIM2);
			if (Rise[1] > Drop[1])
				pwm_in[1] = 65535 - Rise[1] + Drop[1];
			else
				pwm_in[1] = Drop[1] - Rise[1];
		}
	}

	if (TIM_GetITStatus(TIM2, TIM_IT_CC3) != RESET)            //捕获1发生捕获事件
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC3); //清除中断标志位
		if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2) == 1)
		{
			TIM_OC3PolarityConfig(TIM2, TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
			Rise[2] = TIM_GetCapture3(TIM2);
		}
		else
		{
			TIM_OC3PolarityConfig(TIM2, TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
			Drop[2] = TIM_GetCapture3(TIM2);
			if (Rise[2] > Drop[2])
				pwm_in[2] = 65535 - Rise[2] + Drop[2];
			else
				pwm_in[2] = Drop[2] - Rise[2];
		}
	}

	if (TIM_GetITStatus(TIM2, TIM_IT_CC4) != RESET)            //捕获1发生捕获事件
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC4); //清除中断标志位
		if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3) == 1)
		{
			TIM_OC4PolarityConfig(TIM2, TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
			Rise[3] = TIM_GetCapture4(TIM2);
		}
		else
		{
			TIM_OC4PolarityConfig(TIM2, TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
			Drop[3] = TIM_GetCapture4(TIM2);
			if (Rise[3] > Drop[3])
				pwm_in[3] = 65535 - Rise[3] + Drop[3];
			else
				pwm_in[3] = Drop[3] - Rise[3];
		}
	}
}

void TIM3_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)   //捕获1发生捕获事件
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC1); //清除中断标志位
		if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6) == 1)
		{
			TIM_OC1PolarityConfig(TIM3, TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
			Rise[4] = TIM_GetCapture1(TIM3);
		}
		else
		{
			TIM_OC1PolarityConfig(TIM3, TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
			Drop[4] = TIM_GetCapture1(TIM3);
			if (Rise[4] > Drop[4])
				pwm_in[4] = 65535 - Rise[4] + Drop[4];
			else
				pwm_in[4] = Drop[4] - Rise[4];
		}
	}

	if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)            //捕获1发生捕获事件
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC2); //清除中断标志位
		if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7) == 1)
		{
			TIM_OC2PolarityConfig(TIM3, TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
			Rise[5] = TIM_GetCapture2(TIM3);
		}
		else
		{
			TIM_OC2PolarityConfig(TIM3, TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
			Drop[5] = TIM_GetCapture2(TIM3);
			if (Rise[5] > Drop[5])
				pwm_in[5] = 65535 - Rise[5] + Drop[5];
			else
				pwm_in[5] = Drop[5] - Rise[5];
		}
	}

	if (TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET)            //捕获1发生捕获事件
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC3); //清除中断标志位
		if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) == 1)
		{
			TIM_OC3PolarityConfig(TIM3, TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
			Rise[6] = TIM_GetCapture3(TIM3);
		}
		else
		{
			TIM_OC3PolarityConfig(TIM3, TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
			Drop[6] = TIM_GetCapture3(TIM3);
			if (Rise[6] > Drop[6])
				pwm_in[6] = 65535 - Rise[6] + Drop[6];
			else
				pwm_in[6] = Drop[6] - Rise[6];
		}
	}

	if (TIM_GetITStatus(TIM3, TIM_IT_CC4) != RESET)            //捕获1发生捕获事件
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC4); //清除中断标志位
		if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 1)
		{
			TIM_OC4PolarityConfig(TIM3, TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
			Rise[7] = TIM_GetCapture4(TIM3);
		}
		else
		{
			TIM_OC4PolarityConfig(TIM3, TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
			Drop[7] = TIM_GetCapture4(TIM3);
			if (Rise[7] > Drop[7])
				pwm_in[7] = 65535 - Rise[7] + Drop[7];
			else
				pwm_in[7] = Drop[7] - Rise[7];
		}
	}
}
