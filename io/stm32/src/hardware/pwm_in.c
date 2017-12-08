#include <pwm_in.h>

void TIM3_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void TIM3_Mode_Config(void)
{
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //使能TIM3时钟

	TIM_TimeBaseStructure.TIM_Period = 2000; //设置在下一个更新事件装入活动的自动重装载寄存
	TIM_TimeBaseStructure.TIM_Prescaler = 72; //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;               //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;               //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);               //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;                //通道2
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;     //上升沿触发
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //管脚与寄存器对应关系
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; 		//预分频
	TIM_ICInitStructure.TIM_ICFilter = 0;	//滤波设置, 经理几个周期跳变认定波形稳定

	TIM_PWMIConfig(TIM3, &TIM_ICInitStructure);                     //PWM输入配置
	TIM_SelectInputTrigger(TIM3, TIM_TS_TI2FP2);                     //选择有效输入端，选择IC2为始终触发源
	TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);                     //TIM从模式：触发信号的上升沿重新初始化计数器和触发寄存器的更新事件
	TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Enable);                     //启动定时器的被动触发

	TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE); //中断配置
	TIM_ClearITPendingBit(TIM3, TIM_IT_CC3); //清除中断标志位
	TIM_Cmd(TIM3, ENABLE);   		//打开TIM3

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;   		//TIM2 PWM输入捕获的中断的优先级为1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

__IO float DutyCycle = 0;
__IO u32 IC1Value = 0;
__IO u32 IC2Value = 0;
__IO u32 Frequency = 0;

void TIM3_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET)        //捕获1发生捕获事件
	{
		IC1Value = TIM_GetCapture1(TIM3);                      //采集占空比
		IC2Value = TIM_GetCapture2(TIM3);                    //采集周期
		Frequency = 72000000 / IC2Value;
	}
	TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);               //清除中断标志位
}

void TIM3_PWM_Init(void)
{
	TIM3_GPIO_Config();
	TIM3_Mode_Config();
}

float get_pwm_in()
{
	return DutyCycle;
}

u32 get_pwm_in1()
{
	return IC1Value;
}

u32 get_pwm_in2()
{
	return IC2Value;
}
