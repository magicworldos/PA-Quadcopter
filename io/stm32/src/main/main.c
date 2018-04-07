#include <typedef.h>
#include <pwm_in.h>
#include <pwm_out.h>
#include <led_light.h>
#include <timer_tick.h>
#include <serial_port.h>

extern u16 pwm_in[8];
extern u16 pwm_out[8];
extern u32 pwm_in_error_count;
extern u32 pwm_out_error_count;

extern u16 status;

void RCC_config() //如果外部晶振为8M，PLLCLK=SYSCLK=72M，HCLK=72M，//P2CLK=72M，P1CLK=36M，ADCCLK=36M，USBCLK=48M，TIMCLK=72M
{
	ErrorStatus HSEStartUpStatus; // 定义错误状态变量
	RCC_DeInit(); //将RCC寄存器重新设置为默认值
	RCC_HSEConfig(RCC_HSE_ON); //打开外部高速时钟晶振
	HSEStartUpStatus = RCC_WaitForHSEStartUp(); // 等待外部高速时钟晶振工作
	if (HSEStartUpStatus == SUCCESS)
	{
		RCC_HCLKConfig(RCC_SYSCLK_Div1); //设置AHB不分频，HCLK=SYSCLK
		RCC_PCLK2Config(RCC_HCLK_Div1); //设置APB2不分频，P2CLK=HCLK
		RCC_PCLK1Config(RCC_HCLK_Div2); //设置APB1 为2分频，P1CLK=HCLK/2
//		FLASH_SetLatency(FLASH_Latency_2); //设置FLASH代码延时
//		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable); //使能预取指缓存
		RCC_PLLConfig(RCC_CFGR_PLLSRC_HSI_Div2, RCC_CFGR_PLLMULL9); //设置PLL时钟源，
		//外部时钟不分频，为HSE的9倍频8MHz * 9 = 72MHz
		RCC_PLLCmd(ENABLE); //使能PLL
		while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)	//等待PLL准备就绪
		{
		}
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); //设置PLL为系统时钟源
		while (RCC_GetSYSCLKSource() != 0x08) //判断PLL是否是系统时钟
		{
		}
	}

	/*RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOD, ENABLE); // 打开 PB 和 PD用于点亮 LED 灯*/
}

int main(int argc, char* argv[])
{
	SystemInit();

	led_init();

	led0_off();
	led1_off();

	RCC_config();

	pwm_in_init();

	pwm_out_init();

	serial_port_init();

	timer_init();

	timer_start();

	while (1)
	{
		serial_port_frame_recv_pwm(pwm_out);
		if (pwm_out_error_count < 2 * PWM_ERR_MAX)
		{
			pwm_out_error_count++;
		}
		if (pwm_out_error_count > PWM_ERR_MAX)
		{
			pwm_out_set_failsafe();
		}

		if (pwm_in_error_count < 2 * PWM_ERR_MAX)
		{
			pwm_in_error_count++;
		}
		if (pwm_in_error_count > PWM_ERR_MAX)
		{
			memset(pwm_in, 0, sizeof(u16) * 8);
			pwm_out_set_failsafe();
			serial_port_frame_send_rc(pwm_in);
		}
		serial_port_frame_send_rc(pwm_in);

		pwm_out_set_value();

		//RC and PWMOUT OK
		if (pwm_in_error_count < PWM_ERR_MAX && pwm_out_error_count < PWM_ERR_MAX)
		{
			led0_blink(100 * 1000);
		}
		//RC or PWMOUT error
		else
		{
			status = 0xffff;
			led0_blink(1000 * 1000);
		}

		//lock
		if (status != 0)
		{
			led1_blink(1000 * 1000);
		}

		//everything is going OK.
		if (status == 0)
		{
			led1_blink(100 * 1000);
		}
		timer_delay_ms(5);
	}
}

