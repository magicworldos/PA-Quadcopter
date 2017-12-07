#include <typedef.h>
#include <led.h>
#include <timer.h>
#include <pwm.h>
#include <uart.h>
#include <exti.h>

extern u32 pwm1;
extern u32 pwm2;
extern u32 pwm3;
extern u32 pwm4;

int main(int argc, char* argv[])
{
	SystemInit();

	led_init();

	EXTIX_Init();

	UART_Init();

	pwm_init();

	timer_init();

	timer_start();

	TIM_SetCompare1(TIM4, 600);
	TIM_SetCompare2(TIM4, 800);
	TIM_SetCompare3(TIM4, 1000);
	TIM_SetCompare4(TIM4, 1200);
	u32 i = 0;
	u8 u = 0;
	while (1)
	{

//		Frequency++;
		timer_delay_ms(100);
//		led0_on();
//		timer_delay_ms(500);
//		led0_off();
//		timer_delay_ms(500);
		if (i++ % 10 == 0)
		{
			u32 d = 0;
			if (u % 4 == 0)
			{
				d = pwm1;
			}
			if (u % 4 == 1)
			{
				d = pwm2;
			}
			if (u % 4 == 2)
			{
				d = pwm3;
			}
			if (u % 4 == 3)
			{
				d = pwm4;
			}
			u++;
			for (int i = 0; i < 4; i++)
			{
				u8 ch = d % 10;
				ch += 0x30;
				Uart1_PutChar(ch);
				d /= 10;
			}
			Uart1_PutChar('\n');
		}
	}
}

