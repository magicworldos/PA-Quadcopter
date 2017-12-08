#include <typedef.h>
#include <led.h>
#include <timer.h>
#include <pwm.h>
#include <uart.h>
#include <exti.h>
#include <frame.h>

extern u16 pwm[6];
//extern u16 rc_data[6];


int main(int argc, char* argv[])
{
	SystemInit();

	led_init();

	EXTIX_Init();

	UART_Init();

	pwm_init();

	timer_init();

	timer_start();

	TIM_SetCompare1(TIM4, 500);
	TIM_SetCompare2(TIM4, 900);
	TIM_SetCompare3(TIM4, 1300);
	TIM_SetCompare4(TIM4, 1700);
//	u32 i = 0;
//	u8 u = 0;
	led0_off();
	while (1)
	{
		timer_delay_ms(10);
//		for (int i = 0; i < 6; i++)
//		{
//			rc_data[i] = pwm[i];
//		}
		frame_send_rc_data(pwm);

//		led0_on();
//		timer_delay_ms(500);
//		led0_off();
//		timer_delay_ms(500);
//		u32 d = 0;
//		if (u % 4 == 0)
//		{
//			d = pwm1;
//		}
//		if (u % 4 == 1)
//		{
//			d = pwm2;
//		}
//		if (u % 4 == 2)
//		{
//			d = pwm3;
//		}
//		if (u % 4 == 3)
//		{
//			d = pwm4;
//		}
//		u++;
//		for (int i = 0; i < 4; i++)
//		{
//			u8 ch = d % 10;
//			ch += 0x30;
//			Uart1_PutChar(ch);
//			d /= 10;
//		}
//		Uart1_PutChar('\n');
//		Uart1_PutChar('\r');
	}
}

