#include <typedef.h>
#include <led.h>
#include <timer.h>
#include <pwm.h>
//#include <pwm_in.h>
#include <uart.h>
#include <exti.h>
#include <frame.h>

extern u16 pwm[6];

void pwm_limit(u16 *pwm, int len)
{
	for (int i = 0; i < len; i++)
	{
		pwm[i] = pwm[i] > 2000 ? 2000 : pwm[i];
	}
}

int main(int argc, char* argv[])
{
	SystemInit();

	led_init();

	EXTIX_Init();
	//TIM3_PWM_Init();

	UART_Init();

	pwm_init();

	timer_init();

	timer_start();

	TIM_SetCompare1(TIM4, 500);
	TIM_SetCompare2(TIM4, 900);
	TIM_SetCompare3(TIM4, 1300);
	TIM_SetCompare4(TIM4, 1700);

	led0_off();

	while (1)
	{
		pwm_limit(pwm, 6);
		frame_send_rc_data(pwm);

		timer_delay_ms(10);
	}
}

