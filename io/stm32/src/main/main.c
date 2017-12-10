#include <typedef.h>
#include <led.h>
#include <timer.h>
#include <pwm.h>
#include <uart.h>
#include <exti.h>
#include <frame.h>

extern u16 motor[6];

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

	UART_Init();

	pwm_init();

	timer_init();

	timer_start();

	TIM_SetCompare1(TIM4, 0);
	TIM_SetCompare2(TIM4, 0);
	TIM_SetCompare3(TIM4, 0);
	TIM_SetCompare4(TIM4, 0);

	led0_off();

	while (1)
	{
		get_motor_pwm();
		pwm_limit(motor, 4);
		TIM_SetCompare1(TIM4, motor[0]);
		TIM_SetCompare2(TIM4, motor[1]);
		TIM_SetCompare3(TIM4, motor[2]);
		TIM_SetCompare4(TIM4, motor[3]);

		timer_delay_ms(1);
	}
}

