#include <typedef.h>
#include <led.h>
#include <timer.h>
#include <pwm.h>

int main(int argc, char* argv[])
{
	SystemInit();

	led_init();

	tim4_pwm_init();

	timer_init();

	timer_start();

	TIM_SetCompare1(TIM4, 1000);

	while (1)
	{
		TIM_SetCompare4(TIM4, 1000);
		led0_on();
		timer_delay_ms(10000);
		TIM_SetCompare4(TIM4, 1800);
		led0_off();
		timer_delay_ms(10000);
	}
}

