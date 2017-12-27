#include <typedef.h>
#include <pwm_in.h>
#include <pwm_out.h>
#include <led_light.h>
#include <timer_tick.h>
#include <serial_port.h>

extern u16 pwm_in[8];
extern u16 pwm_out[4];
extern u32 pwm_in_error_count;
extern u32 pwm_out_error_count;

int main(int argc, char* argv[])
{
	SystemInit();

	led_init();

	led_off();

	pwm_in_init();

	pwm_out_init();

	serial_port_init();

	timer_init();

	timer_start();

	u32 i = 0;

	while (1)
	{
		if (pwm_in_error_count < 2 * PWM_ERR_MAX)
		{
			pwm_in_error_count++;
		}
		if (pwm_in_error_count < PWM_ERR_MAX)
		{
			serial_port_frame_send_rc(pwm_in);
		}

		serial_port_frame_recv_pwm(pwm_out);

		if (pwm_out_error_count < 2 * PWM_ERR_MAX)
		{
			pwm_out_error_count++;
		}
		if (pwm_out_error_count > PWM_ERR_MAX)
		{
			memset(pwm_out, 0, sizeof(u16) * 4);
		}
		pwm_out_set_value();

		led_blink(500 * 1000);

		timer_delay_ms(5);
	}
}

