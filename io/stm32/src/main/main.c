#include <typedef.h>
#include <pwm_in.h>
#include <pwm_out.h>
#include <led_light.h>
#include <timer_tick.h>
#include <serial_port.h>

extern u16 pwm_in[8];
extern u16 pwm[4];

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
		serial_port_frame_send_rc(pwm_in);

		serial_port_frame_recv_pwm(pwm);

		pwm_out_set_value();

		led_blink(500 * 1000);

		timer_delay_ms(5);
	}
}

