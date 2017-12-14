#include <timer.h>

__IO u32 timer_delay = 0;
__IO u32 timer_tick = 0;

void timer_init()
{
	// 10us一次中断
	if (SysTick_Config(72))
	{
		while (1)
		{
		}
	}
	//关闭时钟
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

void timer_start()
{
	//打开时钟
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

void timer_delay_ms(u32 ms)
{
	timer_delay = ms;
	timer_delay *= 1000;
	while (timer_delay != 0)
	{
	}
}

void timer_delay_us(u32 us)
{
	timer_delay = us;
	while (timer_delay != 0)
	{
	}
}
void timer_decrement()
{
	timer_tick++;
	if (timer_delay != 0)
	{
		timer_delay--;
	}
}

void SysTick_Handler()
{
	timer_decrement();
}
