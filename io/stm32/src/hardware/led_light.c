/*
 * led_light.c
 *
 *  Created on: Jun 25, 2017
 *      Author: lidq
 */

#include <led_light.h>

extern u32 timer_tick;

void led_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void led0_on()
{
	GPIO_ResetBits(GPIOA, GPIO_Pin_4);
}

void led0_off()
{
	GPIO_SetBits(GPIOA, GPIO_Pin_4);
}

void led1_on()
{
	GPIO_ResetBits(GPIOA, GPIO_Pin_5);
}

void led1_off()
{
	GPIO_SetBits(GPIOA, GPIO_Pin_5);
}

void led0_blink(u32 usecs)
{
	if (timer_tick % usecs < usecs / 2)
	{
		led0_on();
	}
	else
	{
		led0_off();
	}
}

void led1_blink(u32 usecs)
{
	if (timer_tick % usecs < usecs / 2)
	{
		led1_on();
	}
	else
	{
		led1_off();
	}
}

void led_blink(u32 usecs)
{
	if (timer_tick % usecs < usecs / 2)
	{
		led0_on();
		led1_off();
	}
	else
	{
		led0_off();
		led1_on();
	}
}
