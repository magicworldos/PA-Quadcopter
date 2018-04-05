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

void led_on()
{
	GPIO_ResetBits(GPIOA, GPIO_Pin_4);
//	GPIO_SetBits(GPIOA, GPIO_Pin_5);
}

void led_off()
{
	GPIO_SetBits(GPIOA, GPIO_Pin_4);
//	GPIO_ResetBits(GPIOA, GPIO_Pin_5);
}

void led_blink(u32 usecs)
{
	if (timer_tick % (2 * usecs) < usecs)
	{
		led_on();
	}
	else
	{
		led_off();
	}
}
