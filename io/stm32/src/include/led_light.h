/*
 * led_light.h
 *
 *  Created on: Jun 25, 2017
 *      Author: lidq
 */

#ifndef __LED_LIGHT_H
#define __LED_LIGHT_H

#include <typedef.h>

void led_init(void);

void led0_on(void);

void led0_off(void);

void led0_blink(u32 usecs);

void led1_on(void);

void led1_off(void);

void led1_blink(u32 usecs);

void led_blink(u32 usecs);

#endif
