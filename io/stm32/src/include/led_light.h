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

void led_on(void);

void led_off(void);

void led_blink(u32 usecs);

#endif
