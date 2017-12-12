/*
 * timer_tick.h
 *
 *  Created on: Jun 25, 2017
 *      Author: lidq
 */

#ifndef __TIMER_INIT_H
#define __TIMER_INIT_H

#include <typedef.h>

void timer_init(void);

void timer_start(void);

void timer_delay_ms(u32 ms);

void timer_delay_us(u32 us);

void timer_decrement(void);

#endif
