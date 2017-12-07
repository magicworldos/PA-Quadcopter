#ifndef __TIMER_H
#define __TIMER_H

#include <typedef.h>

void timer_init();

void timer_start();

void timer_delay_ms(u32 ms);

void timer_delay_us(u32 us);

void timer_decrement();

#endif
