/*
 * pwm_out.h
 *
 *  Created on: Jun 25, 2017
 *      Author: lidq
 */

#ifndef __PWM_OUT_H
#define __PWM_OUT_H

#include <typedef.h>

#define TIM_PRESCALER		(72)
#define TIM_PERIOD			(2000)
#define TIM_PULSE			(0)

void pwm_out_init(void);

void pwm_out_gpio_config(void);

void pwm_out_mode_config(void);

void pwm_out_set_value();

#endif
