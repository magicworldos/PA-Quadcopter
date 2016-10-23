/*
 * hcsr04.h
 *
 *  Created on: Sep 9, 2016
 *  
 *  四轴飞行控制器  Copyright (C) 2016  李德强 
 */

#ifndef INCLUDE_HCSR04_H_
#define INCLUDE_HCSR04_H_

#include <typedef.h>

//定义HC-SR04引脚
#define PORT_CS_TRIG        (15)
#define PORT_CS_ECHO        (16)

typedef struct
{
	struct timeval timer_start;
	struct timeval timer_end;
} s_dis;

int __init(s_engine *engine, s_params *params);

int __destory(s_engine *e, s_params *p);

int __status();

void distance_trig();

void distance();

float kalman_filter(float est, float est_devi, float measure, float measure_devi, float *devi);

#endif /* INCLUDE_GY953_H_ */
