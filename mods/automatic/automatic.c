/*
 * automatic.c
 *
 *  Created on: Oct 18, 2016
 *      Author: lidq
 */

#include <automatic.h>

int r = 0;
int st = 0;
pthread_t pthd;
s_engine *e = NULL;
s_params *p = NULL;

float finally_height = 0;

int __init(s_engine *engine, s_params *params)
{
	e = engine;
	p = params;

	st = 1;
	r = 1;

	pthread_create(&pthd, (const pthread_attr_t*) NULL, (void* (*)(void*)) &automatic, NULL);

	return 0;
}

int __destory(s_engine *e, s_params *p)
{
	r = 0;

	return 0;
}

int __status()
{
	return st;
}

void automatic()
{
	//XYZ的增量式PID处理数据，当前、上一次
	float h_et = 0.0, h_et_1 = 0.0;
	float sum = 0;
	while (r)
	{
		if (e->mode == MODE_TAKEOFF)
		{
			h_et_1 = h_et;
			h_et = (e->target_height - e->height) * 100;

			float v = automatic_pid(h_et, h_et_1, &sum);
		}
		else if (e->mode == MODE_FALLINGOFF)
		{

		}

		usleep(50 * 1000);
	}

	st = 0;
}

float automatic_pid(float et, float et_1, float *sum)
{
	*sum += p->ki_h * et;
	//增量式PID反馈控制
	return p->kp_h * et + (*sum) + p->kd_h * (et - et_1);
}
