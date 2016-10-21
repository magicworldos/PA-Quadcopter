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
	//高度的增量式PID处理数据，当前、上一次
	float h_et = 0.0, h_et_1 = 0.0, h_et_2 = 0.0;
	int status = 0;

	while (r)
	{
		usleep(50 * 1000);

		if (e->mode == MODE_TAKEOFF)
		{
			status = 0;
			if (e->lock || e->v < PROCTED_SPEED)
			{
				continue;
			}

			h_et_2 = h_et_1;
			h_et_1 = h_et;
			h_et = e->target_height - e->height;

			float h_devi = automatic_pid(h_et, h_et_1, h_et_2);
			e->v += h_devi;
		}
		else if (e->mode == MODE_FALLINGOFF)
		{
			if (e->lock || e->v < PROCTED_SPEED)
			{
				continue;
			}

			//开始
			if (status == 0)
			{
				status = 1;
			}
			if (status == 1)
			{
				e->target_height = e->height + 2;
				status = 2;
			}
			if (status == 2)
			{
				h_et_2 = h_et_1;
				h_et_1 = h_et;
				h_et = e->target_height - e->height;

				float h_devi = automatic_pid(h_et, h_et_1, h_et_2);
				e->v += h_devi;
				if (e->target_height > 1.2)
				{
					e->target_height -= 0.005;
				}
				else if (e->target_height > 0.5)
				{
					e->target_height -= 0.003;
				}
				else
				{
					e->target_height -= 0.001;
				}
			}
		}
		else if (e->mode == MODE_MANUAL)
		{
			status = 0;
			if (e->lock || e->v < PROCTED_SPEED)
			{
				continue;
			}
		}
	}

	st = 0;
}

float automatic_pid(float et, float et_1, float et_2)
{
	return p->kp_h * (et - et_1) + p->ki_h * et + p->kd_h * (et - 2 * et_1 + et_2);
}
