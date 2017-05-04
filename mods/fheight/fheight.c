/*
 * automatic.c
 *
 *  Created on: Oct 18, 2016
 *      Author: lidq
 */

#include "../fheight/fheight.h"

int r  = 0;
int st = 0;
pthread_t pthd;
s_engine* e = NULL;
s_params* p = NULL;

int __init(s_engine* engine, s_params* params)
{
	e = engine;
	p = params;

	st = 1;
	r  = 1;

	pthread_create(&pthd, (const pthread_attr_t*)NULL, (void* (*)(void*)) & fheight_automatic, NULL);

	return 0;
}

int __destory(s_engine* e, s_params* p)
{
	r = 0;

	return 0;
}

int __status() { return st; }

void fheight_automatic()
{
	//高度的增量式PID处理数据，当前、上一次
	float h_et = 0.0, h_et_1 = 0.0, h_et_2 = 0.0;
	float target_height = 0;
	float t		    = 0;
	float origin_v      = 0;
	while (r)
	{
		usleep(50 * 1000);

		if (e->mode == MODE_AUTO)
		{
			if (e->lock)
			{
				continue;
			}

			h_et      = e->height_target - e->height;
			e->h_devi = fheight_pid(h_et, h_et_1, &e->h_sum);
			h_et_1    = h_et;

			float v = origin_v + e->h_devi;
			fheight_ev_limit(&v);
			e->v = v;
		}
		else if (e->mode == MODE_MANUAL)
		{
			origin_v = e->v;
			if (e->lock || e->v < PROCTED_SPEED)
			{
				continue;
			}
		}
	}

	st = 0;
}

// PID反馈控制
float fheight_pid(float et, float et2, float* sum)
{
	//计算积分参数累加和，消除稳态误差
	*sum += p->h_ki * et;
	fheight_limit(sum);

	//对X、Y轴做PID反馈控制
	float devi = p->h_kp * et + (*sum) + p->h_kd * (et - et2);
	fheight_limit(&devi);
	//返回X、Y轴补偿值
	return devi;
}

//速度限幅
void fheight_limit(float* v)
{
	if (v == NULL)
	{
		return;
	}
	*v = *v > FHEIGHT_MAX_SPEED ? FHEIGHT_MAX_SPEED : *v;
	*v = *v < -FHEIGHT_MAX_SPEED ? -FHEIGHT_MAX_SPEED : *v;
}

//速度限幅
void fheight_ev_limit(float* v)
{
	if (v == NULL)
	{
		return;
	}
	*v = *v > MAX_SPEED_RUN_MAX ? MAX_SPEED_RUN_MAX : *v;
	*v = *v < MAX_SPEED_RUN_MIN ? MAX_SPEED_RUN_MIN : *v;
}
