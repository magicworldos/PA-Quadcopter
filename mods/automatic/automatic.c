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
	while (1)
	{
		if (e->mode == MODE_TAKEOFF)
		{

		}
		else if (e->mode == MODE_FALLINGOFF)
		{

		}

		usleep(50 * 1000);
	}
}
