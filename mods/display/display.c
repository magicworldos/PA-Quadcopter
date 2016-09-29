/*
 * display.c
 *
 *  Created on: Sep 27, 2016
 *      Author: lidq
 */

#include <display.h>

int r = 0;
int st = 0;
pthread_t pthd;
s_engine *e = NULL;
s_params *p = NULL;

int __init(s_engine *engine, s_params *params)
{
	e = engine;
	p = params;

#ifndef __DISPLAY_DISABLED__
	st = 1;
	r = 1;
	pthread_create(&pthd, (const pthread_attr_t*) NULL, (void* (*)(void*)) &run, NULL);
#endif

	printf("[ OK ] Display Init.\n");

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

void run()
{
	usleep(1000);

	while (r)
	{
		printf("[%s]", e->lock == 1 ? "LOCKED" : "UNLOCK");

#ifdef __DISPLAY_MODE_MORE__
		printf("[xyz: %+7.3f %+7.3f %+7.3f][g: %+7.3f %+7.3f %+7.3f][a: %+7.3f %+7.3f %+7.3f][s: %4d %4d %4d %4d]", e->x + e->dx + e->mx, e->y + e->dy + e->my, e->z + e->dz, e->gx + e->dgx, e->gy + e->dgy, e->gz + e->dgz, e->ax + e->dax, e->ay + e->day, e->az + e->daz, e->speed[0], e->speed[1], e->speed[2], e->speed[3]);
#endif

		if (p->ctl_type == 0)
		{
			printf("[pid: %+5.2f %+5.2f %+5.2f]", p->kp, p->ki, p->kd);
		}
		else if (p->ctl_type == 1)
		{
			printf("[pidv: %+5.2f %+5.2f %+5.2f]", p->kp_v, p->ki_v, p->kd_v);
		}
		if (p->ctl_type == 2)
		{
			printf("[pidz: %+5.2f %+5.2f %+5.2f]", p->kp_z, p->ki_z, p->kd_z);
		}
		else if (p->ctl_type == 3)
		{
			printf("[pidzv: %+5.2f %+5.2f %+5.2f]", p->kp_zv, p->ki_zv, p->kd_zv);
		}
		else if (p->ctl_type == 4)
		{
			printf("[pida: %+5.2f %+5.2f %+5.2f]", p->kp_a, p->ki_a, p->kd_a);
		}
		else if (p->ctl_type == 5)
		{
			printf("[cxy: %+5.2f %+5.2f]", p->cx, p->cy);
		}
		else if (p->ctl_type == 6)
		{
			printf("[ctl zero: %4d %4d %4d]", p->ctl_fb_zero, p->ctl_lr_zero, p->ctl_pw_zero);
		}

#ifndef __DISPLAY_MODE_MORE__
		else if (p->ctl_type == 7)
		{
			printf("[xyz: %+7.3f %+7.3f %+7.3f]", x_angle, y_angle, z_angle);
		}
		else if (p->ctl_type == 8)
		{
			printf("[g: %+7.3f %+7.3f %+7.3f]", e->gx + e->dgx, e->gy + e->dgy, e->gz + e->dgz);
		}
		else if (p->ctl_type == 9)
		{
			printf("[a: %+7.3f %+7.3f %+7.3f]", e->ax + e->dax, e->ay + e->day, e->az + e->daz);
		}
		else if (p->ctl_type == 10)
		{
			printf("[s: %4d %4d %4d %4d]", e->speed[0], e->speed[1], e->speed[2], e->speed[3]);
		}
#endif

		printf("\n");

		usleep(DISPLAY_SPEED * 1000);
	}

	st = 0;
}
