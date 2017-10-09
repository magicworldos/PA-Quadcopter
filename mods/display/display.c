/*
 * display.c
 *
 *  Created on: Sep 27, 2016
 *      Author: lidq
 */

#include <display.h>

s32 r = 0;
s32 st = 0;
pthread_t pthd;
s_engine* e = NULL;
s_params* p = NULL;

s32 __init(s_engine* engine, s_params* params)
{
	e = engine;
	p = params;

	st = 1;
	r = 1;
	pthread_create(&pthd, (const pthread_attr_t*) NULL, (void* (*)(void*)) &display_run, NULL);

	printf("[ OK ] Display Init.\n");

	return 0;
}

s32 __destory(s_engine* e, s_params* p)
{
	r = 0;

	return 0;
}

s32 __status()
{
	return st;
}

void display_run()
{
	usleep(1000);

	while (r)
	{
		printf("[%s]", e->lock == 1 ? "LOCKED" : "UNLOCK");
		if (display_get_onoff(7))
		{
			if (p->ctl_type == 0)
			{
				printf("[%s]", " PID");
			}
			else if (p->ctl_type == 1)
			{
				printf("[%s]", "VPID");
			}
			else if (p->ctl_type == 2)
			{
				printf("[%s]", "APID");
			}

		}
		if (display_get_onoff(0))
		{
			printf("[v: %4.0f]", e->v);
		}
		if (display_get_onoff(1))
		{
			printf("[xyz: %+7.3f %+7.3f %+7.3f]", e->tx, e->ty, e->tz);
		}
		if (display_get_onoff(2))
		{
			printf("[ctlxy: %+7.3f %+7.3f]", e->ctlmx, e->ctlmy);
		}
		if (display_get_onoff(3))
		{
			printf("[g: %+7.3f %+7.3f %+7.3f]", e->tgx, e->tgy, e->tgz);
		}
		if (display_get_onoff(4))
		{
			printf("[pid: %+5.2f %+5.2f %+5.2f]", p->kp, p->ki, p->kd);
		}
		if (display_get_onoff(5))
		{
			printf("[vpid: %+5.2f %+5.2f %+5.2f]", p->v_kp, p->v_ki, p->v_kd);
		}
		if (display_get_onoff(6))
		{
			printf("[ctl zero: %4d %4d %4d %4d %4d %4d]", p->ctl_fb_zero, p->ctl_lr_zero, p->ctl_pw_zero, p->ctl_md_zero, p->ctl_ud_zero, p->ctl_di_zero);
		}
		if (display_get_onoff(8))
		{
			printf("[a: %+7.3f %+7.3f %+7.3f]", e->ax, e->ay, e->az);
		}
		if (display_get_onoff(9))
		{
			printf("[vz: %+5.2f]", e->vz);
		}
		if (display_get_onoff(10))
		{
			printf("[vz_devi: %+5.2f]", e->vz_devi);
		}
		if (display_get_onoff(11))
		{
			printf("[vzpid: %+5.2f %+5.2f %+5.2f]", p->vz_kp, p->vz_ki, p->vz_kd);
		}

		printf("\n");

		usleep(DISPLAY_SPEED * 1000);
	}

	st = 0;
}

//取得显示开关
s32 display_get_onoff(s32 bit)
{
	return (p->ctl_display >> bit) & 0x1;
}
