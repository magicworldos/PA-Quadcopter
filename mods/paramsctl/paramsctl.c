/*
 * paramsctl.c
 *
 *  Created on: May 24, 2016
 *
 *  四轴飞行控制器  Copyright (C) 2016  李德强
 */

#include <paramsctl.h>

s_params params_cache;

s32 st = 0;
s32 r = 0;
f32 ctl_step = 0.1;
pthread_t pthd;
s_engine* e = NULL;
s_params* p = NULL;

s32 __init(s_engine* engine, s_params* params)
{
	e = engine;
	p = params;
	st = 1;
	r = 1;

	//载入参数
	params_load();

	//启动键盘接收
	pthread_create(&pthd, (const pthread_attr_t*) NULL, (void* (*)(void*)) &params_input, NULL);

	printf("[ OK ] Paramsctl Init.\n");

	return 0;
}

s32 __destory(s_engine* e, s_params* p)
{
	r = 0;
	//保存缓存中的参数
	params_save();

	usleep(50 * 1000);

	resetTermios();

	st = 0;

	return 0;
}

s32 __status()
{
	return st;
}

//保存参数
void params_save()
{
	s8 quad_home[MAX_PATH_NAME];
	char* v = getenv("QUAD_HOME");
	memcpy(quad_home, v, strlen(v) + 1);
	s8 path[MAX_PATH_NAME];
	snprintf(path, MAX_PATH_NAME, "%s/" QUAD_PMS_FILE, quad_home);

	FILE* fp = fopen(path, "wb");
	if (fp == NULL)
	{
		printf("save params error!\n");
		return;
	}
	fwrite(&params_cache, sizeof(char), sizeof(s_params), fp);
	fclose(fp);
}

//载入参数
void params_load()
{
	s8 quad_home[MAX_PATH_NAME];
	char* v = getenv("QUAD_HOME");
	memcpy(quad_home, v, strlen(v) + 1);
	s8 path[MAX_PATH_NAME];
	snprintf(path, MAX_PATH_NAME, "%s/" QUAD_PMS_FILE, quad_home);

	FILE* fp = fopen(path, "rb");
	if (fp == NULL)
	{
		printf("load params error!\n");
		//如果载入失败则重置参数
		params_reset();
		memcpy(&params_cache, p, sizeof(s_params));
		return;
	}
	//载入参数
	fread(p, sizeof(char), sizeof(s_params), fp);
	memcpy(&params_cache, p, sizeof(s_params));
	fclose(fp);
}

//保存参数到缓存
void params_to_cache()
{
	memcpy(&params_cache, p, sizeof(s_params));
}

//从缓存载入参数
void params_from_cache()
{
	memcpy(p, &params_cache, sizeof(s_params));
}

//重置参数
void params_reset()
{
	// XY轴欧拉角PID参数
	p->kp = 88.0;
	p->ki = 0.0;
	p->kd = 0.0;
	// XY轴欧拉角PID参数
	p->v_kp = 5.2;
	p->v_ki = 0.8;
	p->v_kd = 42.0;
	// 垂直加速度PID参数
	p->vz_kp = 0.0;
	p->vz_ki = 0.0;
	p->vz_kd = 0.0;
	//摇控器3通道起始值
	p->ctl_fb_zero = 1500;
	p->ctl_lr_zero = 1500;
	p->ctl_pw_zero = 1100;
	p->ctl_md_zero = 1000;
	p->ctl_ud_zero = 1060;
	p->ctl_di_zero = 1000;
	//调参类型
	p->ctl_type = 0;
	//显示类型
	p->ctl_display = 0;
}

//键盘接收按键
void params_input()
{
	usleep(1000);

	//按键
	char ch = 0;
	while (r)
	{
		//按键
		ch = getch();
		// *
		if (ch == '*')
		{
			p->ctl_type = (++p->ctl_type % 3);
		}
		// 7
		else if (ch == '7')
		{
			if (p->ctl_type == 0)
			{
				p->kp += ctl_step;
			}
			else if (p->ctl_type == 1)
			{
				p->v_kp += ctl_step;
			}
			else if (p->ctl_type == 2)
			{
				p->vz_kp += ctl_step;
			}
		}
		// 8
		else if (ch == '8')
		{
			if (p->ctl_type == 0)
			{
				p->ki += ctl_step;
			}
			else if (p->ctl_type == 1)
			{
				p->v_ki += ctl_step;
			}
			else if (p->ctl_type == 2)
			{
				p->vz_ki += ctl_step;
			}
		}
		// 9
		else if (ch == '9')
		{
			if (p->ctl_type == 0)
			{
				p->kd += ctl_step;
			}
			else if (p->ctl_type == 1)
			{
				p->v_kd += ctl_step;
			}
			else if (p->ctl_type == 2)
			{
				p->vz_kd += ctl_step;
			}
		}
		// 4
		else if (ch == '4')
		{
			if (p->ctl_type == 0)
			{
				p->kp -= ctl_step;
			}
			else if (p->ctl_type == 1)
			{
				p->v_kp -= ctl_step;
			}
			else if (p->ctl_type == 2)
			{
				p->vz_kp -= ctl_step;
			}
		}
		// 5
		else if (ch == '5')
		{
			if (p->ctl_type == 0)
			{
				p->ki -= ctl_step;
			}
			else if (p->ctl_type == 1)
			{
				p->v_ki -= ctl_step;
			}
			else if (p->ctl_type == 2)
			{
				p->vz_ki -= ctl_step;
			}
		}
		// 6
		else if (ch == '6')
		{
			if (p->ctl_type == 0)
			{
				p->kd -= ctl_step;
			}
			else if (p->ctl_type == 1)
			{
				p->v_kd -= ctl_step;
			}
			else if (p->ctl_type == 2)
			{
				p->vz_kd -= ctl_step;
			}
		}
		//-
		else if (ch == '-')
		{
			e->v -= STEP_V;
			e->v = e->v < 0 ? 0 : e->v;
		}
		//+
		else if (ch == '+')
		{
			if (e->v == 0)
			{
				e->v = PROCTED_SPEED;
			}
			else
			{
				e->v += STEP_V;
				e->v = e->v > MAX_SPEED_RUN_MAX ? MAX_SPEED_RUN_MAX : e->v;
			}
		}
		// 0
		else if (ch == '0')
		{
			e->v = 0;
		}
		// 1
		else if (ch == '1')
		{
			ctl_step = 0.01;
		}
		// 2
		else if (ch == '2')
		{
			ctl_step = 0.1;
		}
		// 3
		else if (ch == '3')
		{
			ctl_step = 1;
		}
		else if (ch == 'q')
		{
			params_set_onoff(0);
		}
		else if (ch == 'w')
		{
			params_set_onoff(1);
		}
		else if (ch == 'e')
		{
			params_set_onoff(2);
		}
		else if (ch == 'r')
		{
			params_set_onoff(3);
		}
		else if (ch == 't')
		{
			params_set_onoff(4);
		}
		else if (ch == 'y')
		{
			params_set_onoff(5);
		}
		else if (ch == 'u')
		{
			params_set_onoff(6);
		}
		else if (ch == 'i')
		{
			params_set_onoff(7);
		}
		else if (ch == 'o')
		{
			params_set_onoff(8);
		}
		else if (ch == 'p')
		{
			params_set_onoff(9);
		}
		else if (ch == 'Q')
		{
			params_set_onoff(10);
		}
		else if (ch == 'W')
		{
			params_set_onoff(11);
		}
		else if (ch == 'E')
		{
			params_set_onoff(12);
		}
		else if (ch == 'R')
		{
			params_set_onoff(13);
		}
		else if (ch == 'T')
		{
			params_set_onoff(14);
		}
		else if (ch == 'Y')
		{
			params_set_onoff(15);
		}
		else if (ch == 'U')
		{
			params_set_onoff(16);
		}
		else if (ch == 'I')
		{
			params_set_onoff(17);
		}
		else if (ch == 'O')
		{
			params_set_onoff(18);
		}
		else if (ch == 'P')
		{
			params_set_onoff(19);
		}
		//保存所有参数到缓存
		else if (ch == 'S')
		{
			params_to_cache();
		}
		//读取缓存中的所有参数
		else if (ch == 'L')
		{
			params_from_cache();
		}
		//锁定电机
		else if (ch == 'J')
		{
			e->lock = 1;
		}
		//解锁电机
		else if (ch == 'F')
		{
			e->lock = 0;
		}
		usleep(1);
	}

	st = 0;
}

//设置显示开关
void params_set_onoff(s32 bit)
{
	//如果是开
	if ((p->ctl_display >> bit) & 0x1)
	{
		//关闭
		p->ctl_display &= ~(0x1 << bit);
	}
	//如果是关
	else
	{
		//打开
		p->ctl_display |= 0x1 << bit;
	}
}
