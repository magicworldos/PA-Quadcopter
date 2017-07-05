/*
 * paramsctl.c
 *
 *  Created on: May 24, 2016
 *
 *  四轴飞行控制器  Copyright (C) 2016  李德强
 */

#include <paramsctl.h>

s_params params_cache;

int st	 = 0;
int r	  = 0;
float ctl_step = 1;
pthread_t pthd;
s_engine* e = NULL;
s_params* p = NULL;

int __init(s_engine* engine, s_params* params)
{
	e  = engine;
	p  = params;
	st = 1;
	r  = 1;

	//载入参数
	params_load();

	//启动键盘接收
	pthread_create(&pthd, (const pthread_attr_t*)NULL, (void* (*)(void*)) & params_input, NULL);

	printf("[ OK ] Paramsctl Init.\n");

	return 0;
}

int __destory(s_engine* e, s_params* p)
{
	r = 0;
	//保存缓存中的参数
	params_save();

	usleep(50 * 1000);

	resetTermios();

	st = 0;

	return 0;
}

int __status() { return st; }

//保存参数
void params_save()
{
	FILE* fp = fopen(QUAD_PMS_FILE, "wb");
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
	FILE* fp = fopen(QUAD_PMS_FILE, "rb");
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
void params_to_cache() { memcpy(&params_cache, p, sizeof(s_params)); }

//从缓存载入参数
void params_from_cache() { memcpy(p, &params_cache, sizeof(s_params)); }

//重置参数
void params_reset()
{
	// XY轴欧拉角PID参数
	p->kp = 122.0;
	p->ki = 8.0;
	p->kd = 0.0;
	// XY轴欧拉角PID参数
	p->v_kp = 10.0;
	p->v_ki = 0.0;
	p->v_kd = 6.0;
	// XY轴欧拉角PID参数
	p->h_kp = 0.0;
	p->h_ki = 0.0;
	p->h_kd = 0.0;
	// XY轴中心点校正补偿
	p->cx = 0;
	p->cy = 0;
	//摇控器3通道起始值
	p->ctl_fb_zero = 1500;
	p->ctl_lr_zero = 1500;
	p->ctl_pw_zero = 1100;
	p->ctl_md_zero = 1000;
	p->ctl_ud_zero = 1060;
	p->ctl_di_zero = 1000;
	//显示类型
	p->ctl_type = 1;
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
		// 7
		if (ch == '7')
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
				p->h_kp += ctl_step;
			}
			else if (p->ctl_type == 3)
			{
				//p->cx += ctl_step;
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
				p->h_ki += ctl_step;
			}
			else if (p->ctl_type == 3)
			{
				//p->cy += ctl_step;
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
				p->h_kd += ctl_step;
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
				p->h_kp -= ctl_step;
			}
			else if (p->ctl_type == 3)
			{
				//p->cx -= ctl_step;
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
				p->h_ki -= ctl_step;
			}
			else if (p->ctl_type == 3)
			{
				//p->cy -= ctl_step;
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
				p->h_kd -= ctl_step;
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
			ctl_step = 0.1;
		}
		// 2
		else if (ch == '2')
		{
			ctl_step = 1;
		}
		// 3
		else if (ch == '3')
		{
			ctl_step = 10;
		}
		// x轴y轴PID参数
		else if (ch == 'q')
		{
			p->ctl_type = 0;
		}
		//角速度PID_V参数
		else if (ch == 'w')
		{
			p->ctl_type = 1;
		}
		//角速度PID_V参数
		else if (ch == 'e')
		{
			p->ctl_type = 2;
		}
		//角速度PID_V参数
		else if (ch == 'r')
		{
			p->ctl_type = 3;
		}
		//陀螺仪校准参数cx、cy、cz
		else if (ch == 'o')
		{
			p->ctl_type = 4;
		}
		//摇控器起始读数
		else if (ch == 'p')
		{
			p->ctl_type = 5;
		}
		//角度
		else if (ch == 'z')
		{
			p->ctl_type = 6;
		}
		//角速度
		else if (ch == 'x')
		{
			p->ctl_type = 7;
		}
		//电机转速
		else if (ch == 'c')
		{
			p->ctl_type = 8;
		}
		//电机转速
		else if (ch == 'v')
		{
			p->ctl_type = 9;
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
