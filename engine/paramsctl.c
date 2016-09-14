/*
 * paramsctl.c
 *
 *  Created on: May 24, 2016
 *
 *  四轴飞行控制器  Copyright (C) 2016  李德强
 */

#include <paramsctl.h>

//q 0: x轴y轴PID参数
//w 1: 角速度PID_V参数
//e 2: z轴PID_Z参数
//r 3: 陀螺仪校准参数cx、cy、cz
int ctl_type = 9;
//多线程描述符
pthread_t pthdctl;
//引擎
extern s_engine engine;
//参数
extern s_params params;
//参数缓存
extern s_params params_cache;

//保存参数
void params_save()
{
	FILE *fp = fopen(QUAD_PMS_FILE, "wb");
	if (fp == null)
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
	FILE *fp = fopen(QUAD_PMS_FILE, "rb");
	if (fp == null)
	{
		printf("save params error!\n");
		//如果载入失败则重置参数
		params_reset();
		memcpy(&params_cache, &params, sizeof(s_params));
		return;
	}
	//载入参数
	fread(&params, sizeof(char), sizeof(s_params), fp);
	memcpy(&params_cache, &params, sizeof(s_params));
	fclose(fp);
}

//保存参数到缓存
void params_to_cache()
{
	memcpy(&params_cache, &params, sizeof(s_params));
}

//从缓存载入参数
void params_from_cache()
{
	memcpy(&params, &params_cache, sizeof(s_params));
}

//重置参数
void params_reset()
{
	//XY轴欧拉角PID参数
	params.kp = 9.4;
	params.ki = 1.8;
	params.kd = 8.0;
	//旋转角速度PID参数
	params.kp_v = 9.3;
	params.ki_v = 5.4;
	params.kd_v = 8.2;
	//Z轴欧拉角PID参数
	params.kp_z = 3.6;
	params.ki_z = 0.8;
	params.kd_z = 2.8;
	//XY轴加速度PID参数
	params.kp_a = 28.0;
	params.ki_a = 18.6;
	params.kd_a = 23.8;
	//XY轴中心点校正补偿
	params.cx = 0.0;
	params.cy = -3.5;
	//摇控器3通道起始值
	params.ctl_fb_zero = 1407;
	params.ctl_lr_zero = 1610;
	params.ctl_pw_zero = 1011;
}

//键盘接收按键
void params_input()
{
	//按键
	char ch = 0;
	while (1)
	{
		//按键
		ch = getch();
		//7
		if (ch == '7')
		{
			if (ctl_type == 0)
			{
				params.kp += CTL_STEP;
			}
			else if (ctl_type == 1)
			{
				params.kp_v += CTL_STEP;
			}
			else if (ctl_type == 2)
			{
				params.kp_z += CTL_STEP;
			}
			else if (ctl_type == 3)
			{
				params.kp_a += CTL_STEP;
			}
			else if (ctl_type == 4)
			{
				params.cx += CTL_STEP;
			}
		}
		//8
		else if (ch == '8')
		{
			if (ctl_type == 0)
			{
				params.ki += CTL_STEP;
			}
			else if (ctl_type == 1)
			{
				params.ki_v += CTL_STEP;
			}
			else if (ctl_type == 2)
			{
				params.ki_z += CTL_STEP;
			}
			else if (ctl_type == 3)
			{
				params.ki_a += CTL_STEP;
			}
			else if (ctl_type == 4)
			{
				params.cy += CTL_STEP;
			}
		}
		//9
		else if (ch == '9')
		{
			if (ctl_type == 0)
			{
				params.kd += CTL_STEP;
			}
			else if (ctl_type == 1)
			{
				params.kd_v += CTL_STEP;
			}
			else if (ctl_type == 2)
			{
				params.kd_z += CTL_STEP;
			}
			else if (ctl_type == 3)
			{
				params.kd_a += CTL_STEP;
			}
		}
		//4
		else if (ch == '4')
		{
			if (ctl_type == 0)
			{
				params.kp -= CTL_STEP;
			}
			else if (ctl_type == 1)
			{
				params.kp_v -= CTL_STEP;
			}
			else if (ctl_type == 2)
			{
				params.kp_z -= CTL_STEP;
			}
			else if (ctl_type == 3)
			{
				params.kp_a -= CTL_STEP;
			}
			else if (ctl_type == 4)
			{
				params.cx -= CTL_STEP;
			}
		}
		//5
		else if (ch == '5')
		{
			if (ctl_type == 0)
			{
				params.ki -= CTL_STEP;
			}
			else if (ctl_type == 1)
			{
				params.ki_v -= CTL_STEP;
			}
			else if (ctl_type == 2)
			{
				params.ki_z -= CTL_STEP;
			}
			else if (ctl_type == 3)
			{
				params.ki_a -= CTL_STEP;
			}
			else if (ctl_type == 4)
			{
				params.cy -= CTL_STEP;
			}
		}
		//6
		else if (ch == '6')
		{
			if (ctl_type == 0)
			{
				params.kd -= CTL_STEP;
			}
			else if (ctl_type == 1)
			{
				params.kd_v -= CTL_STEP;
			}
			else if (ctl_type == 2)
			{
				params.kd_z -= CTL_STEP;
			}
			else if (ctl_type == 3)
			{
				params.kd_a -= CTL_STEP;
			}
		}
		//-
		else if (ch == '-')
		{
			engine.v -= STEP_V;
		}
		//+
		else if (ch == '+')
		{
			if (engine.v == 0)
			{
				engine.v = PROCTED_SPEED;
			}
			else
			{
				engine.v += STEP_V;
			}
		}
		//0
		else if (ch == '0')
		{
			engine.v = 0;
		}
		//x轴y轴PID参数
		else if (ch == 'q')
		{
			ctl_type = 0;
		}
		//角速度PID_V参数
		else if (ch == 'w')
		{
			ctl_type = 1;
		}
		//z轴PID_Z参数
		else if (ch == 'e')
		{
			ctl_type = 2;
		}
		//xy轴加速度PID_A参数
		else if (ch == 'r')
		{
			ctl_type = 3;
		}
		//陀螺仪校准参数cx、cy、cz
		else if (ch == 't')
		{
			ctl_type = 4;
		}
		//摇控器起始读数
		else if (ch == 'y')
		{
			ctl_type = 5;
		}
#ifndef __DISPLAY_MODE_MORE__
		//角度
		else if (ch == 'u')
		{
			ctl_type = 6;
		}
		//角速度
		else if (ch == 'i')
		{
			ctl_type = 7;
		}
		//加速度
		else if (ch == 'o')
		{
			ctl_type = 8;
		}
		//电机转速
		else if (ch == 'p')
		{
			ctl_type = 9;
		}
#endif
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
		else if (ch == 'F')
		{
			engine.lock = 1;
		}
		//解锁电机
		else if (ch == 'J')
		{
			engine.lock = 0;
		}
	}
}
