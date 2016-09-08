/*
 * paramsctl.c
 *
 *  Created on: May 24, 2016
 *      Author: lidq
 */

#include <paramsctl.h>

//使用ctl状态，退出时要重置console
int use_ctl = 0;
//q 0: x轴y轴PID参数
//w 1: 角速度PID_V参数
//e 2: z轴PID_Z参数
//r 3: 陀螺仪校准参数cx、cy、cz
int ctl_type = 0;
//多线程描述符
pthread_t pthdctl;
//引擎
extern s_engine engine;
//参数
extern s_params params;

//初始化参数控制
void params_start()
{
	//使用ctl状态置为1
	use_ctl = 1;
	//启动键盘接收
	pthread_create(&pthdctl, (const pthread_attr_t*) null, (void* (*)(void*)) &params_input, null);
}

//保存参数
void params_save()
{
	FILE *fp = fopen(QUAD_PMS_FILE, "wb");
	if (fp == null)
	{
		return;
	}
	fwrite(&params, sizeof(char), sizeof(s_params), fp);
	fclose(fp);
}

//载入参数
void params_load()
{
	FILE *fp = fopen(QUAD_PMS_FILE, "rb");
	if (fp == null)
	{
		//如果载入失败则重置参数
		params_reset();
		return;
	}
	//载入参数
	fread(&params, sizeof(char), sizeof(s_params), fp);
	fclose(fp);
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
		if (ch == 55)
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
				engine.cx += CTL_STEP;
			}
		}
		//8
		else if (ch == 56)
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
				engine.cy += CTL_STEP;
			}
		}
		//9
		else if (ch == 57)
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
		}
		//4
		else if (ch == 52)
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
				engine.cx -= CTL_STEP;
			}
		}
		//5
		else if (ch == 53)
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
				engine.cy -= CTL_STEP;
			}
		}
		//6
		else if (ch == 54)
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
		}
		//-
		else if (ch == 45)
		{
			for (int i = 0; i < 4; i++)
			{
				engine.v[i] -= STEP_V;
			}
		}
		//+
		else if (ch == 43)
		{
			for (int i = 0; i < 4; i++)
			{
				engine.v[i] += STEP_V;
			}
		}
		//0
		else if (ch == '0')
		{
			for (int i = 0; i < 4; i++)
			{
				engine.v[i] = 0;
			}
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
		//陀螺仪校准参数cx、cy、cz
		else if (ch == 'r')
		{
			ctl_type = 3;
		}
		//保存所有参数到文件
		else if (ch == 'S')
		{
			params_save();
		}
		//读取文件中的所有参数
		else if (ch == 'L')
		{
			params_load();
		}
	}
}

//清理控制器
void params_clear()
{
	if (use_ctl)
	{
		resetTermios();
	}
	exit(0);
}
