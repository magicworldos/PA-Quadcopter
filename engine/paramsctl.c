/*
 * paramsctl.c
 *
 *  Created on: May 24, 2016
 *
 *  四轴飞行控制器  Copyright (C) 2016  李德强
 */

#include <paramsctl.h>

//使用ctl状态，退出时要重置console
int use_ctl = 0;
//q 0: x轴y轴PID参数
//w 1: 角速度PID_V参数
//e 2: z轴PID_Z参数
//r 3: 陀螺仪校准参数cx、cy、cz
int ctl_type = 9;
int openfile = 0;
//多线程描述符
pthread_t pthdctl;
//引擎
extern s_engine engine;
//参数
extern s_params params;

//保存参数
void params_save()
{
	FILE *fp = fopen(QUAD_PMS_FILE, "wb");
	if (fp == null)
	{
		printf("save params error!\n");
		return;
	}
	openfile = 1;
	fwrite(&params, sizeof(char), sizeof(s_params), fp);
	fclose(fp);
	openfile = 0;
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
		return;
	}
	openfile = 1;
	//载入参数
	fread(&params, sizeof(char), sizeof(s_params), fp);
	fclose(fp);
	openfile = 0;
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
	params.kp_a = 13.8;
	params.ki_a = 12.6;
	params.kd_a = 8.8;
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
				engine.cx += CTL_STEP;
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
				engine.cy += CTL_STEP;
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
				engine.cx -= CTL_STEP;
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
				engine.cy -= CTL_STEP;
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
			engine.v += STEP_V;
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
		//陀螺仪校准参数cx、cy、cz
		else if (ch == 'y')
		{
			ctl_type = 5;
		}
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
		//保存所有参数到文件
		else if (ch == 's')
		{
			if (!openfile)
			{
				params_save();
			}
		}
		//读取文件中的所有参数
		else if (ch == 'l')
		{
			if (!openfile)
			{
				params_load();
			}
		}
	}
}
