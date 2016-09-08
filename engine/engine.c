/*
 * engine.c
 *
 *  Created on: Apr 12, 2016
 *
 *  四轴飞行控制器  Copyright (C) 2016  李德强
 */

#include <engine.h>
#include <driver.h>
#include <paramsctl.h>

//引擎
s_engine engine;
//参数
s_params params;
//信号量
sem_t sem_engine;
//多线程描述符
pthread_t pthd;
//显示状态
extern int ctl_type;
//显示摇控器读数
int ctl_fb = 0;
int ctl_lr = 0;
int ctl_pw = 0;

//启动引擎
void engine_start(int argc, char *argv[])
{
	//处理Ctrl + C退出信号
	signal(SIGINT, (void (*)(int)) &engine_exception);

	//初始化引擎信号量
	sem_init(&sem_engine, 0, 0);

	//处理启动参数
	if (argc >= 2)
	{
		//正常模式，飞行，调参
		if (strcmp(argv[1], "--fly") == 0)
		{

#ifndef __PC_DEBUG__
			//初始化驱动
			driver_setup();
#endif
			//重置引擎
			engine_reset(&engine);
			//载入参数
			params_load();
			//启动MPU6050陀螺仪数据读入线程
			pthread_create(&pthd, (const pthread_attr_t*) null, (void* (*)(void*)) &engine_mpu, null);
			//启动飞行引擎
			pthread_create(&pthd, (const pthread_attr_t*) null, (void* (*)(void*)) &engine_fly, null);
			//启动参数调整
			pthread_create(&pthd, (const pthread_attr_t*) null, (void* (*)(void*)) &params_start, null);
			//主线程休眠
			sem_wait(&sem_engine);

			return;
		}

		//校准摇控器模式
		if (strcmp(argv[1], "--ctl") == 0)
		{
#ifndef __PC_DEBUG__
			//初始化驱动
			driver_setup();
#endif

			while (1)
			{
				printf("FB: %4d\tLR: %4d\tPW: %4d\n", ctl_fb, ctl_lr, ctl_pw);
				usleep(10 * 1000);
			}
			return;
		}

		//电机调试模式
		if (strcmp(argv[1], "--test") == 0)
		{
			if (argc == 5)
			{
				int en_no;
				int en_port;
				int en_speed;
				int en_msecs;

				//电机所在GPIO引脚编号
				sscanf(argv[2], "%d", &en_port);
				//调定速度
				sscanf(argv[3], "%d", &en_speed);
				//调试时长，毫秒
				sscanf(argv[4], "%d", &en_msecs);
				//开始调试电机
				driver_ent_run(en_port, en_speed, en_msecs);

				return;
			}
		}
	}

	printf("unknown option ...\n");
	printf("usage: quadcopter\n");
	printf("\t[--fly: Fly with remote control and adjust quadcopter's parameters by keybroad.]\n");
	printf("\t[--ctl: Display remote control values.]\n");
	printf("\t[--test: Test the connection to the motor come in raspberry.]\n");
	printf("\tex. quadcopter --test [GPIO] [SPEED] [MSECS]\n");
	return;
}

//引擎核心算法平衡算法
void engine_fly()
{
	s_engine *e = &engine;

	//XYZ的增量式PID处理数据，当前、上一次、上上次
	float x_et = 0.0, x_et_1 = 0.0, x_et_2 = 0.0;
	float y_et = 0.0, y_et_1 = 0.0, y_et_2 = 0.0;
	float z_et = 0.0, z_et_1 = 0.0, z_et_2 = 0.0;

	//XY轴旋转角速度的增量式PID处理数据，当前、上一次、上上次
	float xv_et = 0.0, xv_et_1 = 0.0, xv_et_2 = 0.0;
	float yv_et = 0.0, yv_et_1 = 0.0, yv_et_2 = 0.0;

	while (1)
	{
		//处理X轴欧拉角平衡补偿
		//计算角度：欧拉角x + 校准补偿dx + 中心补偿cx + 移动倾斜角mx
		float x_angle = e->x + e->dx + e->cx + e->mx;
		//角度范围校验
		x_angle = x_angle < -MAX_ANGLE ? -MAX_ANGLE : x_angle;
		x_angle = x_angle > MAX_ANGLE ? MAX_ANGLE : x_angle;
		//设置PID数据
		x_et_2 = x_et_1;
		x_et_1 = x_et;
		x_et = x_angle;
		//使用XY轴的欧拉角的PID反馈控制算法
		float x_devi = engine_pid(x_et, x_et_1, x_et_2);
		//得出引擎的X轴平衡补偿
		e->v_devi[1] = x_devi;
		e->v_devi[3] = -x_devi;

		//处理Y轴欧拉角平衡补偿
		//计算角度：欧拉角y + 校准补偿dy + 中心补偿cy + 移动倾斜角my
		float y_angle = e->y + e->dy + e->cy + e->my;
		//角度范围校验
		y_angle = y_angle < -MAX_ANGLE ? -MAX_ANGLE : y_angle;
		y_angle = y_angle > MAX_ANGLE ? MAX_ANGLE : y_angle;
		//设置PID数据
		y_et_2 = y_et_1;
		y_et_1 = y_et;
		y_et = y_angle;
		//使用XY轴的欧拉角的PID反馈控制算法
		float y_devi = engine_pid(y_et, y_et_1, y_et_2);
		//得出引擎的Y轴平衡补偿
		e->v_devi[0] = +y_devi;
		e->v_devi[2] = -y_devi;

		//处理Z轴欧拉角平衡补偿
		//计算角度：欧拉角z + 校准补偿dz
		float z_angle = e->z + e->dz;
		//设置PID数据
		z_et_2 = z_et_1;
		z_et_1 = z_et;
		z_et = z_angle;
		//使用Z轴的欧拉角的PID反馈控制算法
		float z_devi = engine_pid_z(z_et, z_et_1, z_et_2);
		//得出引擎的Y轴平衡补偿
		e->v_devi[0] += z_devi;
		e->v_devi[2] += z_devi;
		e->v_devi[1] += -z_devi;
		e->v_devi[3] += -z_devi;

		//处理XY轴旋转角速度平衡补偿
		float xv_devi = 0;
		float yv_devi = 0;

		//设置X轴PID数据
		xv_et_2 = xv_et_1;
		xv_et_1 = xv_et;
		xv_et = e->gx;
		//使用X轴的旋转角速度的PID反馈控制算法
		xv_devi = engine_pid_v(xv_et, xv_et_1, xv_et_2);

		//设置Y轴PID数据
		yv_et_2 = yv_et_1;
		yv_et_1 = yv_et;
		yv_et = e->gy;
		//使用Y轴的旋转角速度的PID反馈控制算法
		yv_devi = engine_pid_v(yv_et, yv_et_1, yv_et_2);

		//对引擎的4个轴做角速度平衡补偿
		e->v_devi[1] += -xv_devi;
		e->v_devi[3] += +xv_devi;
		e->v_devi[0] += +yv_devi;
		e->v_devi[2] += -yv_devi;

		//引擎运转，调用驱动，调控电机转数
		engine_move(e);

		printf("[xyz: %+7.3f %+7.3f %+7.3f][gxyz: %+7.3f %+7.3f %+7.3f][s0: %3d %3d %3d %3d]", x_angle, y_angle, z_angle, e->gx, e->gy, e->gz, e->speed[0], e->speed[1], e->speed[2], e->speed[3]);
		if (ctl_type == 0)
		{
			printf("[pid: %+5.2f %+5.2f %+5.2f]", params.kp, params.ki, params.kd);
		}
		else if (ctl_type == 1)
		{
			printf("[pid_v: %+5.2f %+5.2f %+5.2f]", params.kp_v, params.ki_v, params.kd_v);
		}
		else if (ctl_type == 2)
		{
			printf("[pid_z: %+5.2f %+5.2f %+5.2f]", params.kp_z, params.ki_z, params.kd_z);
		}
		else if (ctl_type == 3)
		{
			printf("[c_xy: %+5.2f %+5.2f]", e->cx, e->cy);
		}
		printf("\n");

		//原定计算频率1000Hz，但由于MPU6050的输出为100hz只好降低到100hz
		usleep(ENG_TIMER * 1000);
	}
}

//引擎运转
void engine_move(s_engine *e)
{
	//设置电机实际转数
	for (int i = 0; i < 4; i++)
	{
		//设置电机转数为做引擎速度 + 补偿
		e->speed[i] = (int) (e->v[i] + e->v_devi[i]);
	}

	//校验电机转数范围
	engine_rechk_speed(e);

	//调用驱动设置电机转数
	driver_set_speed0(e->speed[0]);
	driver_set_speed1(e->speed[1]);
	driver_set_speed2(e->speed[2]);
	driver_set_speed3(e->speed[3]);
}

//校验电机转数范围
void engine_rechk_speed(s_engine *e)
{
	for (int i = 0; i < 4; i++)
	{
		if (e->v[i] > MAX_SPEED_RUN_MAX)
		{
			e->v[i] = MAX_SPEED_RUN_MAX;
		}
		if (e->v[i] < MAX_SPEED_RUN_MIN)
		{
			e->v[i] = MAX_SPEED_RUN_MIN;
		}

		if (e->speed[i] > MAX_SPEED_RUN_MAX)
		{
			e->speed[i] = MAX_SPEED_RUN_MAX;
		}
		if (e->speed[i] < MAX_SPEED_RUN_MIN)
		{
			e->speed[i] = MAX_SPEED_RUN_MIN;
		}

		//在电机低速时，停止转动，并禁用平衡补偿，保护措施
		if (e->v[i] < 10)
		{
			e->speed[i] = 0;
			//在电机停转时，做陀螺仪补偿
			engine_set_dxy();
		}
	}
}

//取绝对值
float engine_abs(float value)
{
	if (value < 0)
	{
		return -value;
	}

	return value;
}

//XY轴的欧拉角PID反馈控制
float engine_pid(float et, float et_1, float et_2)
{
	//增量式PID反馈控制
	return params.kp * (et - et_1) + (params.ki * et) + params.kd * (et - 2 * et_1 + et_2);
}

//Z轴的欧拉角PID反馈控制，参数与XY轴的PID不一样
float engine_pid_z(float et, float et_1, float et_2)
{
	//增量式PID反馈控制
	return params.kp_z * (et - et_1) + (params.ki_z * et) + params.kd_z * (et - 2 * et_1 + et_2);
}

//对旋转角速度做PID反馈控制
float engine_pid_v(float et, float et_1, float et_2)
{
	//增量式PID反馈控制
	return params.kp_v * (et - et_1) + (params.ki_v * et) + params.kd_v * (et - 2 * et_1 + et_2);
}

//引擎重置
void engine_reset(s_engine *e)
{
	//陀螺仪修正补偿XYZ轴
	e->dx = 0;
	e->dy = 0;
	e->dz = 0;
	//陀螺仪中心校准补偿XY轴
	e->cx = 15.0;
	e->cy = 12.0;
	//XYZ欧拉角
	e->x = 0;
	e->y = 0;
	e->z = 0;
	//飞行移动倾斜角
	e->mx = 0;
	e->my = 0;
	//XYZ轴旋转角速度
	e->gx = 0;
	e->gy = 0;
	e->gz = 0;

	//重置速度
	for (int i = 0; i < 4; i++)
	{
		//速度置为0
		e->v[i] = 0;
		//平衡补偿置为0
		e->v_devi[i] = 0;
		//电机实际速度置为0
		e->speed[i] = 0;
	}
}

//陀螺仪补偿
void engine_set_dxy()
{
	s_engine *e = &engine;
	//补偿陀螺仪读数，将3个轴的欧拉角都补偿为0
	e->dx = -e->x;
	e->dy = -e->y;
	e->dz = -e->z;
}

//取得MPU6050陀螺仪读数
void engine_mpu()
{
	s_engine *e = &engine;
	while (1)
	{
		//读取并设置陀螺仪读数到引擎
		mpu6050_value(&e->z, &e->x, &e->y, &e->gy, &e->gx, &e->gz);
	}
}

//读入摇控器“前/后”的PWM信号
void engine_fb_pwm(int fb)
{
	ctl_fb = fb;
	//由2000～1600信号修正为-32.0 ～ +32.0角度
	engine.mx = ((float) (fb - CTL_FB)) / 50.0 * 4.0;
}

//读入摇控器“左/右”的PWM信号
void engine_lr_pwm(int lr)
{
	ctl_lr = lr;
	//由2000～1600信号修正为-32.0 ～ +32.0角度
	engine.my = ((float) (lr - CTL_LR)) / 50.0 * 4.0;
}

//读入摇控器“油门”的PWM信号
void engine_pw_pwm(int pw)
{
	ctl_pw = pw;
	//读入速度
	float v = (float) (pw - CTL_PW);
	//设置引擎的速度
	for (int i = 0; i < 4; i++)
	{
		engine.v[i] = v;
	}
}

//异常处理
void engine_exception()
{
	//引擎清理
	engine_clear();
	sleep(1);
	//退出
	exit(0);
}

//引擎清理
void engine_clear()
{
	//重置引擎
	engine_reset(&engine);
	//清理驱动
	driver_clear();
	//清理调参
	params_clear();
}
