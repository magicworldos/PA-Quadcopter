/*
 * driver.c
 *
 *  Created on: Apr 16, 2016
 *
 *  四轴飞行控制器  Copyright (C) 2016  李德强
 */

#include <motor.h>

//电机实际速度
s32 speed[MOTOR_COUNT];
s32 ports[MOTOR_COUNT] = { PORT_MOTOR0, PORT_MOTOR1, PORT_MOTOR2, PORT_MOTOR3 };

pthread_t pthddr;

s32 sts = 0;
s32 st[MOTOR_COUNT];
s32 r = 0;
pthread_t pthd;
s_engine* e = NULL;
s_params* p = NULL;

//初始化时电调可能需要行程校准，通常是3秒最大油门，再3秒最小油门，但不是必要的，可以不做
s32 __init(s_engine* engine, s_params* params)
{
	e = engine;
	p = params;

	r = 1;
	sts = 1;
	for (s32 i = 0; i < MOTOR_COUNT; i++)
	{
		//状态
		st[i] = 1;
		//电机初始速度为0
		speed[i] = 0;

		//设置电机GPIO为输出引脚
		pinMode(ports[i], OUTPUT);
		//初始值为低电平,保护措施
		digitalWrite(ports[i], LOW);

		//启动速度平衡补偿
		pthread_create(&pthddr, (const pthread_attr_t*) NULL, (void* (*)(void*)) &motor_balance_compensation, NULL);

		//启动电机信号输出线程
		pthread_create(&pthddr, (const pthread_attr_t*) NULL, (void* (*)(void*)) &motor_run, (void*) (long) i);
	}

	printf("[ OK ] Motor Init.\n");

	return 0;
}

s32 __destory(s_engine* e, s_params* p)
{
	r = 0;
	usleep(10 * 1000);

	//电机停止
	for (s32 i = 0; i < MOTOR_COUNT; i++)
	{
		speed[i] = 0;
	}

	usleep(10 * 1000);

	for (s32 i = 0; i < MOTOR_COUNT; i++)
	{
		//状态
		st[i] = 0;
	}

	return 0;
}

s32 __status()
{
	for (s32 i = 0; i < MOTOR_COUNT; i++)
	{
		if (st[i])
		{
			return st[i];
		}
	}

	return sts;
}

//将电机速度转为PWM信号
void motor_set_pwm(s32 speed, motor_pwm* pwm)
{
	//高电平时长
	pwm->time_m = TIME_DEP + speed;
	//低电平时长
	pwm->time_w = TIME_PWM_WIDTH - pwm->time_m;
}

//对电机发送PWM信号
void motor_run_pwm(s32 motor, motor_pwm* pwm)
{
	//高电平
	digitalWrite(ports[motor], HIGH);
	usleep(pwm->time_m);
	//低电平
	digitalWrite(ports[motor], LOW);
	usleep(pwm->time_w);
}

//向电机发送PWM信号
void motor_run(void* args)
{
	long motor = (long) args;
	motor_pwm pwm;
	while (r)
	{
		//将电机速度转为PWM信号
		motor_set_pwm(speed[motor], &pwm);
		//对电机发送PWM信号
		motor_run_pwm(motor, &pwm);

		//pwm范围是0~1024
		//pwmWrite(ports[motor], 512 + (speed[motor] / 2));
	}

	st[motor] = 0;
}

//速度平衡补偿
void motor_balance_compensation()
{
	while (r)
	{
		e->v = e->v > MAX_SPEED_RUN_MAX ? MAX_SPEED_RUN_MAX : e->v;
		e->v = e->v < MAX_SPEED_RUN_MIN ? MAX_SPEED_RUN_MIN : e->v;

		//在电机锁定时，停止转动，并禁用平衡补偿，保护措施
		if (e->lock || e->v < PROCTED_SPEED)
		{
			for (s32 i = 0; i < MOTOR_COUNT; i++)
			{
				speed[i] = 0;
			}
			usleep(ENG_TIMER * 1000);
			continue;
		}

#ifdef _FLY_MODE_I_
		//四轴平衡补偿I型
		speed[0] = (int)e->v + e->xv_devi - e->zv_devi;
		speed[1] = (int)e->v + e->yv_devi + e->zv_devi;
		speed[2] = (int)e->v - e->xv_devi - e->zv_devi;
		speed[3] = (int)e->v - e->yv_devi + e->zv_devi;
#endif

#ifdef _FLY_MODE_X_
		//四轴平衡补偿X型
		speed[0] = (int) e->v + (e->xv_devi / 2) - (e->yv_devi / 2) - e->zv_devi;
		speed[1] = (int) e->v + (e->xv_devi / 2) + (e->yv_devi / 2) + e->zv_devi;
		speed[2] = (int) e->v - (e->xv_devi / 2) + (e->yv_devi / 2) - e->zv_devi;
		speed[3] = (int) e->v - (e->xv_devi / 2) - (e->yv_devi / 2) + e->zv_devi;
#endif

		//对电机限幅
		for (s32 i = 0; i < MOTOR_COUNT; i++)
		{
			speed[i] = speed[i] > MAX_SPEED_RUN_MAX ? MAX_SPEED_RUN_MAX : speed[i];
			speed[i] = speed[i] < MAX_SPEED_RUN_MIN ? MAX_SPEED_RUN_MIN : speed[i];
		}

		usleep(ENG_TIMER * 1000);
	}

	sts = 0;
}
