/*
 * driver.c
 *
 *  Created on: Apr 16, 2016
 *
 *  四轴飞行控制器  Copyright (C) 2016  李德强
 */

#include <motor.h>

//电机实际速度
int speed[MOTOR_COUNT];
int ports[MOTOR_COUNT] =
{ PORT_MOTOR0, PORT_MOTOR1, PORT_MOTOR2, PORT_MOTOR3 };

pthread_t pthddr;

int st[MOTOR_COUNT];
int r = 0;
pthread_t pthd;
s_engine *e = NULL;
s_params *p = NULL;

//初始化时电调可能需要行程校准，通常是3秒最大油门，再3秒最小油门，但不是必要的，可以不做
int __init(s_engine *engine, s_params *params)
{
	e = engine;
	p = params;

	r = 1;
	for (int i = 0; i < MOTOR_COUNT; i++)
	{
		//状态
		st[i] = 1;
		//电机初始速度为0
		speed[i] = 0;

		//设置电机GPIO为输出引脚
		pinMode(ports[i], OUTPUT);

#ifndef __PC_TEST__

		//启动电机信号输出线程
		pthread_create(&pthddr, (const pthread_attr_t*) NULL, (void* (*)(void*)) &motor_run, (void *) (u64) i);
#endif

	}

	return 0;
}

int __destory(s_engine *e, s_params *p)
{
	r = 0;
	usleep(10 * 1000);

	//电机停止
	for (int i = 0; i < MOTOR_COUNT; i++)
	{
		speed[i] = 0;
	}

	usleep(10 * 1000);

	for (int i = 0; i < MOTOR_COUNT; i++)
	{
		//状态
		st[i] = 0;
	}

	return 0;
}

int __status()
{
	for (int i = 0; i < MOTOR_COUNT; i++)
	{
		if (st[i])
		{
			return st[i];
		}
	}
	return 0;
}

//将电机速度转为PWM信号
void motor_set_pwm(int speed, motor_pwm *pwm)
{
	//高电平时长
	pwm->time_m = TIME_DEP + speed;
	//低电平时长
	pwm->time_w = TIME_PWM_WIDTH - pwm->time_m;
}

//对电机发送PWM信号
void motor_run_pwm(int motor, motor_pwm *pwm)
{
	//高电平
	digitalWrite(ports[motor], HIGH);
	usleep(pwm->time_m);
	//低电平
	digitalWrite(ports[motor], LOW);
	usleep(pwm->time_w);
}

//向电机发送PWM信号
void motor_run(void *args)
{
	u64 motor = (u64) args;
	motor_pwm pwm;
	while (r)
	{
		//校验电机有效范围
		speed[motor] = motor_rechk_speed(e->speed[motor]);

		//将电机速度转为PWM信号
		motor_set_pwm(speed[motor], &pwm);

		//对电机发送PWM信号
		motor_run_pwm(motor, &pwm);
	}

	st[motor] = 0;
}

//校验电机转数范围
int motor_rechk_speed(int speed)
{
	if (speed > MAX_SPEED_RUN_MAX)
	{
		speed = MAX_SPEED_RUN_MAX;
	}
	if (speed < MAX_SPEED_RUN_MIN)
	{
		speed = MAX_SPEED_RUN_MIN;
	}

	//在电机锁定时，停止转动，并禁用平衡补偿，保护措施
	if (e->lock || e->v < PROCTED_SPEED)
	{
		speed = 0;
	}

	return speed;
}
