/*
 * hcsr04.c
 *
 *  Created on: Sep 9, 2016
 *
 *  四轴飞行控制器  Copyright (C) 2016  李德强
 */

#include <hcsr04.h>

int st = 0;
pthread_t pthd;
s_engine *e = NULL;
s_params *p = NULL;

s_dis dis;

//信号噪声
float h_est_devi = 0.01;
float h_measure_devi = 0.05;
//卡尔曼滤波
float h_est = 0.0, h_devi = 0.0;

//初始化陀螺仪
int __init(s_engine *engine, s_params *params)
{
	e = engine;
	p = params;
	st = 1;

	//设置引脚为输出引脚
	pinMode(PORT_CS_TRIG, OUTPUT);
	pinMode(PORT_CS_ECHO, INPUT);

	//初始化HC-SR04
	digitalWrite(PORT_CS_TRIG, HIGH);
	//持续15微秒
	usleep(15);
	digitalWrite(PORT_CS_TRIG, LOW);

	wiringPiISR(PORT_CS_ECHO, INT_EDGE_BOTH, &distance);

	return 0;
}

int __destory(s_engine *e, s_params *p)
{
	st = 0;

	return 0;
}

int __status()
{
	return st;
}

void distance()
{
	//读取电平信号
	int value = digitalRead(PORT_CS_ECHO);
	//如果是高电平
	if (value)
	{
		//计时开始
		gettimeofday(&dis.timer_start, NULL);
		return;
	}

	//如果是低电平,计时结束
	gettimeofday(&dis.timer_end, NULL);
	//计算高电平时长
	long timer = (dis.timer_end.tv_sec - dis.timer_start.tv_sec) * 1000000 + (dis.timer_end.tv_usec - dis.timer_start.tv_usec);

	float height = 0;
	//如果超过30ms超出测距范围，有效范围4.0米
	if (timer > 30 * 1000)
	{
		//默认最远距离为5米
		height = 5.0;
	}
	else
	{
		//声音在空气中传播速度3.40m/s，距离往返除以2。
		height = timer * 340.0 / 2.0;
	}

	//对高度卡尔曼滤波
	h_est = kalman_filter(h_est, h_est_devi, height, h_measure_devi, &h_devi);
	//设定引擎中调试值
	e->height = h_est;
}

/***
 * est预估值
 * est_devi预估偏差
 * measure测量读数
 * measure_devi测量噪声
 * devi上一次最优偏差
 */
float kalman_filter(float est, float est_devi, float measure, float measure_devi, float *devi)
{
	//预估高斯噪声的偏差
	float q = sqrt((*devi) * (*devi) + est_devi * est_devi);
	//卡尔曼增益
	float kg = q * q / (q * q + measure_devi * measure_devi);
	//滤波结果
	float val = est + kg * (measure - est);
	//最优偏差
	*devi = sqrt((1.0 - kg) * q * q);

	return val;
}

