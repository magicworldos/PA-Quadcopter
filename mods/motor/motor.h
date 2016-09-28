/*
 * driver.h
 *
 *  Created on: Apr 16, 2016
 *
 *  四轴飞行控制器  Copyright (C) 2016  李德强
 */

#ifndef _INCLUDE_MOTOR_H_
#define _INCLUDE_MOTOR_H_

#include <typedef.h>

#define MOTOR_COUNT			(4)

//4个电机的GPIO引脚
#define PORT_MOTOR0	  		(27)
#define PORT_MOTOR1	  		(28)
#define PORT_MOTOR2	  		(26)
#define PORT_MOTOR3	  		(29)

//PWM信号时长
#define TIME_PWM_WIDTH		(2000)
//电调起始时长
#define TIME_DEP			(1000)

//PWM信号结构
typedef struct
{
	//高电平时长
	int time_m;
	//低电平时长
	int time_w;
} motor_pwm;

int __init(s_engine *e, s_params *p);

int __destory(s_engine *e, s_params *p);

int __status();

void motor_setup();

//清理驱动
void motor_clear();

//将电机速度转为PWM信号
void motor_set_pwm(int speed, motor_pwm *pwm);

//对电机0发送PWM信号
void motor_run_pwm(int motor, motor_pwm *pwm);

//向电机0发送PWM信号
void motor_run(void *args);

//校验电机转数范围
int motor_rechk_speed(int speed);

#endif /* INCLUDE_DRIVER_H_ */
