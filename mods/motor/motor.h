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

//电机数量
#define MOTOR_COUNT (4)

// PWM信号时长
#define TIME_PWM_WIDTH (2000)
//电调起始时长
#define TIME_DEP (1000)

// PWM信号结构
typedef struct
{
	//高电平时长
	s32 time_m;
	//低电平时长
	s32 time_w;
} motor_pwm;

s32 __init(s_engine* e, s_params* p);

s32 __destory(s_engine* e, s_params* p);

s32 __status();

void motor_setup();

//清理驱动
void motor_clear();

//将电机速度转为PWM信号
void motor_set_pwm(s32 speed, motor_pwm* pwm);

//对电机0发送PWM信号
void motor_run_pwm(s32 motor, motor_pwm* pwm);

//向电机0发送PWM信号
void motor_run(void* args);

//速度平衡补偿
void motor_balance_compensation();

#endif /* INCLUDE_DRIVER_H_ */
