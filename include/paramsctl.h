/*
 * paramsctl.h
 *
 *  Created on: May 24, 2016
 *
 * 四轴飞行控制器  Copyright (C) 2016  李德强
 */

#ifndef _INCLUDE_PARAMSCTL_H_
#define _INCLUDE_PARAMSCTL_H_

#include <typedef.h>
#include <getch.h>
#include <engine.h>

//参数文件
#define QUAD_PMS_FILE "params/quadcopter.pms"
//调参幅度
#define CTL_STEP		(0.1)
//高参幅度速度
#define STEP_V			(10)

//参数
typedef struct
{
	//XY轴欧拉角PID参数
	float kp;
	float ki;
	float kd;
	//旋转角速度PID参数
	float kp_v;
	float ki_v;
	float kd_v;
//	//Z轴欧拉角PID参数
//	float kp_z;
//	float ki_z;
//	float kd_z;
//	//Z旋转角速度PID参数
//	float kp_zv;
//	float ki_zv;
//	float kd_zv;
	//XY轴加速度PID参数
	float kp_a;
	float ki_a;
	float kd_a;
	//中心校准补偿
	float cx;
	float cy;
	//摇控器3通道起始读数
	int ctl_fb_zero;
	int ctl_lr_zero;
	int ctl_pw_zero;
} s_params;

//保存参数
void params_save();

//载入参数
void params_load();

//重置参数
void params_reset();

//键盘接收按键
void params_input();

#endif /* _INCLUDE_PARAMSCTL_H_ */
