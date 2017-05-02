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

//参数文件
#define QUAD_PMS_FILE "params/quadcopter.pms"
//高参幅度速度
#define STEP_V			(10)

int __init(s_engine *engine, s_params *params);

int __destory(s_engine *e, s_params *p);

int __status();

//保存参数
void params_save();

//载入参数
void params_load();

//重置参数
void params_reset();

//键盘接收按键
void params_input();

#endif /* _INCLUDE_PARAMSCTL_H_ */
