/*
 * paramsctl.h
 *
 *  Created on: May 24, 2016
 *
 * 四轴飞行控制器  Copyright (C) 2016  李德强
 */

#ifndef _INCLUDE_PARAMSCTL_H_
#define _INCLUDE_PARAMSCTL_H_

#include <getch.h>
#include <typedef.h>

//参数文件
#define QUAD_PMS_FILE "params/quadcopter.pms"
//高参幅度速度
#define STEP_V (10)

s32 __init(s_engine* engine, s_params* params);

s32 __destory(s_engine* e, s_params* p);

s32 __status();

//保存参数
void params_save();

//载入参数
void params_load();

//重置参数
void params_reset();

//键盘接收按键
void params_input();

//设置显示开关
void params_set_onoff(s32 bit);

#endif /* _INCLUDE_PARAMSCTL_H_ */
