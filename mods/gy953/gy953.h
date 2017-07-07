/*
 * gy953.h
 *
 *  Created on: Sep 9, 2016
 *
 *  四轴飞行控制器  Copyright (C) 2016  李德强
 */

#ifndef INCLUDE_GY953_H_
#define INCLUDE_GY953_H_

#include <typedef.h>

s32 __init(s_engine* engine, s_params* params);

s32 __destory(s_engine* e, s_params* p);

s32 __status();

//初始化陀螺仪
s32 gy953_init(u32 init);

//循环读取陀螺仪读数
void gy953_value();

//读取一个类型的数值
s32 gy953_read(float* x, float* y, float* z);

//关闭陀螺仪
void gy953_close();

#endif /* INCLUDE_GY953_H_ */
