/*
 * engine.h
 *
 *  Created on: Apr 12, 2016
 *
 *  四轴飞行控制器  Copyright (C) 2016  李德强
 */

#ifndef SRC_ENGINE_H_
#define SRC_ENGINE_H_

#include <dlmod.h>
#include <typedef.h>

//启动引擎
void engine_start(int argc, char* argv[]);

//启动飞行模式
void engine_start_fly();

//引擎核心算法平衡算法
void engine_fly();

// XY轴的欧拉角PID反馈控制
float engine_pid(float et, float dg, float* sum);

// XY轴的角速度PID反馈控制
float engine_v_pid(float et, float dg, float* sum);

//速度限幅
void engine_limit(float* v);

//卡尔曼滤波
float engine_kalman_filter(float est, float est_devi, float measure, float measure_devi, float* devi);

//引擎重置
void engine_reset(s_engine* e);

//陀螺仪补偿
void engine_set_dxy();

//绝对值
float engine_abs(float v);

//电机锁定解锁处理
void engine_lock();

//系统信号处理
void engine_handler();

#endif /* SRC_ENGINE_H_ */
