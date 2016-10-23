/*
 * engine.h
 *
 *  Created on: Apr 12, 2016
 *
 *  四轴飞行控制器  Copyright (C) 2016  李德强
 */

#ifndef SRC_ENGINE_H_
#define SRC_ENGINE_H_

#include <typedef.h>
#include <dlmod.h>

//渐进式方向增量
#define DIRECT_VALUE				(0.005)

//电调起始时长(仅在--ctl模式下用)
#define TEST_ZERO_MS				(1000)
//最大调试时长10秒(仅在--ctl模式下用)
#define TEST_MAX_MS					(10000)

//启动引擎
void engine_start(int argc, char *argv[]);

//引擎核心算法平衡算法
void engine_fly();

//XY轴的欧拉角PID反馈控制
float engine_pid(float et, float dg, float *sum);

//卡尔曼滤波
float engine_kalman_filter(float est, float est_devi, float measure, float measure_devi, float *devi);

//引擎重置
void engine_reset(s_engine *e);

//陀螺仪补偿
void engine_set_dxy();

//绝对值
float engine_abs(float v);

//电机锁定解锁处理
void engine_lock();

//电机调试
void engine_ent_run(int en_port, int en_speed, int en_msecs);

//系统信号处理
void engine_handler();

#endif /* SRC_ENGINE_H_ */
