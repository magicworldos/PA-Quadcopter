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
void engine_start(s32 argc, char* argv[]);

//启动飞行模式
void engine_start_fly();

//引擎核心算法平衡算法
void engine_fly();

//内环PID输入角度输出角速度
f32 engine_outside_pid(f32 et, f32 et2, float* sum);

//内环PID输入角速度输出PWM比例（千分比）
f32 engine_inside_pid(f32 et, f32 et2, float* sum);

//外环角速度限幅
void engine_limit_palstance(float* palstance);

//内环PWM限幅
void engine_limit_pwm(float* v);

//卡尔曼滤波
f32 engine_kalman_filter(f32 est, f32 est_devi, f32 measure, f32 measure_devi, float* devi);

//引擎重置
void engine_reset(s_engine* e);

//陀螺仪补偿
void engine_set_dxy();

//绝对值
f32 engine_abs(f32 v);

//电机锁定解锁处理
void engine_lock();

//系统信号处理
void engine_handler();

#endif /* SRC_ENGINE_H_ */
