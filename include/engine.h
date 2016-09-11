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
#include <if_mpu6050.h>
#define								__DISPLAY_MODE_MORE__

//原定计算频率1000Hz，但由于MPU6050的输出为100hz只好降低到100hz
//10ms 100Hz
#define ENG_TIMER					(10)
//最大角度
#define MAX_ANGLE					(90.0)
//电机最大速度
#define MAX_SPEED_RUN_MAX			(1000)
//电机最小速度
#define MAX_SPEED_RUN_MIN			(0)

//引擎结构
typedef struct
{
	//电机锁定状态，默认为锁定
	int lock;
	//XYZ欧拉角
	float x;
	float y;
	float z;
	//修正补偿
	float dx;
	float dy;
	float dz;
	//飞行移动倾斜角
	float mx;
	float my;
	//XYZ轴旋转角速度
	float gx;
	float gy;
	float gz;
	//补偿XYZ轴旋转角速度
	float dgx;
	float dgy;
	float dgz;
	//XYZ轴加速度
	float ax;
	float ay;
	float az;
	//修正加速度补偿
	float dax;
	float day;
	float daz;
	//引擎速度
	float v;
	//速度补偿
	float v_devi[4];
	//电机实际速度
	int speed[4];

} s_engine;

//启动引擎
void engine_start(int argc, char *argv[]);

//引擎核心算法平衡算法
void engine_fly();

//MPU6050
void engine_mpu();

//引擎运转
void engine_move(s_engine *e);

//校验电机转数范围
void engine_rechk_speed(s_engine *e);

//取绝对值
float engine_abs(float value);

//XY轴的欧拉角PID反馈控制
float engine_pid(float et, float et_1, float et_2);

//Z轴的欧拉角PID反馈控制，参数与XY轴的PID不一样
float engine_pid_z(float et, float et_1, float et_2);

//对旋转角速度做PID反馈控制
float engine_pid_v(float et, float et_1, float et_2);

//对XY轴加速度做PID反馈控制
float engine_pid_a(float et, float et_1, float et_2);

//引擎重置
void engine_reset(s_engine *e);

//陀螺仪补偿
void engine_set_dxy();

//读入摇控器“前/后”的PWM信号
void engine_fb_pwm(int fb);

//读入摇控器“左/右”的PWM信号
void engine_lr_pwm(int lr);

//读入摇控器“油门”的PWM信号
void engine_pw_pwm(int pw);

//电机锁定解锁处理
void engine_lock();

//异常处理
void engine_exception();

//引擎清理
void engine_clear();

#endif /* SRC_ENGINE_H_ */
