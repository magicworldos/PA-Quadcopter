/*
 * typedef.h
 *
 *  Created on: Mar 4, 2016
 *
 *  四轴飞行控制器  Copyright (C) 2016  李德强
 */

#ifndef _INCLUDE_TYPEDEF_H_
#define _INCLUDE_TYPEDEF_H_

#include <dirent.h>
#include <dlfcn.h>
#include <math.h>
#include <pthread.h>
#include <semaphore.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#include <wiringPi.h>
#include <wiringSerial.h>

typedef char s8;
typedef unsigned char u8;

typedef signed short s16;
typedef unsigned short u16;

typedef signed int s32;
typedef unsigned int u32;

typedef signed long long s64;
typedef unsigned long long u64;

//保护最低速度
#define PROCTED_SPEED (100)
//电机最大速度
#define MAX_SPEED_RUN_MAX (1000)
//电机最小速度
#define MAX_SPEED_RUN_MIN (0)
//原定计算频率1000Hz，但由于MPU6050的输出为100hz只好降低到100hz
// 10ms 100Hz
#define ENG_TIMER (10)

#define MODE_MANUAL (0)
#define MODE_AUTO (1)

#define MAX_ACC	(20.0)

#define PV	(23.5)

//引擎结构
typedef struct
{
	//电机锁定状态，默认为锁定
	int lock;
	//实际欧拉角
	float tx;
	float ty;
	float tz;
	// XYZ欧拉角
	float x;
	float y;
	float z;
	//修正补偿
	float dx;
	float dy;
	float dz;
	float dax;
	float day;
	//摇控器移动倾角
	float ctlmx;
	float ctlmy;
	// XYZ轴旋转角速度
	float gx;
	float gy;
	float gz;
	//修正补偿
	float dgx;
	float dgy;
	float dgz;
	// XYZ加速度
	float ax;
	float ay;
	float az;
	//引擎速度
	float v;
	// XYZ欧拉角补偿
	float x_devi;
	float y_devi;
	float z_devi;
	// XYZ角速度补偿
	float xv_devi;
	float yv_devi;
	float zv_devi;
	//欧拉角累加值
	float x_sum;
	float y_sum;
	float z_sum;
	//欧拉角累加值
	float x_v_sum;
	float y_v_sum;
	float z_v_sum;
	//高度累加值
	float h_sum;

	//其它参数
	//显示摇控器读数
	int ctl_fb;
	int ctl_lr;
	int ctl_pw;
	int ctl_md;
	int ctl_ud;
	int ctl_di;

	//测量高度
	float height;
	//期望高度
	float height_target;
	//高度补偿
	float h_devi;

	//最低油门,最左，最右
	u32 lock_status;
	u32 mode;
} s_engine;

//参数
typedef struct
{
	// XY轴欧拉角PID参数
	float kp;
	float ki;
	float kd;
	// XYZ角速度PID参数
	float v_kp;
	float v_ki;
	float v_kd;
	//高度PID参数
	float h_kp;
	float h_ki;
	float h_kd;
	//中心校准补偿
	float cx;
	float cy;
	//摇控器4通道起始读数
	int ctl_fb_zero;
	int ctl_lr_zero;
	int ctl_pw_zero;
	int ctl_md_zero;
	int ctl_ud_zero;
	int ctl_di_zero;

	int ctl_type;
} s_params;

#endif /* INCLUDE_TYPEDEF_H_ */
