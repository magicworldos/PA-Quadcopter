/*
 * typedef.h
 *
 *  Created on: Mar 4, 2016
 *
 *  四轴飞行控制器  Copyright (C) 2016  李德强
 */

#ifndef _INCLUDE_TYPEDEF_H_
#define _INCLUDE_TYPEDEF_H_

#include <defconfig.h>

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

typedef float f32;
typedef double f64;

typedef signed long long s64;
typedef unsigned long long u64;

//引擎结构
typedef struct
{
	//电机锁定状态，默认为锁定
	s32 lock;

	//实际欧拉角
	f32 tx;
	f32 ty;
	f32 tz;

	// XYZ欧拉角
	f32 x;
	f32 y;
	f32 z;

	//修正补偿
	f32 dx;
	f32 dy;
	f32 dz;
	f32 dax;
	f32 day;
	f32 daz;

	//摇控器移动倾角
	f32 ctlmx;
	f32 ctlmy;

	// XYZ轴旋转角速度
	f32 gx;
	f32 gy;
	f32 gz;

	//修正补偿
	f32 dgx;
	f32 dgy;
	f32 dgz;

	//实际角速度
	f32 tgx;
	f32 tgy;
	f32 tgz;

	// XYZ加速度
	f32 ax;
	f32 ay;
	f32 az;
	f32 axt;
	f32 ayt;
	f32 azt;
	f32 vx;
	f32 vy;
	f32 vz;

	//引擎速度
	f32 v;

	// XYZ角速度补偿
	f32 xv_devi;
	f32 yv_devi;
	f32 zv_devi;

	//其它参数
	//显示摇控器读数
	s32 ctl_fb;
	s32 ctl_lr;
	s32 ctl_pw;
	s32 ctl_md;
	s32 ctl_ud;
	s32 ctl_di;

	//最低油门,最左，最右
	u32 lock_status;
	u32 mode;
} s_engine;

//参数
typedef struct
{
	// XY轴欧拉角PID参数
	f32 kp;
	f32 ki;
	f32 kd;
	// XYZ角速度PID参数
	f32 v_kp;
	f32 v_ki;
	f32 v_kd;
	//垂直加速度PID参数
	f32 vz_kp;
	f32 vz_ki;
	f32 vz_kd;
	//摇控器4通道起始读数
	s32 ctl_fb_zero;
	s32 ctl_lr_zero;
	s32 ctl_pw_zero;
	s32 ctl_md_zero;
	s32 ctl_ud_zero;
	s32 ctl_di_zero;
	//PID调参类型
	u32 ctl_type;
	//显示内容
	u32 ctl_display;
} s_params;

#endif /* INCLUDE_TYPEDEF_H_ */
