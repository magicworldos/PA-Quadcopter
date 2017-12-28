/*
 * controller.h
 *
 *  Created on: Sep 28, 2016
 *      Author: lidq
 */

#ifndef MODS_CONTROLLER_CONTROLLER_H_
#define MODS_CONTROLLER_CONTROLLER_H_

#include <typedef.h>

//摇控器接收机的4个通道读数范围
#define CTL_PWM_MIN (980)
#define CTL_PWM_MAX (2020)
//摇控器幅度通道比例范围,通过此通道来修改方向通道的数值比例
#define CTL_DI_MIN (1000)
#define CTL_DI_MAX (2000)
//最大倾斜角
#define MAX_ANGLE	MAX_PALSTANCE

typedef struct
{
	struct timeval timer_start;
	struct timeval timer_end;
} s_ctl_pwm;

s32 __init(s_engine* engine, s_params* params);

s32 __destory(s_engine* e, s_params* p);

s32 __status();

void controller_ctl_pwm(s32 gpio_port, s_ctl_pwm* ctl_pwm);

//读取摇控器接收机的PWM信号“前后”
void controller_ctl_pwm_fb();

//读取摇控器接收机的PWM信号“左右”
void controller_ctl_pwm_lr();

//读取摇控器接收机的PWM信号“油门”
void controller_ctl_pwm_pw();

//读取摇控器接收机的PWM第4通道
void controller_ctl_pwm_md();

//读取摇控器接收机的PWM第5通道
void controller_ctl_pwm_ud();

//读取摇控器接收机的PWM方向舵比例缩放通道
void controller_ctl_pwm_di();

//读入摇控器“前/后”的PWM信号
void controller_fb_pwm(s32 fb);

//读入摇控器“左/右”的PWM信号
void controller_lr_pwm(s32 lr);

//读入摇控器“油门”的PWM信号
void controller_pw_pwm(s32 pw);

//读入摇控器第4通道的PWM信号
void controller_md_pwm(s32 md);

//读入摇控器第5通道的PWM信号
void controller_ud_pwm(s32 md);

//读入摇控器方向舵比例缩放通道的PWM信号
void controller_di_pwm(s32 md);

//取绝对值
f32 controller_abs(f32 x);

//二次曲线函数
f32 controller_parabola(f32 x);

/***
 * est预估值
 * est_devi预估偏差
 * measure测量读数
 * measure_devi测量噪声
 * devi上一次最优偏差
 */
f32 controller_kalman_filter(f32 est, f32 est_devi, f32 measure, f32 measure_devi, float* devi);

#endif /* MODS_CONTROLLER_CONTROLLER_H_ */
