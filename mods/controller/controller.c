/*
 * controller.c
 *
 *  Created on: Sep 28, 2016
 *      Author: lidq
 */

#include <controller.h>

int st = 0;
pthread_t pthd;
s_engine *e = NULL;
s_params *p = NULL;

s_ctl_pwm ctl_pwm_fb;
s_ctl_pwm ctl_pwm_lr;
s_ctl_pwm ctl_pwm_pw;

//摇控器pwm信号噪声
float ctl_est_devi = 1;
float ctl_measure_devi = 5;
//前后卡尔曼滤波
float fb_est = 0.0, fb_devi = 0.0, fb_est1 = 0.0, fb_devi1 = 0.0, fb_est2 = 0.0, fb_devi2 = 0.0;
//左右卡尔曼滤波
float lr_est = 0.0, lr_devi = 0.0, lr_est1 = 0.0, lr_devi1 = 0.0, lr_est2 = 0.0, lr_devi2 = 0.0;
//油门卡尔曼滤波
float pw_est = 0.0, pw_devi = 0.0, pw_est1 = 0.0, pw_devi1 = 0.0, pw_est2 = 0.0, pw_devi2 = 0.0;

int __init(s_engine *engine, s_params *params)
{
	e = engine;
	p = params;
	st = 1;

	//设置摇控器3个通道到GPIO为输入引脚
	pinMode(GPIO_FB, INPUT);
	pinMode(GPIO_LR, INPUT);
	pinMode(GPIO_PW, INPUT);

#ifndef __PC_TEST__
	wiringPiISR(GPIO_FB, INT_EDGE_BOTH, &controller_ctl_pwm_fb);
	wiringPiISR(GPIO_LR, INT_EDGE_BOTH, &controller_ctl_pwm_lr);
	wiringPiISR(GPIO_PW, INT_EDGE_BOTH, &controller_ctl_pwm_pw);
#endif

	printf("[ OK ] Controller Init.\n");

	return 0;
}

int __destory(s_engine *e, s_params *p)
{
	st = 0;

	return 0;
}

int __status()
{
	return st;
}

void controller_ctl_pwm(int gpio_port, s_ctl_pwm *ctl_pwm)
{
	//读取电平信号
	int value = digitalRead(gpio_port);
	//如果是高电平
	if (value)
	{
		//计时开始
		gettimeofday(&ctl_pwm->timer_start, NULL);
		return;
	}

	//计时结束
	gettimeofday(&ctl_pwm->timer_end, NULL);
	//计算高电平时长
	long timer = (ctl_pwm->timer_end.tv_sec - ctl_pwm->timer_start.tv_sec) * 1000000 + (ctl_pwm->timer_end.tv_usec - ctl_pwm->timer_start.tv_usec);
	//如果超过低于1.0ms或大于2ms则视为无效
	if (timer < CTL_PWM_MIN || timer > CTL_PWM_MAX)
	{
		return;
	}

	//向引擎发送“前后”数值
	if (gpio_port == GPIO_FB)
	{
		//对方向舵前后通道做卡尔曼滤波
		fb_est = engine_kalman_filter(fb_est, ctl_est_devi, timer, ctl_measure_devi, &fb_devi);
		fb_est1 = engine_kalman_filter(fb_est1, ctl_est_devi, fb_est, ctl_measure_devi, &fb_devi1);
		fb_est2 = engine_kalman_filter(fb_est2, ctl_est_devi, fb_est1, ctl_measure_devi, &fb_devi2);
		controller_fb_pwm(fb_est2);
	}
	//向引擎发送“左右”数值
	else if (gpio_port == GPIO_LR)
	{
		//对方向舵左右通道做卡尔曼滤波
		lr_est = engine_kalman_filter(lr_est, ctl_est_devi, timer, ctl_measure_devi, &lr_devi);
		lr_est1 = engine_kalman_filter(lr_est1, ctl_est_devi, lr_est, ctl_measure_devi, &lr_devi1);
		lr_est2 = engine_kalman_filter(lr_est2, ctl_est_devi, lr_est1, ctl_measure_devi, &lr_devi2);
		controller_lr_pwm(lr_est2);
	}
	//向引擎发送“油门”数值
	else if (gpio_port == GPIO_PW)
	{
		//对油门通道做卡尔曼滤波
		pw_est = engine_kalman_filter(pw_est, ctl_est_devi, timer, ctl_measure_devi, &pw_devi);
		pw_est1 = engine_kalman_filter(pw_est1, ctl_est_devi, pw_est, ctl_measure_devi, &pw_devi1);
		pw_est2 = engine_kalman_filter(pw_est2, ctl_est_devi, pw_est1, ctl_measure_devi, &pw_devi2);
		controller_pw_pwm(pw_est2);
	}

}

//读取摇控器接收机的PWM信号“前后”
void controller_ctl_pwm_fb()
{
	controller_ctl_pwm(GPIO_FB, &ctl_pwm_fb);
}

//读取摇控器接收机的PWM信号“左右”
void controller_ctl_pwm_lr()
{
	controller_ctl_pwm(GPIO_LR, &ctl_pwm_lr);
}

//读取摇控器接收机的PWM信号“油门”
void controller_ctl_pwm_pw()
{
	controller_ctl_pwm(GPIO_PW, &ctl_pwm_pw);
}

//读入摇控器“前/后”的PWM信号
void controller_fb_pwm(int fb)
{
	if (fb < CTL_PWM_MIN || fb > CTL_PWM_MAX)
	{
		return;
	}
	if (p->ctl_fb_zero < CTL_PWM_MIN || p->ctl_fb_zero > CTL_PWM_MAX)
	{
		p->ctl_fb_zero = 1400;
	}
	e->ctl_fb = fb;
	//由2000～1600信号修正为-32.0 ～ +32.0角度
	//采用二次曲线来对倾斜角做过滤，使角度变化更平滑
	e->ctlmx = controller_parabola(((float) (fb - p->ctl_fb_zero)) / 50.0 * 4.0);
}

//读入摇控器“左/右”的PWM信号
void controller_lr_pwm(int lr)
{
	if (lr < CTL_PWM_MIN || lr > CTL_PWM_MAX)
	{
		return;
	}
	if (p->ctl_lr_zero < CTL_PWM_MIN || p->ctl_lr_zero > CTL_PWM_MAX)
	{
		p->ctl_lr_zero = 1400;
	}
	e->ctl_lr = lr;
	//由2000～1600信号修正为-32.0 ～ +32.0角度
	//采用二次曲线来对倾斜角做过滤，使角度变化更平滑
	e->ctlmy = controller_parabola(((float) (lr - p->ctl_lr_zero)) / 50.0 * 4.0);

	//如果是最左或最右
	if (abs(lr - p->ctl_lr_zero) > 160)
	{
		//如果是最左
		if (lr - p->ctl_lr_zero < 0)
		{
			e->lock_status |= (0x1 << 2);
			e->lock_status &= ~(0x1 << 1);
			return;
		}
		else
		{
			e->lock_status |= (0x1 << 1);
			e->lock_status &= ~(0x1 << 2);
			return;
		}
	}
	e->lock_status &= ~(0x1 << 1);
	e->lock_status &= ~(0x1 << 2);
}

//读入摇控器“油门”的PWM信号
void controller_pw_pwm(int pw)
{
	if (pw < CTL_PWM_MIN || pw > CTL_PWM_MAX)
	{
		return;
	}
	if (p->ctl_pw_zero < CTL_PWM_MIN || p->ctl_pw_zero > CTL_PWM_MAX)
	{
		p->ctl_pw_zero = 1000;
	}
	e->ctl_pw = pw;
	//读入速度
	float v = (float) (pw - p->ctl_pw_zero);
	//设置引擎的速度
	e->v = v;

	//如果是最低油门
	if (abs(pw - p->ctl_pw_zero) < 50)
	{
		e->lock_status |= 0x1;
	}
	else
	{
		e->lock_status &= (~0x1);
	}
}

//取绝对值
float controller_abs(float x)
{
	if (x < 0)
	{
		return -x;
	}
	return x;
}

//二次曲线函数
float controller_parabola(float x)
{
	float flag = x / controller_abs(x);
	return flag * (1.0 / 22.0) * (x * x);
}
