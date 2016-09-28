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

int __init(s_engine *engine, s_params *params)
{
	e = engine;
	p = params;
	st = 1;

	//设置摇控器3个通道到GPIO为输入引脚
	pinMode(GPIO_FB, INPUT);
	pinMode(GPIO_LR, INPUT);
	pinMode(GPIO_PW, INPUT);

	//启动摇控器接收机信号输入线程
	ctl_pwm_fb.timer_sum = 0;
	ctl_pwm_fb.timer_max = 0;
	ctl_pwm_fb.timer_min = 9999;
	ctl_pwm_fb.timer_n = 0;

	ctl_pwm_lr.timer_sum = 0;
	ctl_pwm_lr.timer_max = 0;
	ctl_pwm_lr.timer_min = 9999;
	ctl_pwm_lr.timer_n = 0;

	ctl_pwm_pw.timer_sum = 0;
	ctl_pwm_pw.timer_max = 0;
	ctl_pwm_pw.timer_min = 9999;
	ctl_pwm_pw.timer_n = 0;

	wiringPiISR(GPIO_FB, INT_EDGE_BOTH, &controller_ctl_pwm_fb);
	wiringPiISR(GPIO_LR, INT_EDGE_BOTH, &controller_ctl_pwm_lr);
	wiringPiISR(GPIO_PW, INT_EDGE_BOTH, &controller_ctl_pwm_pw);

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

	ctl_pwm->timer_sum += timer;
	ctl_pwm->timer_max = timer > ctl_pwm->timer_max ? timer : ctl_pwm->timer_max;
	ctl_pwm->timer_min = timer < ctl_pwm->timer_min ? timer : ctl_pwm->timer_min;
	ctl_pwm->timer_n++;
	if (ctl_pwm->timer_n >= 5)
	{
		ctl_pwm->timer_avg = (ctl_pwm->timer_sum - ctl_pwm->timer_max - ctl_pwm->timer_min) / (ctl_pwm->timer_n - 2);

		//向引擎发送“前后”数值
		if (gpio_port == GPIO_FB)
		{
			controller_fb_pwm(ctl_pwm->timer_avg);
		}
		//向引擎发送“左右”数值
		else if (gpio_port == GPIO_LR)
		{
			controller_lr_pwm(ctl_pwm->timer_avg);
		}
		//向引擎发送“油门”数值
		else if (gpio_port == GPIO_PW)
		{
			controller_pw_pwm(ctl_pwm->timer_avg);
		}

		ctl_pwm->timer_sum = 0;
		ctl_pwm->timer_max = 0;
		ctl_pwm->timer_min = 9999;
		ctl_pwm->timer_n = 0;
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
