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
s_ctl_pwm ctl_pwm_md;
s_ctl_pwm ctl_pwm_ud;
s_ctl_pwm ctl_pwm_di;

//摇控器pwm信号噪声
float ctl_est_devi = 1;
float ctl_measure_devi = 15;
//前后卡尔曼滤波
float fb_est = 0.0, fb_devi = 0.0;
//左右卡尔曼滤波
float lr_est = 0.0, lr_devi = 0.0;
//油门卡尔曼滤波
float pw_est = 0.0, pw_devi = 0.0;
////第4通道卡尔曼滤波
//float md_est = 0.0, md_devi = 0.0;
////第5通道卡尔曼滤波
//float ud_est = 0.0, ud_devi = 0.0;
//第6通道卡尔曼滤波
float di_est = 0.0, di_devi = 0.0;

int __init(s_engine *engine, s_params *params)
{
	e = engine;
	p = params;
	st = 1;

	//设置摇控器6个通道到GPIO为输入引脚
	pinMode(GPIO_FB, INPUT);
	pinMode(GPIO_LR, INPUT);
	pinMode(GPIO_PW, INPUT);
	pinMode(GPIO_MD, INPUT);
	pinMode(GPIO_UD, INPUT);
	pinMode(GPIO_DI, INPUT);
	//注册摇控器6个通道的引脚变化中断
	wiringPiISR(GPIO_FB, INT_EDGE_BOTH, &controller_ctl_pwm_fb);
	wiringPiISR(GPIO_LR, INT_EDGE_BOTH, &controller_ctl_pwm_lr);
	wiringPiISR(GPIO_PW, INT_EDGE_BOTH, &controller_ctl_pwm_pw);
	wiringPiISR(GPIO_MD, INT_EDGE_BOTH, &controller_ctl_pwm_md);
	wiringPiISR(GPIO_UD, INT_EDGE_BOTH, &controller_ctl_pwm_ud);
	wiringPiISR(GPIO_DI, INT_EDGE_BOTH, &controller_ctl_pwm_di);

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

	//如果是低电平,计时结束
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
		fb_est = controller_kalman_filter(fb_est, ctl_est_devi, timer, ctl_measure_devi, &fb_devi);
		controller_fb_pwm(fb_est);
		return;
	}
	//向引擎发送“左右”数值
	if (gpio_port == GPIO_LR)
	{
		//对方向舵左右通道做卡尔曼滤波
		lr_est = controller_kalman_filter(lr_est, ctl_est_devi, timer, ctl_measure_devi, &lr_devi);
		controller_lr_pwm(lr_est);
		return;
	}
	//向引擎发送“油门”数值
	if (gpio_port == GPIO_PW)
	{
		//对油门通道做卡尔曼滤波
		pw_est = controller_kalman_filter(pw_est, ctl_est_devi, timer, ctl_measure_devi, &pw_devi);
		controller_pw_pwm(pw_est);
		return;
	}
	//第4通道
	if (gpio_port == GPIO_MD)
	{
		controller_md_pwm(timer);
		return;
	}
	//第5通道
	if (gpio_port == GPIO_UD)
	{
		controller_ud_pwm(timer);
		return;
	}
	//第6通道
	if (gpio_port == GPIO_DI)
	{
		//对第6通道做卡尔曼滤波
		di_est = controller_kalman_filter(di_est, ctl_est_devi, timer, ctl_measure_devi, &di_devi);
		controller_di_pwm(di_est);
		return;
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

//读取摇控器接收机的PWM信号第4通道
void controller_ctl_pwm_md()
{
	controller_ctl_pwm(GPIO_MD, &ctl_pwm_md);
}

//读取摇控器接收机的PWM信号第5通道
void controller_ctl_pwm_ud()
{
	controller_ctl_pwm(GPIO_UD, &ctl_pwm_ud);
}

//读取摇控器接收机的PWM信号第6通道
void controller_ctl_pwm_di()
{
	controller_ctl_pwm(GPIO_DI, &ctl_pwm_di);
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
		p->ctl_fb_zero = 1500;
	}
	e->ctl_fb = fb;
	//由2000～1600信号修正为-32.0 ～ +32.0角度
	//采用二次曲线来对倾斜角做过滤，使角度变化更平滑
	e->ctlmx = controller_parabola(((float) (fb - p->ctl_fb_zero)) / 10.0);
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
		p->ctl_lr_zero = 1500;
	}
	e->ctl_lr = lr;
	//由2000～1600信号修正为-32.0 ～ +32.0角度
	//采用二次曲线来对倾斜角做过滤，使角度变化更平滑
	e->ctlmy = controller_parabola(((float) (lr - p->ctl_lr_zero)) / 10.0);

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

		e->lock_status |= (0x1 << 1);
		e->lock_status &= ~(0x1 << 2);
		return;
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
		p->ctl_pw_zero = 1100;
	}
	e->ctl_pw = pw;
	//读入速度
	float v = (float) (pw - p->ctl_pw_zero);
	//校验速度范围
	v = v > MAX_SPEED_RUN_MAX ? MAX_SPEED_RUN_MAX : v;
	v = v < MAX_SPEED_RUN_MIN ? MAX_SPEED_RUN_MIN : v;

	//在电机锁定时，停止转动，并禁用平衡补偿，保护措施
	if (e->lock || v < PROCTED_SPEED)
	{
		//设置速度为0
		v = 0;
	}

	if (e->mode == MODE_MANUAL)
	{
		//设置引擎的速度
		e->v = v;
	}

	//如果是最低油门
	if (abs(pw - p->ctl_pw_zero) < PROCTED_SPEED)
	{
		e->lock_status |= 0x1;
	}
	else
	{
		e->lock_status &= (~0x1);
	}
}

//读入摇控器第4通道PWM信号
void controller_md_pwm(int md)
{
	if (md < CTL_PWM_MIN || md > CTL_PWM_MAX)
	{
		return;
	}
	if (p->ctl_md_zero < CTL_PWM_MIN || p->ctl_md_zero > CTL_PWM_MAX)
	{
		p->ctl_md_zero = 2000;
	}
	e->ctl_md = md;
}

//读入摇控器第5通道PWM信号
void controller_ud_pwm(int ud)
{
	if (ud < CTL_PWM_MIN || ud > CTL_PWM_MAX)
	{
		return;
	}
	if (p->ctl_ud_zero < CTL_PWM_MIN || p->ctl_ud_zero > CTL_PWM_MAX)
	{
		p->ctl_ud_zero = 1060;
	}
	e->ctl_ud = ud;
}

//读入摇控器第6通道PWM信号
void controller_di_pwm(int di)
{
	if (di < CTL_PWM_MIN || di > CTL_PWM_MAX)
	{
		return;
	}
	if (p->ctl_di_zero < CTL_PWM_MIN || p->ctl_di_zero > CTL_PWM_MAX)
	{
		p->ctl_di_zero = 1000;
	}
	e->ctl_di = di;
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
	if (controller_abs(x) < 0.0001)
	{
		return 0;
	}
	float flag = x / controller_abs(x);
	float mxy = flag * (1.0 / 36.0) * (x * x);
	mxy = mxy > 40.0 ? 40.0 : mxy;
	mxy = mxy < -40.0 ? -40.0 : mxy;
	return mxy * M_PI / 180.0;
}

/***
 * est预估值
 * est_devi预估偏差
 * measure测量读数
 * measure_devi测量噪声
 * devi上一次最优偏差
 */
float controller_kalman_filter(float est, float est_devi, float measure, float measure_devi, float *devi)
{
	//预估高斯噪声的偏差
	float q = sqrt((*devi) * (*devi) + est_devi * est_devi);
	//卡尔曼增益
	float kg = q * q / (q * q + measure_devi * measure_devi);
	//滤波结果
	float val = est + kg * (measure - est);
	//最优偏差
	*devi = sqrt((1.0 - kg) * q * q);
	return val;
}

