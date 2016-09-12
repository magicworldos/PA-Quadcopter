/*
 * driver.c
 *
 *  Created on: Apr 16, 2016
 *
 *  四轴飞行控制器  Copyright (C) 2016  李德强
 */

#include <driver.h>
#include <engine.h>

//多线程描述符
pthread_t pthdpwd[3];
pthread_t pthddr[4];
//4个电机
s_driver drv0;
s_driver drv1;
s_driver drv2;
s_driver drv3;

//驱动程序初始化
void driver_setup()
{
	//安装WiringPi
	wiringPiSetup();

	//设置摇控器3个通道到GPIO为输入引脚
	pinMode(GPIO_FB, INPUT);
	pinMode(GPIO_LR, INPUT);
	pinMode(GPIO_PW, INPUT);

	//设置电机的4个通道到GPIO为输出引脚
	pinMode(PORT_SPEED0, OUTPUT);
	pinMode(PORT_SPEED1, OUTPUT);
	pinMode(PORT_SPEED2, OUTPUT);
	pinMode(PORT_SPEED3, OUTPUT);

	//电机初始速度为0
	drv0.speed = 0;
	drv1.speed = 0;
	drv2.speed = 0;
	drv3.speed = 0;

	//初始化电调行程校准，通常是3秒最大油门，再3秒最小油门，但不是必要的，可以不做
	//driver_init_speed0();
	//driver_init_speed1();
	//driver_init_speed2();
	//driver_init_speed3();

	//启动摇控器接收机信号输入线程
	pthread_create(&pthdpwd[0], (const pthread_attr_t*) null, (void* (*)(void*)) &driver_ctl_pwm, (void *) &engine_fb_pwm);
	pthread_create(&pthdpwd[1], (const pthread_attr_t*) null, (void* (*)(void*)) &driver_ctl_pwm, (void *) &engine_lr_pwm);
	pthread_create(&pthdpwd[2], (const pthread_attr_t*) null, (void* (*)(void*)) &driver_ctl_pwm, (void *) &engine_pw_pwm);

	//启动电机信号输出线程
	pthread_create(&pthddr[0], (const pthread_attr_t*) null, (void* (*)(void*)) &driver_run0, null);
	pthread_create(&pthddr[1], (const pthread_attr_t*) null, (void* (*)(void*)) &driver_run1, null);
	pthread_create(&pthddr[2], (const pthread_attr_t*) null, (void* (*)(void*)) &driver_run2, null);
	pthread_create(&pthddr[3], (const pthread_attr_t*) null, (void* (*)(void*)) &driver_run3, null);

	printf("Init engine finished.\n");
}

//电机调试
void driver_ent_run(int en_port, int en_speed, int en_msecs)
{
	//安装WiringPi
	wiringPiSetup();
	//设置指定的GPIO引脚为输出引脚
	pinMode(en_port, OUTPUT);

	//设置指定的速度
	driver_pwm pwm;
	driver_set_pwm(en_speed, &pwm);

	//开始调试运行en_msecs毫秒，最多运行10000毫秒（10秒）
	en_msecs = en_msecs > MAX_INIT_MS ? MAX_INIT_MS : en_msecs;
	//由于一个PWM信号周期为2毫秒，所以调试时长要要除以2
	for (int i = 0; i < en_msecs / 2; i++)
	{
		//高电平
		digitalWrite(en_port, HIGH);
		usleep(pwm.time_m);
		//低电平
		digitalWrite(en_port, LOW);
		usleep(pwm.time_w);
	}

	//电机停止
	driver_set_pwm(0, &pwm);
	for (int i = 0; i < 100; i++)
	{
		//高电平
		digitalWrite(en_port, HIGH);
		usleep(pwm.time_m);
		//低电平
		digitalWrite(en_port, LOW);
		usleep(pwm.time_w);
	}
}

//读取摇控器接收机的PWM信号
void driver_ctl_pwm(void (*set_pwm)(int pwm))
{
	//高电平开始时间
	struct timeval start;
	//高电平结束时间
	struct timeval end;
	//状态 1:已开始读入高电平; 0: 未读入高电平
	int status_read_h = 0;
	//引脚值 1:高; 0: 低
	int value = 0;

	int gpio = GPIO_PW;
	if (set_pwm == engine_fb_pwm)
	{
		gpio = GPIO_FB;
	}
	else if (set_pwm == engine_lr_pwm)
	{
		gpio = GPIO_LR;
	}
	else if (set_pwm == engine_pw_pwm)
	{
		gpio = GPIO_PW;
	}

	int i = 0;
	int max = 0;
	int ctlval = 0;

	while (1)
	{
		//读取新信号
		status_read_h = 0;
		value = 0;
		do
		{
			//读取引脚的值
			value = digitalRead(gpio);
			//如果是高电平，且未开始计时
			if (value == 1 && !status_read_h)
			{
				//开始计时
				gettimeofday(&start, NULL);
				//修改状态为已开始计时
				status_read_h = 1;
			}
			//如果已开始计时，且读入低电平
			if (status_read_h && !value)
			{
				//结束计时
				gettimeofday(&end, NULL);
				//计算高电平时长
				long timer = (end.tv_sec - start.tv_sec) * 1000000 + (end.tv_usec - start.tv_usec);
				ctlval += timer;
				max = timer > max ? timer : max;
				if (i++ % 5 == 0)
				{
					//设定PWM到引擎
					set_pwm((ctlval - max) / 4);
					max = 0;
					ctlval = 0;
				}
				//结束本次信号读入
				break;
			}
			//如果已经开始，且未读到高电平
			if (!status_read_h && !value)
			{
				//状态 1:已开始读入低电平; 0: 未读入低电平
				int status_read_l = 0;
				//如是低电平
				if (!value)
				{
					//计时开始，并更新数值，表示已读入
					gettimeofday(&start, NULL);
					status_read_l = 1;
				}
				//如果已读入
				if (status_read_l)
				{
					//读取当前时间
					gettimeofday(&end, NULL);
					//计算时长
					long timer = (end.tv_sec - start.tv_sec) * 1000000 + (end.tv_usec - start.tv_usec);
					//如果超过2毫秒的时长则结束本次信号读入
					if (timer > 2000)
					{
						break;
					}
				}
			}
		}
		while (1);
		//每次读入信号休眠1微秒
		usleep(1);
	}
}

//清理驱动
void driver_clear()
{
	//电机停止
	driver_set_speed0(0);
	driver_set_speed1(0);
	driver_set_speed2(0);
	driver_set_speed3(0);

	usleep(100 * 1000);
}

//将电机速度转为PWM信号
void driver_set_pwm(int speed, driver_pwm *pwm)
{
	//高电平时长
	pwm->time_m = TIME_DEP + speed;
	//低电平时长
	pwm->time_w = TIME_PWM_WIDTH - pwm->time_m;
}

//对电机0发送PWM信号
void driver_run_pwm0(driver_pwm *pwm)
{
	//高电平
	digitalWrite(PORT_SPEED0, HIGH);
	usleep(pwm->time_m);
	//低电平
	digitalWrite(PORT_SPEED0, LOW);
	usleep(pwm->time_w);
}

//对电机1发送PWM信号
void driver_run_pwm1(driver_pwm *pwm)
{
	//高电平
	digitalWrite(PORT_SPEED1, HIGH);
	usleep(pwm->time_m);
	//低电平
	digitalWrite(PORT_SPEED1, LOW);
	usleep(pwm->time_w);
}

//对电机2发送PWM信号
void driver_run_pwm2(driver_pwm *pwm)
{
	//高电平
	digitalWrite(PORT_SPEED2, HIGH);
	usleep(pwm->time_m);
	//低电平
	digitalWrite(PORT_SPEED2, LOW);
	usleep(pwm->time_w);
}

//对电机3发送PWM信号
void driver_run_pwm3(driver_pwm *pwm)
{
	//高电平
	digitalWrite(PORT_SPEED3, HIGH);
	usleep(pwm->time_m);
	//低电平
	digitalWrite(PORT_SPEED3, LOW);
	usleep(pwm->time_w);
}

//向电机0发送PWM信号
void driver_run0()
{
	driver_pwm pwm;
	while (1)
	{
		driver_set_pwm(drv0.speed, &pwm);
		driver_run_pwm0(&pwm);
	}
}

//向电机1发送PWM信号
void driver_run1()
{
	driver_pwm pwm;
	while (1)
	{
		driver_set_pwm(drv1.speed, &pwm);
		driver_run_pwm1(&pwm);
	}
}

//向电机2发送PWM信号
void driver_run2()
{
	driver_pwm pwm;
	while (1)
	{
		driver_set_pwm(drv2.speed, &pwm);
		driver_run_pwm2(&pwm);
	}
}

//向电机3发送PWM信号
void driver_run3()
{
	driver_pwm pwm;
	while (1)
	{
		driver_set_pwm(drv3.speed, &pwm);
		driver_run_pwm3(&pwm);
	}
}

//设置电机0实际速度
void driver_set_speed0(int speed)
{
	drv0.speed = speed;
}

//设置电机1实际速度
void driver_set_speed1(int speed)
{
	drv1.speed = speed;
}

//设置电机2实际速度
void driver_set_speed2(int speed)
{
	drv2.speed = speed;
}

//设置电机3实际速度
void driver_set_speed3(int speed)
{
	drv3.speed = speed;
}

