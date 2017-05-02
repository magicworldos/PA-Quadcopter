/*
 * emode.c
 *
 *  Created on: Apr 26, 2017
 *      Author: lidq
 */

#include <emode.h>
#include <engine.h>

//引擎
extern s_engine engine;
//参数
extern s_params params;
//用于存放动态链接库
extern s_list list;

void emode_start_gyro(char* argv2)
{
	char modname[0x200];
	snprintf(modname, 0x200, "./lib/lib%s.so", argv2);
	//重置引擎
	engine_reset(&engine);
	s_engine *e = &engine;
	s_params *p = &params;

	//载入MPU6050模块
	s_dlmod *mod_gyro = dlmod_open(modname);
	if (mod_gyro == NULL)
	{
		return;
	}

	//初始化模块链表
	list_init(&list, &dlmod_free_mod);
	//加入陀螺仪模块
	list_insert(&list, mod_gyro);
	//运行模块功能
	list_visit(&list, (void *) &dlmod_run_pt_init);

	while (1)
	{
		printf("[xyz: %+7.3f %+7.3f %+7.3f ][g: %+7.3f %+7.3f %+7.3f]\n", e->x, e->y, e->z, e->gx, e->gy, e->gz);
		usleep(2 * 1000);
	}
}

void emode_start_control()
{
	//重置引擎
	engine_reset(&engine);
	s_engine *e = &engine;
	s_params *p = &params;

	//载入参数调整模块
	s_dlmod *mod_paramsctl = dlmod_open("./lib/libparamsctl.so");
	if (mod_paramsctl == NULL)
	{
		return;
	}

	//载入摇控器模块
	s_dlmod *mod_controller = dlmod_open("./lib/libcontroller.so");
	if (mod_controller == NULL)
	{
		return;
	}

	//初始化模块链表
	list_init(&list, &dlmod_free_mod);
	//加入参数控制模块
	list_insert(&list, mod_paramsctl);
	//加入摇控器模块
	list_insert(&list, mod_controller);
	//运行模块功能
	list_visit(&list, (void *) &dlmod_run_pt_init);

	while (1)
	{
		//方向前后
		params.ctl_fb_zero = e->ctl_fb;
		//方向左右
		params.ctl_lr_zero = e->ctl_lr;
		//油门
		params.ctl_pw_zero = e->ctl_pw;
		params.ctl_md_zero = e->ctl_md;
		params.ctl_ud_zero = e->ctl_ud;
		params.ctl_di_zero = e->ctl_di;

		printf("[FB: %4d LR: %4d PW: %4d MD: %4d UD: %4d DI: %4d ] - [FB: %4d LR: %4d PW: %4d MD: %4d UD: %4d DI: %4d]\n", e->ctl_fb, e->ctl_lr, e->ctl_pw, e->ctl_md, e->ctl_ud, e->ctl_di, p->ctl_fb_zero, p->ctl_lr_zero, p->ctl_pw_zero, p->ctl_md_zero, p->ctl_ud_zero, p->ctl_di_zero);

		usleep(2 * 1000);
	}
}

void emode_start_test(char* argv2, char* argv3, char* argv4)
{
	//GPIO引脚编号
	int en_port;
	//速度
	int en_speed;
	//时长
	int en_msecs;

	//电机所在GPIO引脚编号
	sscanf(argv2, "%d", &en_port);
	//调定速度
	sscanf(argv3, "%d", &en_speed);
	//调试时长，毫秒
	sscanf(argv4, "%d", &en_msecs);
	//开始调试电机
	emode_ent_run(en_port, en_speed, en_msecs);
}

//电机调试
void emode_ent_run(int en_port, int en_speed, int en_msecs)
{
	//设置指定的GPIO引脚为输出引脚
	pinMode(en_port, OUTPUT);

	//开始调试运行en_msecs毫秒，最多运行10000毫秒（10秒）
	en_msecs = en_msecs > TEST_MAX_MS ? TEST_MAX_MS : en_msecs;
	//由于一个PWM信号周期为2毫秒，所以调试时长要要除以2
	for (int i = 0; i < en_msecs / 2; i++)
	{
		//高电平
		digitalWrite(en_port, HIGH);
		usleep(TEST_ZERO_MS + en_speed);
		//低电平
		digitalWrite(en_port, LOW);
		usleep(TEST_ZERO_MS - en_speed);
	}

	//停止
	for (int i = 0; i < 3000; i++)
	{
		//高电平
		digitalWrite(en_port, HIGH);
		usleep(1000);
		//低电平
		digitalWrite(en_port, LOW);
		usleep(1000);
	}
}
