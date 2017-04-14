/*
 * engine.c
 *
 *  Created on: Apr 12, 2016
 *
 *  四轴飞行控制器  Copyright (C) 2016  李德强
 */

#include <engine.h>

//引擎
s_engine engine;
//参数
s_params params;
//参数缓存，保存用
s_params params_cache;
//用于存放动态链接库
s_list list;

//信号量
sem_t sem_engine;
//多线程描述符
pthread_t pthd;

//启动引擎
void engine_start(int argc, char *argv[])
{
	//处理Ctrl + C退出信号
	signal(SIGINT, (void (*)(int)) &engine_handler);

	//初始化引擎信号量
	sem_init(&sem_engine, 0, 0);

	//处理启动参数
	if (argc >= 2)
	{
		//初始化WiringPi
		wiringPiSetup();

		//正常模式，飞行，调参
		if (strcmp(argv[1], "--fly") == 0)
		{
			engine_start_fly();
			return;
		}

		//陀螺仪读数模式
		if (strcmp(argv[1], "--gyro") == 0 && argc == 3)
		{
			engine_start_gyro(argv[2]);
			return;
		}

		//校准摇控器模式
		if (strcmp(argv[1], "--ctl") == 0)
		{
			engine_start_control();
			return;
		}

		//电机调试模式
		if (strcmp(argv[1], "--test") == 0 && argc == 5)
		{
			engine_start_test(argv[2], argv[3], argv[4]);
			return;
		}
	}

	printf("unknown option ...\n");
	printf("usage: quadcopter\n");
	printf("\t[--fly: Fly with remote control and adjust quadcopter's parameters by keybroad.]\n");
	printf("\t[--test [GPIO] [SPEED 0-1000] [MS]: Test the connection to the motor come in raspberry.]\n");
	printf("\t[--ctl: Display remote control values.]\n");
	printf("\t[--gyro [MODULE]: Display gyro values.]\n");
	printf("\tex. quadcopter --test [GPIO] [SPEED] [MSECS]\n");
	return;
}

//启动飞行模式
void engine_start_fly()
{
	//重置引擎
	engine_reset(&engine);
	//启动摇控器锁定、解锁电机
	pthread_create(&pthd, (const pthread_attr_t*) NULL, (void* (*)(void*)) &engine_lock, NULL);
	//启动飞行引擎
	pthread_create(&pthd, (const pthread_attr_t*) NULL, (void* (*)(void*)) &engine_fly, NULL);
	//载入并执行动态链接库
	dlmod_init();

	//主线程休眠
	sem_wait(&sem_engine);
}

//引擎核心算法平衡算法
void engine_fly()
{
	s_engine *e = &engine;

	//欧拉角的上一次读数
	float x_et = 0.0;
	float y_et = 0.0;
	float z_et = 0.0;
	//角速度的上一次读数
	float x_v_et = 0.0;
	float y_v_et = 0.0;
	float z_v_et = 0.0;

	while (1)
	{
		//处理欧拉角平衡补偿
		e->tx = e->x + e->dx + params.cx + e->ctlmx;
		e->ty = e->y + e->dy + params.cy + e->ctlmy;
		e->tz = e->z + e->dz;

		//使用欧拉角的PID反馈控制算法
		e->x_devi = engine_pid(e->tx, x_et, &e->x_sum);
		e->y_devi = engine_pid(e->ty, y_et, &e->y_sum);
		e->z_devi = engine_pid(e->tz, z_et, &e->z_sum);

		//角速度PID
		e->xv_devi = engine_v_pid(e->gx, x_v_et, &e->x_v_sum);
		e->yv_devi = engine_v_pid(e->gy, y_v_et, &e->y_v_sum);
		e->zv_devi = engine_v_pid(e->gz, z_v_et, &e->z_v_sum);

		//记录欧拉角的上一次读数
		x_et = e->tx;
		y_et = e->ty;
		z_et = e->tz;

		//记录角速度的上一次读数
		x_v_et = e->gx;
		y_v_et = e->gy;
		z_v_et = e->gz;

		//在电机锁定时，停止转动，并禁用平衡补偿，保护措施
		if (e->lock || e->v < PROCTED_SPEED)
		{
			//设置速度为0
			e->v = 0;
			//在电机停转时，做陀螺仪补偿
			engine_set_dxy();
		}
		//原定计算频率1000Hz，但由于MPU6050的输出为100hz只好降低到100hz
		usleep(ENG_TIMER * 1000);
	}
}

void engine_start_gyro(char* argv2)
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

void engine_start_control()
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

void engine_start_test(char* argv2,char* argv3,char* argv4)
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
	engine_ent_run(en_port, en_speed, en_msecs);
}

//电机锁定解锁处理
void engine_lock()
{
	s_engine *e = &engine;

	//动作开始时间
	struct timeval start;
	//动作计时
	struct timeval end;

	while (1)
	{
		u32 status = e->lock_status;
		//计时状态
		int timer_start = 0;
		while (1)
		{
			//最低油门方向最左方向最右
			if (!timer_start && (e->lock_status == 3 || e->lock_status == 5))
			{
				//开始计时
				gettimeofday(&start, NULL);
				//计时状态
				timer_start = 1;
			}

			if (timer_start)
			{
				if (status != e->lock_status)
				{
					break;
				}

				gettimeofday(&end, NULL);
				long timer = (end.tv_sec - start.tv_sec) * 1000000 + (end.tv_usec - start.tv_usec);
				if (timer >= 1000 * 1000)
				{
					//方向最左侧解锁电机
					if ((e->lock_status >> 2) & 0x1)
					{
						engine.lock = 0;
						break;
					}
					//方向最右侧锁定电机
					if ((e->lock_status >> 1) & 0x1)
					{
						engine.lock = 1;
						break;
					}
				}
			}
			usleep(100 * 1000);
		}
		usleep(100 * 1000);
	}
}

//XY轴的欧拉角PID反馈控制
float engine_pid(float et, float et2, float *sum)
{
	s_engine *e = &engine;

	//平衡补偿最值范围，保护措施，防止在油门很小时积分参数产生的累加和太大导致飞行侧翻
	float mval = e->v / 2.0;
	//计算积分参数累加和，消除稳态误差
	*sum += params.ki * et;
	//校验积分参数累加和补偿最值范围
	*sum = *sum > mval ? mval : *sum;
	*sum = *sum < -mval ? -mval : *sum;

	//对X、Y轴做PID反馈控制
	float devi = params.kp * et + (*sum) + params.kd * (et - et2);
	//校验平衡补偿最值范围
	devi = devi > mval ? mval : devi;
	devi = devi < -mval ? -mval : devi;
	//返回X、Y轴补偿值
	return devi;
}

//XY轴的欧拉角PID反馈控制
float engine_v_pid(float et, float et2, float *sum)
{
	s_engine *e = &engine;

	//平衡补偿最值范围，保护措施，防止在油门很小时积分参数产生的累加和太大导致飞行侧翻
	float mval = e->v / 2.0;
	//计算积分参数累加和，消除稳态误差
	*sum += params.v_ki * et;
	//校验积分参数累加和补偿最值范围
	*sum = *sum > mval ? mval : *sum;
	*sum = *sum < -mval ? -mval : *sum;

	//对X、Y轴做PID反馈控制
	float devi = params.v_kp * et + (*sum) + params.v_kd * (et - et2);
	//校验平衡补偿最值范围
	devi = devi > mval ? mval : devi;
	devi = devi < -mval ? -mval : devi;
	//返回X、Y轴补偿值
	return devi;
}

/***
 * est预估值
 * est_devi预估偏差
 * measure测量读数
 * measure_devi测量噪声
 * devi上一次最优偏差
 */
float engine_kalman_filter(float est, float est_devi, float measure, float measure_devi, float *devi)
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

//引擎重置
void engine_reset(s_engine *e)
{
	e->lock = 1;
	//实际欧拉角
	e->tx = 0;
	e->ty = 0;
	e->tz = 0;
	//陀螺仪修正补偿XYZ轴
	e->dx = 0;
	e->dy = 0;
	e->dz = 0;
	//XYZ欧拉角
	e->x = 0;
	e->y = 0;
	e->z = 0;
	//摇控器飞行移动倾斜角
	e->ctlmx = 0;
	e->ctlmy = 0;
	//XYZ轴旋转角速度
	e->gx = 0;
	e->gy = 0;
	e->gz = 0;
	//重置速度速度置为0
	e->v = 0;
	//XYZ欧拉角补偿
	e->x_devi = 0;
	e->y_devi = 0;
	e->z_devi = 0;
	//XYZ角速度补偿
	e->xv_devi = 0;
	e->yv_devi = 0;
	e->zv_devi = 0;

	//XYZ欧拉角累加值
	e->x_sum = 0;
	e->y_sum = 0;
	e->z_sum = 0;
	//XYZ角速度累加值
	e->x_v_sum = 0;
	e->y_v_sum = 0;
	e->z_v_sum = 0;
	//显示摇控器读数
	e->ctl_fb = 0;
	e->ctl_lr = 0;
	e->ctl_pw = 0;
	e->ctl_md = 0;
	e->ctl_ud = 0;
	e->ctl_di = 0;
	//最低油门,最左，最右
	e->lock_status = 0;
	//0手动模式
	//1自动定高模式
	e->mode = MODE_MANUAL;
}

//陀螺仪补偿
void engine_set_dxy()
{
	s_engine *e = &engine;
	//补偿陀螺仪读数，将3个轴的欧拉角都补偿为0
	e->dx = -e->x;
	e->dy = -e->y;
	e->dz = -e->z;

	e->x_sum = 0;
	e->y_sum = 0;
	e->z_sum = 0;

	e->x_v_sum = 0;
	e->y_v_sum = 0;
	e->z_v_sum = 0;
}

//绝对值
float engine_abs(float v)
{
	if (v < 0)
	{
		return -v;
	}
	return v;
}

//电机调试
void engine_ent_run(int en_port, int en_speed, int en_msecs)
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

//系统信号处理
void engine_handler()
{
	//重置引擎
	engine_reset(&engine);

	//清理动态链接库
	dlmod_destory();

	//退出
	exit(0);
}
