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

			return;
		}

		//校准摇控器模式
		if (strcmp(argv[1], "--ctl") == 0)
		{
			//重置引擎
			engine_reset(&engine);
			s_engine *e = &engine;
			s_params *p = &params;

			s_dlmod *mod_paramsctl = dlmod_open("./lib/libparamsctl.so");
			if (mod_paramsctl == NULL)
			{
				return;
			}

			s_dlmod *mod_controller = dlmod_open("./lib/libcontroller.so");
			if (mod_controller == NULL)
			{
				return;
			}

			list_init(&list, &dlmod_free_mod);
			list_insert(&list, mod_paramsctl);
			list_insert(&list, mod_controller);
			list_visit(&list, (void *) &dlmod_run_pt_init);

			//摇控器pwm信号噪声
			float ctl_est_devi = 1;
			float ctl_measure_devi = 5;
			//前后卡尔曼滤波
			float fb_est = 0.0, fb_devi = 0.0, fb_est1 = 0.0, fb_devi1 = 0.0;
			//左右卡尔曼滤波
			float lr_est = 0.0, lr_devi = 0.0, lr_est1 = 0.0, lr_devi1 = 0.0;
			//油门卡尔曼滤波
			float pw_est = 0.0, pw_devi = 0.0, pw_est1 = 0.0, pw_devi1 = 0.0;
			//模式卡尔曼滤波
			float md_est = 0.0, md_devi = 0.0, md_est1 = 0.0, md_devi1 = 0.0;
			float ud_est = 0.0, ud_devi = 0.0, ud_est1 = 0.0, ud_devi1 = 0.0;
			float di_est = 0.0, di_devi = 0.0, di_est1 = 0.0, di_devi1 = 0.0;

			while (1)
			{
				//对方向舵前后通道做卡尔曼滤波
				e->ctl_fb;
				fb_est = engine_kalman_filter(fb_est, ctl_est_devi, e->ctl_fb, ctl_measure_devi, &fb_devi);
				fb_est1 = engine_kalman_filter(fb_est1, ctl_est_devi, fb_est, ctl_measure_devi, &fb_devi1);
				params.ctl_fb_zero = fb_est1;
				//对方向舵左右通道做卡尔曼滤波
				e->ctl_lr;
				lr_est = engine_kalman_filter(lr_est, ctl_est_devi, e->ctl_lr, ctl_measure_devi, &lr_devi);
				lr_est1 = engine_kalman_filter(lr_est1, ctl_est_devi, lr_est, ctl_measure_devi, &lr_devi1);
				params.ctl_lr_zero = lr_est1;
				//对油门通道做卡尔曼滤波
				e->ctl_pw;
				pw_est = engine_kalman_filter(pw_est, ctl_est_devi, e->ctl_pw, ctl_measure_devi, &pw_devi);
				pw_est1 = engine_kalman_filter(pw_est1, ctl_est_devi, pw_est, ctl_measure_devi, &pw_devi1);
				params.ctl_pw_zero = pw_est1;

				//对模式通道做卡尔曼滤波
				e->ctl_md;
				md_est = engine_kalman_filter(md_est, ctl_est_devi, e->ctl_md, ctl_measure_devi, &md_devi);
				md_est1 = engine_kalman_filter(md_est1, ctl_est_devi, md_est, ctl_measure_devi, &md_devi1);
				params.ctl_md_zero = md_est1;

				e->ctl_ud;
				ud_est = engine_kalman_filter(ud_est, ctl_est_devi, e->ctl_ud, ctl_measure_devi, &ud_devi);
				ud_est1 = engine_kalman_filter(ud_est1, ctl_est_devi, ud_est, ctl_measure_devi, &ud_devi1);
				params.ctl_ud_zero = ud_est1;

				e->ctl_di;
				di_est = engine_kalman_filter(di_est, ctl_est_devi, e->ctl_di, ctl_measure_devi, &di_devi);
				di_est1 = engine_kalman_filter(di_est1, ctl_est_devi, di_est, ctl_measure_devi, &di_devi1);
				params.ctl_di_zero = di_est1;

				printf("[FB: %4d LR: %4d PW: %4d MD: %4d UD: %4d DI: %4d ] - [FB: %4d LR: %4d PW: %4d MD: %4d UD: %4d DI: %4d]\n", e->ctl_fb, e->ctl_lr, e->ctl_pw, e->ctl_md, e->ctl_ud, e->ctl_di, p->ctl_fb_zero, p->ctl_lr_zero, p->ctl_pw_zero, p->ctl_md_zero, p->ctl_ud_zero, p->ctl_di_zero);

				usleep(2 * 1000);
			}

			return;
		}

		//电机调试模式
		if (strcmp(argv[1], "--test") == 0)
		{
			if (argc == 5)
			{
				int en_no;
				int en_port;
				int en_speed;
				int en_msecs;

				//电机所在GPIO引脚编号
				sscanf(argv[2], "%d", &en_port);
				//调定速度
				sscanf(argv[3], "%d", &en_speed);
				//调试时长，毫秒
				sscanf(argv[4], "%d", &en_msecs);
				//开始调试电机
				engine_ent_run(en_port, en_speed, en_msecs);

				return;
			}
		}
	}

	printf("unknown option ...\n");
	printf("usage: quadcopter\n");
	printf("\t[--fly: Fly with remote control and adjust quadcopter's parameters by keybroad.]\n");
	printf("\t[--test: Test the connection to the motor come in raspberry.]\n");
	printf("\t[--ctl: Display remote control values.]\n");
	printf("\tex. quadcopter --test [GPIO] [SPEED] [MSECS]\n");
	return;
}

//引擎核心算法平衡算法
void engine_fly()
{
	s_engine *e = &engine;

	//XYZ的增量式PID处理数据
	float x_et = 0.0;
	float y_et = 0.0;
	float z_et = 0.0;

	//xyz欧拉角噪声
	float xyz_est_devi = 0.01;
	float xyz_measure_devi = 0.05;
	//x轴欧拉角卡尔曼滤波
	float x_est = 0.0, x_devi = 0.0;
	//y轴欧拉角卡尔曼滤波
	float y_est = 0.0, y_devi = 0.0;
	//z轴欧拉角卡尔曼滤波
	float z_est = 0.0, z_devi = 0.0;
	//xy轴角速度噪声
	float xyz_v_est_devi = 0.01;
	float xyz_v_measure_devi = 0.01;
	//x轴角速度卡尔曼滤波
	float xv_est = 0.0, xv_devi = 0.0;
	//y轴角速度卡尔曼滤波
	float yv_est = 0.0, yv_devi = 0.0;

	while (1)
	{
		//处理X轴欧拉角平衡补偿
		//对X轴欧拉角卡尔曼滤波
		x_est = engine_kalman_filter(x_est, xyz_est_devi, e->x, xyz_measure_devi, &x_devi);
		e->tx = x_est + e->dx + params.cx + e->ctlmx;
		x_et = e->tx;
		//使用XY轴的欧拉角的PID反馈控制算法
		e->x_devi = engine_pid(x_et, e->dgx + e->gx, &e->x_sum);
		//得出引擎的X轴平衡补偿

		//处理Y轴欧拉角平衡补偿
		//对Y轴欧拉角卡尔曼滤波
		y_est = engine_kalman_filter(y_est, xyz_est_devi, e->y, xyz_measure_devi, &y_devi);
		e->ty = y_est + e->dy + params.cy + e->ctlmy;
		y_et = e->ty;
		//使用XY轴的欧拉角的PID反馈控制算法
		e->y_devi = engine_pid(y_et, e->dgy + e->gy, &e->y_sum);

		//处理Z轴欧拉角平衡补偿
		//计算角度：欧拉角z + 校准补偿dz
		float z_angle = e->z + e->dz;
		//对Z轴欧拉角卡尔曼滤波
		z_est = engine_kalman_filter(z_est, xyz_est_devi, z_angle, xyz_measure_devi, &z_devi);
		z_angle = z_est;
		z_et = z_angle;
		//使用欧拉角的PID反馈控制算法
		e->z_devi = engine_pidz(z_et, e->dgz + e->gz, &e->z_sum);

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
float engine_pid(float et, float dg, float *sum)
{
	s_engine *e = &engine;

	float mval = e->v / 2.0;
	float sv = engine_abs(et);
	float st = 0.1;
	if (et < 0)
	{
		st = -0.1;
	}
	*sum += sv < 0.1 ? params.ki * et : params.ki * st;
	*sum = *sum > mval ? mval : *sum;
	*sum = *sum < -mval ? -mval : *sum;

	float devi = params.kp * et + (*sum) + params.kd * dg;
	devi = devi > e->v ? e->v : devi;
	devi = devi < -e->v ? -e->v : devi;
	return devi;
}

//XY轴的欧拉角PID反馈控制
float engine_pidz(float et, float dg, float *sum)
{
	s_engine *e = &engine;

	float mval = e->v / 2.0;
	float sv = engine_abs(et);
	float st = 0.1;
	if (et < 0)
	{
		st = -0.1;
	}
	*sum += sv < 0.1 ? params.ki_z * et : params.ki_z * st;
	*sum = *sum > mval ? mval : *sum;
	*sum = *sum < -mval ? -mval : *sum;

	float devi = params.kp_z * et + (*sum) + params.kd_z * dg;
	devi = devi > e->v ? e->v : devi;
	devi = devi < -e->v ? -e->v : devi;
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
	//XYZ轴旋转角速度修正补偿
	e->dgx = 0;
	e->dgy = 0;
	e->dgz = 0;
	//重置速度速度置为0
	e->v = 0;
	//XYZ欧拉角补偿
	e->x_devi = 0;
	e->y_devi = 0;
	e->z_devi = 0;

	//XYZ欧拉角累加值
	e->x_sum = 0;
	e->y_sum = 0;
	e->z_sum = 0;
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
	//高度
	e->height = 0;
	//目标高度
	e->target_height = 0;
}

//陀螺仪补偿
void engine_set_dxy()
{
	s_engine *e = &engine;
	//补偿陀螺仪读数，将3个轴的欧拉角都补偿为0
	e->dx = -e->x;
	e->dy = -e->y;
	e->dz = -e->z;
	//补偿角速度读数，将3个轴的角速度都补偿为0
	e->dgx = -e->gx;
	e->dgy = -e->gy;
	e->dgz = -e->gz;

	e->x_sum = 0;
	e->y_sum = 0;
	e->z_sum = 0;
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
