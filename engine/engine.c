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

				printf("[FB: %4d LR: %4d PW: %4d] - [FB: %4d LR: %4d PW: %4d]\n", e->ctl_fb, e->ctl_lr, e->ctl_pw, p->ctl_fb_zero, p->ctl_lr_zero, p->ctl_pw_zero);

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

	//XYZ的增量式PID处理数据，当前、上一次、上上次
	float x_et = 0.0, x_et_1 = 0.0, x_et_2 = 0.0;
	float y_et = 0.0, y_et_1 = 0.0, y_et_2 = 0.0;
	float z_et = 0.0, z_et_1 = 0.0, z_et_2 = 0.0;

	//XYZ轴旋转角速度的增量式PID处理数据，当前、上一次、上上次
	float xv_et = 0.0, xv_et_1 = 0.0, xv_et_2 = 0.0;
	float yv_et = 0.0, yv_et_1 = 0.0, yv_et_2 = 0.0;
	float zv_et = 0.0, zv_et_1 = 0.0, zv_et_2 = 0.0;

	//XY轴加速度的增量式PID处理数据，当前、上一次、上上次
	float xa_et = 0.0, xa_et_1 = 0.0, xa_et_2 = 0.0;
	float ya_et = 0.0, ya_et_1 = 0.0, ya_et_2 = 0.0;

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
		//渐进式方向舵X轴
		if (e->mx < e->ctlmx)
		{
			e->mx += DIRECT_VALUE;
		}
		else if (e->mx > e->ctlmx)
		{
			e->mx -= DIRECT_VALUE;
		}
		//渐进式方向舵Y轴
		if (e->my < e->ctlmy)
		{
			e->my += DIRECT_VALUE;
		}
		else if (e->my > e->ctlmy)
		{
			e->my -= DIRECT_VALUE;
		}

		//处理X轴欧拉角平衡补偿
		//计算角度：欧拉角x + 校准补偿dx + 中心补偿cx + 移动倾斜角mx
		float x_angle = e->x + e->dx + params.cx + e->mx;
		//对X轴欧拉角卡尔曼滤波
		x_est = engine_kalman_filter(x_est, xyz_est_devi, x_angle, xyz_measure_devi, &x_devi);
		x_angle = x_est;
		//角度范围校验
		x_angle = x_angle < -MAX_ANGLE ? -MAX_ANGLE : x_angle;
		x_angle = x_angle > MAX_ANGLE ? MAX_ANGLE : x_angle;
		//设置PID数据
		x_et_2 = x_et_1;
		x_et_1 = x_et;
		x_et = x_angle;
		e->tx = x_angle;
		//使用XY轴的欧拉角的PID反馈控制算法
		e->x_devi = engine_pid(x_et, x_et_1, x_et_2, &e->x_sum);
		//得出引擎的X轴平衡补偿

		//处理Y轴欧拉角平衡补偿
		//计算角度：欧拉角y + 校准补偿dy + 中心补偿cy + 移动倾斜角my
		float y_angle = e->y + e->dy + params.cy + e->my;
		//对Y轴欧拉角卡尔曼滤波
		y_est = engine_kalman_filter(y_est, xyz_est_devi, y_angle, xyz_measure_devi, &y_devi);
		y_angle = y_est;
		//角度范围校验
		y_angle = y_angle < -MAX_ANGLE ? -MAX_ANGLE : y_angle;
		y_angle = y_angle > MAX_ANGLE ? MAX_ANGLE : y_angle;
		//设置PID数据
		y_et_2 = y_et_1;
		y_et_1 = y_et;
		y_et = y_angle;
		e->ty = y_angle;
		//使用XY轴的欧拉角的PID反馈控制算法
		e->y_devi = engine_pid(y_et, y_et_1, y_et_2, &e->y_sum);

		//处理Z轴欧拉角平衡补偿
		//计算角度：欧拉角z + 校准补偿dz
		float z_angle = e->z + e->dz;
		//对Z轴欧拉角卡尔曼滤波
		z_est = engine_kalman_filter(z_est, xyz_est_devi, z_angle, xyz_measure_devi, &z_devi);
		z_angle = z_est;
		//设置PID数据
		z_et_2 = z_et_1;
		z_et_1 = z_et;
		z_et = z_angle;
		//使用欧拉角的PID反馈控制算法
		e->z_devi = engine_pid(z_et, z_et_1, z_et_2, NULL);

		//处理X轴旋转角速度平衡补偿
		float gxv = e->gx + e->dgx;
		//对X轴角速度卡尔曼滤波
		xv_est = engine_kalman_filter(xv_est, xyz_v_est_devi, gxv, xyz_v_measure_devi, &xv_devi);
		gxv = xv_est;
		//设置X轴PID数据
		xv_et_2 = xv_et_1;
		xv_et_1 = xv_et;
		xv_et = gxv;
		//使用X轴的旋转角速度的PID反馈控制算法
		e->xv_devi = engine_pid_v(xv_et, xv_et_1, xv_et_2);

		//处理Y轴旋转角速度平衡补偿
		float gyv = e->gy + e->dgy;
		//对X轴角速度卡尔曼滤波
		yv_est = engine_kalman_filter(yv_est, xyz_v_est_devi, gyv, xyz_v_measure_devi, &yv_devi);
		gyv = yv_est;
		//设置Y轴PID数据
		yv_et_2 = yv_et_1;
		yv_et_1 = yv_et;
		yv_et = gyv;
		//使用Y轴的旋转角速度的PID反馈控制算法
		e->yv_devi = engine_pid_v(yv_et, yv_et_1, yv_et_2);

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
float engine_pid(float et, float et_1, float et_2, float *sum)
{
	s_engine *e = &engine;

	if (sum == NULL)
	{
		return 2.0 * params.kp * (et - et_1) + 4.0 * params.ki * et + 2.0 * params.kd * (et - 2 * et_1 + et_2);
	}

	float a = 2.0;
	*sum += params.ki / 20.0 * et;
	*sum = *sum > e->v / a ? e->v / a : *sum;
	*sum = *sum < -e->v / a ? -e->v / a : *sum;
	//增量式PID反馈控制
	return params.kp * (et - et_1) + params.ki * et + (*sum) + params.kd * (et - 2 * et_1 + et_2);
}

//对旋转角速度做PID反馈控制
float engine_pid_v(float et, float et_1, float et_2)
{
	s_engine *e = &engine;

	//增量式PID反馈控制
	return params.kp_v * (et - et_1) + params.ki_v * et + params.kd_v * (et - 2 * et_1 + et_2);
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
	//飞行移动倾斜角
	e->mx = 0;
	e->my = 0;
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
	//XYZ角速度补偿
	e->xv_devi = 0;
	e->yv_devi = 0;

	//XYZ欧拉角累加值
	e->x_sum = 0;
	e->y_sum = 0;
	//显示摇控器读数
	e->ctl_fb = 0;
	e->ctl_lr = 0;
	e->ctl_pw = 0;
	e->ctl_md = 0;
	//最低油门,最左，最右
	e->lock_status = 0;
	//0手动模式
	//1自动起飞模式
	//2自动降落模式
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

	e->mode = MODE_MANUAL;
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
