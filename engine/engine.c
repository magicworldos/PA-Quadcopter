/*
 * engine.c
 *
 *  Created on: Apr 12, 2016
 *
 *  四轴飞行控制器  Copyright (C) 2016  李德强
 */

#include <emode.h>
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
void engine_start(int argc, char* argv[])
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
			emode_start_gyro(argv[2]);
			return;
		}

		//校准摇控器模式
		if (strcmp(argv[1], "--ctl") == 0)
		{
			emode_start_control();
			return;
		}

		//电机调试模式
		if (strcmp(argv[1], "--test") == 0 && argc == 5)
		{
			emode_start_test(argv[2], argv[3], argv[4]);
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
	s_engine* e = &engine;

	//欧拉角的上一次读数
	float x_et = 0.0;
	float y_et = 0.0;
	float z_et = 0.0;
	//角速度期望值
	float xv_et = 0.0;
	float yv_et = 0.0;
	float zv_et = 0.0;
	//角速度的上一次读数
	float x_v_et = 0.0;
	float y_v_et = 0.0;
	float z_v_et = 0.0;

	while (1)
	{
		//处理欧拉角平衡补偿
		e->tx = e->x + e->dx + e->dax + e->ctlmx;
		e->ty = e->y + e->dy + e->day + e->ctlmy;
		e->tz = e->z + e->dz;

		//期望角速度
		xv_et = e->tx * PV;
		yv_et = e->ty * PV;
		zv_et = e->tz * PV;

		xv_et = xv_et > MAXRA ? MAXRA : xv_et;
		xv_et = xv_et < -MAXRA ? -MAXRA : xv_et;

		yv_et = yv_et > MAXRA ? MAXRA : yv_et;
		yv_et = yv_et < -MAXRA ? -MAXRA : yv_et;

		zv_et = zv_et > MAXRA ? MAXRA : zv_et;
		zv_et = zv_et < -MAXRA ? -MAXRA : zv_et;

		//使用欧拉角的PID反馈控制算法
		e->x_devi = engine_pid(e->tx, x_et, &e->x_sum);
		e->y_devi = engine_pid(e->ty, y_et, &e->y_sum);
		e->z_devi = engine_pid(e->tz, z_et, NULL);

		//角速度PID
		e->xv_devi = engine_v_pid(xv_et + (e->gx + e->dgx), x_v_et + xv_et, &e->x_v_sum);
		e->yv_devi = engine_v_pid(yv_et + (e->gy + e->dgy), y_v_et + yv_et, &e->y_v_sum);
		e->zv_devi = engine_v_pid(zv_et + (e->gz + e->dgz), z_v_et + zv_et, NULL);

		//记录欧拉角的上一次读数
		x_et = e->tx;
		y_et = e->ty;
		z_et = e->tz;

		//记录角速度的上一次读数
		x_v_et = e->gx + e->dgx;
		y_v_et = e->gy + e->dgy;
		z_v_et = e->gz + e->dgz;

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
	s_engine* e = &engine;

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

// XY轴的欧拉角PID反馈控制
float engine_pid(float et, float et2, float* sum)
{
	s_engine* e = &engine;

	//计算积分参数累加和，消除稳态误差
	if (sum == NULL)
	{
		//对X、Y轴做PID反馈控制
		float devi = params.kp * et + params.kd * (et - et2);
		engine_limit(&devi);
		//返回Z轴补偿值
		return devi;
	}

	*sum += params.ki * et;
	engine_limit(sum);

	//对X、Y轴做PID反馈控制
	float devi = params.kp * et + (*sum) + params.kd * (et - et2);
	engine_limit(&devi);
	//返回X、Y轴补偿值
	return devi;
}

// XY轴的欧拉角PID反馈控制
float engine_v_pid(float et, float et2, float* sum)
{
	s_engine* e = &engine;

	//计算积分参数累加和，消除稳态误差
	if (sum == NULL)
	{
		//对X、Y轴做PID反馈控制
		float devi = params.v_kp * et + params.v_kd * (et - et2);
		engine_limit(&devi);
		//返回Z轴补偿值
		return devi;
	}

	//计算积分参数累加和，消除稳态误差
	*sum += params.v_ki * et;
	engine_limit(sum);

	//对X、Y轴做PID反馈控制
	float devi = params.v_kp * et + (*sum) + params.v_kd * (et - et2);
	engine_limit(&devi);
	//返回X、Y轴补偿值
	return devi;
}

//速度限幅
void engine_limit(float* v)
{
	if (v == NULL)
	{
		return;
	}
	s_engine* e = &engine;
	*v = *v > e->v ? e->v : *v;
	*v = *v < -e->v ? -e->v : *v;
}

/***
 * est预估值
 * est_devi预估偏差
 * measure测量读数
 * measure_devi测量噪声
 * devi上一次最优偏差
 */
float engine_kalman_filter(float est, float est_devi, float measure, float measure_devi, float* devi)
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
void engine_reset(s_engine* e)
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
	e->dax = 0;
	e->day = 0;
	// XYZ欧拉角
	e->x = 0;
	e->y = 0;
	e->z = 0;
	// XYZ加速度
	e->ax = 0;
	e->ay = 0;
	e->az = 0;
	//摇控器飞行移动倾斜角
	e->ctlmx = 0;
	e->ctlmy = 0;
	// XYZ轴旋转角速度
	e->gx = 0;
	e->gy = 0;
	e->gz = 0;
	//陀螺仪修正补偿XYZ轴
	e->dgx = 0;
	e->dgy = 0;
	e->dgz = 0;
	//重置速度速度置为0
	e->v = 0;
	// XYZ欧拉角补偿
	e->x_devi = 0;
	e->y_devi = 0;
	e->z_devi = 0;
	// XYZ角速度补偿
	e->xv_devi = 0;
	e->yv_devi = 0;
	e->zv_devi = 0;

	// XYZ欧拉角累加值
	e->x_sum = 0;
	e->y_sum = 0;
	e->z_sum = 0;
	// XYZ角速度累加值
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
	//高度
	e->height = 0;
	e->height_target = 0;
	e->h_devi = 0;
	e->h_sum = 0;
	//最低油门,最左，最右
	e->lock_status = 0;
	// 0手动模式
	// 1自动定高模式
	e->mode = MODE_MANUAL;
}

//陀螺仪补偿
void engine_set_dxy()
{
	s_engine* e = &engine;
	//补偿陀螺仪读数，将3个轴的欧拉角都补偿为0
	e->dx = -e->x;
	e->dy = -e->y;
	e->dz = -e->z;

	//补偿陀螺仪读数，将3个轴的角速度都补偿为0
	e->dgx = -e->gx;
	e->dgy = -e->gy;
	e->dgz = -e->gz;

	e->x_sum = 0;
	e->y_sum = 0;
	e->z_sum = 0;

	e->x_v_sum = 0;
	e->y_v_sum = 0;
	e->z_v_sum = 0;

	e->h_sum = 0;

	if (engine_abs(e->ax) < MAX_ACC)
	{
		e->dax = asin(e->ax / MAX_ACC);
	}
	if (engine_abs(e->ay) < MAX_ACC)
	{
		e->day = asin(e->ay / MAX_ACC);
	}

	e->x_devi = 0;
	e->y_devi = 0;
	e->z_devi = 0;
	e->h_devi = 0;
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
