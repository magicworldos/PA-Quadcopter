/*
 * engine.c
 *
 *  Created on: Apr 12, 2016
 *
 *  四轴飞行控制器  Copyright (C) 2016  李德强
 */

#include <engine.h>
#include <driver.h>
#include <paramsctl.h>

//引擎
s_engine engine;
//参数
s_params params;
//参数缓存，保存用
s_params params_cache;

//信号量
sem_t sem_engine;
//多线程描述符
pthread_t pthd;
//显示摇控器读数
int ctl_fb = 0;
int ctl_lr = 0;
int ctl_pw = 0;

//最低油门,最左，最右
u32 lock_status = 0;

//启动引擎
void engine_start(int argc, char *argv[])
{
	//处理Ctrl + C退出信号
	signal(SIGINT, (void (*)(int)) &engine_exception);

	//初始化引擎信号量
	sem_init(&sem_engine, 0, 0);

	//处理启动参数
	if (argc >= 2)
	{
		//正常模式，飞行，调参
		if (strcmp(argv[1], "--fly") == 0)
		{
#ifndef __PC_TEST__
			//初始化驱动
			driver_setup();
			//mpu6050_setup();
#endif
			//重置引擎
			engine_reset(&engine);
			//载入参数
			params_load();
			//启动MPU6050陀螺仪数据读入线程
//			pthread_create(&pthd, (const pthread_attr_t*) NULL, (void* (*)(void*)) engine_mpu, NULL);
			//启动摇控器锁定、解锁电机
			pthread_create(&pthd, (const pthread_attr_t*) NULL, (void* (*)(void*)) &engine_lock, NULL);
			//启动飞行引擎
			pthread_create(&pthd, (const pthread_attr_t*) NULL, (void* (*)(void*)) &engine_fly, NULL);
			//启动键盘接收
			pthread_create(&pthd, (const pthread_attr_t*) NULL, (void* (*)(void*)) &params_input, NULL);
			//载入并执行动态链接库
			dlmod_init();

			//主线程休眠
			sem_wait(&sem_engine);

			return;
		}
		//校准摇控器模式
		if (strcmp(argv[1], "--ctl") == 0)
		{
			//初始化驱动
			driver_setup();
			//重置引擎
			engine_reset(&engine);
			//载入参数
			params_load();
			//启动键盘接收
			pthread_create(&pthd, (const pthread_attr_t*) NULL, (void* (*)(void*)) &params_input, NULL);

			int i = 0;
			int n = 100;
			int s_fb = 0;
			int s_lr = 0;
			int s_pw = 0;
			while (1)
			{
				printf("[FB: %4d LR: %4d PW: %4d] - [FB: %4d LR: %4d PW: %4d]\n", ctl_fb, ctl_lr, ctl_pw, params.ctl_fb_zero, params.ctl_lr_zero, params.ctl_pw_zero);

				s_fb += ctl_fb;
				s_lr += ctl_lr;
				s_pw += ctl_pw;

				if (i++ % n == 0)
				{
					params.ctl_fb_zero = s_fb / n;
					params.ctl_lr_zero = s_lr / n;
					params.ctl_pw_zero = s_pw / n;

					s_fb = 0;
					s_lr = 0;
					s_pw = 0;
				}

				usleep(10 * 1000);
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
				driver_ent_run(en_port, en_speed, en_msecs);

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

	while (1)
	{
		e->v_devi[0] = 0;
		e->v_devi[1] = 0;
		e->v_devi[2] = 0;
		e->v_devi[3] = 0;

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
		//角度范围校验
		x_angle = x_angle < -MAX_ANGLE ? -MAX_ANGLE : x_angle;
		x_angle = x_angle > MAX_ANGLE ? MAX_ANGLE : x_angle;
		//设置PID数据
		x_et_2 = x_et_1;
		x_et_1 = x_et;
		x_et = x_angle;
		//使用XY轴的欧拉角的PID反馈控制算法
		float x_devi = engine_pid(x_et, x_et_1, x_et_2);
		//得出引擎的X轴平衡补偿
		e->v_devi[0] += -x_devi;
		e->v_devi[2] += +x_devi;

		//处理Y轴欧拉角平衡补偿
		//计算角度：欧拉角y + 校准补偿dy + 中心补偿cy + 移动倾斜角my
		float y_angle = e->y + e->dy + params.cy + e->my;
		//角度范围校验
		y_angle = y_angle < -MAX_ANGLE ? -MAX_ANGLE : y_angle;
		y_angle = y_angle > MAX_ANGLE ? MAX_ANGLE : y_angle;
		//设置PID数据
		y_et_2 = y_et_1;
		y_et_1 = y_et;
		y_et = y_angle;
		//使用XY轴的欧拉角的PID反馈控制算法
		float y_devi = engine_pid(y_et, y_et_1, y_et_2);
		//得出引擎的Y轴平衡补偿
		e->v_devi[1] += -y_devi;
		e->v_devi[3] += +y_devi;

		//处理Z轴欧拉角平衡补偿
		//计算角度：欧拉角z + 校准补偿dz
		float z_angle = e->z + e->dz;
		//设置PID数据
		z_et_2 = z_et_1;
		z_et_1 = z_et;
		z_et = z_angle;
		//使用欧拉角的PID反馈控制算法
		float z_devi = engine_pid_z(z_et, z_et_1, z_et_2);
		//处理Z轴自旋补偿
		e->v_devi[0] += +z_devi;
		e->v_devi[2] += +z_devi;
		e->v_devi[1] += -z_devi;
		e->v_devi[3] += -z_devi;

		//处理XY轴旋转角速度平衡补偿
		float xv_devi = 0;
		float yv_devi = 0;
		float zv_devi = 0;

		//设置X轴PID数据
		xv_et_2 = xv_et_1;
		xv_et_1 = xv_et;
		xv_et = e->gx + e->dgx;
		//使用X轴的旋转角速度的PID反馈控制算法
		xv_devi = engine_pid_v(xv_et, xv_et_1, xv_et_2);

		//设置Y轴PID数据
		yv_et_2 = yv_et_1;
		yv_et_1 = yv_et;
		yv_et = e->gy + e->dgy;
		//使用Y轴的旋转角速度的PID反馈控制算法
		yv_devi = engine_pid_v(yv_et, yv_et_1, yv_et_2);

		//对引擎的4个轴做角速度平衡补偿
		e->v_devi[0] += -xv_devi;
		e->v_devi[2] += +xv_devi;
		e->v_devi[1] += +yv_devi;
		e->v_devi[3] += -yv_devi;

//		//处理XY轴旋加速度平衡补偿
//		float xa_devi = 0;
//		float ya_devi = 0;
//		//设置X轴PID数据
//		xa_et_2 = xa_et_1;
//		xa_et_1 = xa_et;
//		xa_et = e->ax + e->dax;
//		//使用X轴的旋转角速度的PID反馈控制算法
//		xa_devi = engine_pid_a(xa_et, xa_et_1, xa_et_2);
//
//		//设置Y轴PID数据
//		ya_et_2 = ya_et_1;
//		ya_et_1 = ya_et;
//		ya_et = e->ay + e->day;
//		//使用Y轴的旋转角速度的PID反馈控制算法
//		ya_devi = engine_pid_a(ya_et, ya_et_1, ya_et_2);
//
//		//对引擎的4个轴做加速度平衡补偿
//		e->v_devi[0] += +xa_devi;
//		e->v_devi[2] += -xa_devi;
//		e->v_devi[1] += +ya_devi;
//		e->v_devi[3] += -ya_devi;

		//引擎运转，调用驱动，调控电机转数
		engine_move(e);

		//原定计算频率1000Hz，但由于MPU6050的输出为100hz只好降低到100hz
		usleep(ENG_TIMER * 1000);
	}
}

//引擎运转
void engine_move(s_engine *e)
{
	//设置电机实际转数
	for (int i = 0; i < 4; i++)
	{
		//设置电机转数为做引擎速度 + 补偿
		e->speed[i] = (int) (e->v + e->v_devi[i]);
	}

	//校验电机转数范围
	engine_rechk_speed(e);

	//调用驱动设置电机转数
	driver_set_speed0(e->speed[0]);
	driver_set_speed1(e->speed[1]);
	driver_set_speed2(e->speed[2]);
	driver_set_speed3(e->speed[3]);
}

//校验电机转数范围
void engine_rechk_speed(s_engine *e)
{
	if (e->v > MAX_SPEED_RUN_MAX)
	{
		e->v = MAX_SPEED_RUN_MAX;
	}
	if (e->v < MAX_SPEED_RUN_MIN)
	{
		e->v = MAX_SPEED_RUN_MIN;
	}

	for (int i = 0; i < 4; i++)
	{
		if (e->speed[i] > MAX_SPEED_RUN_MAX)
		{
			e->speed[i] = MAX_SPEED_RUN_MAX;
		}
		if (e->speed[i] < MAX_SPEED_RUN_MIN)
		{
			e->speed[i] = MAX_SPEED_RUN_MIN;
		}

		//在电机锁定时，停止转动，并禁用平衡补偿，保护措施
		if (e->lock || e->v < PROCTED_SPEED)
		{
			//设置速度为0
			e->v = 0;
			e->speed[i] = 0;
			//在电机停转时，做陀螺仪补偿
			engine_set_dxy();
		}
	}
}

//取得陀螺仪读数
void engine_mpu()
{
	s_engine *e = &engine;
	while (1)
	{
		//读取yxz轴欧拉角、旋转角速度、加速度
		//xyz角速度，当x角变化时，y轴有旋转角速度；当月y角变化时，x轴有旋转角速度。
		//但为了编写程序和书写方便，x轴的欧拉角和y轴的角速度统一为x；y轴的欧拉角和x轴的角速度统一为y
		//mpu6050_value(&e->z, &e->y, &e->x, &e->gy, &e->gx, &e->gz, &e->ay, &e->ax, &e->az);
		//mpu6050_value(&e->z, &e->y, &e->x, &e->gx, &e->gy, &e->gz, &e->ay, &e->ax, &e->az);
		usleep(1);
	}
}

//取绝对值
float engine_abs(float value)
{
	if (value < 0)
	{
		return -value;
	}

	return value;
}

//读入摇控器“前/后”的PWM信号
void engine_fb_pwm(int fb)
{
	if (fb < CTL_PWM_MIN || fb > CTL_PWM_MAX)
	{
		return;
	}
	if (params.ctl_fb_zero < CTL_PWM_MIN || params.ctl_fb_zero > CTL_PWM_MAX)
	{
		params.ctl_fb_zero = 1400;
	}
	ctl_fb = fb;
	//由2000～1600信号修正为-32.0 ～ +32.0角度
	//采用二次曲线来对倾斜角做过滤，使角度变化更平滑
	engine.ctlmx = engine_parabola(((float) (fb - params.ctl_fb_zero)) / 50.0 * 4.0);
}

//读入摇控器“左/右”的PWM信号
void engine_lr_pwm(int lr)
{
	if (lr < CTL_PWM_MIN || lr > CTL_PWM_MAX)
	{
		return;
	}
	if (params.ctl_lr_zero < CTL_PWM_MIN || params.ctl_lr_zero > CTL_PWM_MAX)
	{
		params.ctl_lr_zero = 1400;
	}
	ctl_lr = lr;
	//由2000～1600信号修正为-32.0 ～ +32.0角度
	//采用二次曲线来对倾斜角做过滤，使角度变化更平滑
	engine.ctlmy = engine_parabola(((float) (lr - params.ctl_lr_zero)) / 50.0 * 4.0);

	//如果是最左或最右
	if (abs(lr - params.ctl_lr_zero) > 160)
	{
		//如果是最左
		if (lr - params.ctl_lr_zero < 0)
		{
			lock_status |= (0x1 << 2);
			lock_status &= ~(0x1 << 1);
			return;
		}
		else
		{
			lock_status |= (0x1 << 1);
			lock_status &= ~(0x1 << 2);
			return;
		}
	}
	lock_status &= ~(0x1 << 1);
	lock_status &= ~(0x1 << 2);
}

//读入摇控器“油门”的PWM信号
void engine_pw_pwm(int pw)
{
	if (pw < CTL_PWM_MIN || pw > CTL_PWM_MAX)
	{
		return;
	}
	if (params.ctl_pw_zero < CTL_PWM_MIN || params.ctl_pw_zero > CTL_PWM_MAX)
	{
		params.ctl_pw_zero = 1000;
	}
	ctl_pw = pw;
	//读入速度
	float v = (float) (pw - params.ctl_pw_zero);
	//设置引擎的速度
	engine.v = v;

	//如果是最低油门
	if (abs(pw - params.ctl_pw_zero) < 50)
	{
		lock_status |= 0x1;
	}
	else
	{
		lock_status &= (~0x1);
	}
}

//电机锁定解锁处理
void engine_lock()
{
	//动作开始时间
	struct timeval start;
	//动作计时
	struct timeval end;

	while (1)
	{
		u32 status = lock_status;
		//计时状态
		int timer_start = 0;
		while (1)
		{
			//最低油门方向最左方向最右
			if (!timer_start && (lock_status == 3 || lock_status == 5))
			{
				//开始计时
				gettimeofday(&start, NULL);
				//计时状态
				timer_start = 1;
			}

			if (timer_start)
			{
				if (status != lock_status)
				{
					break;
				}

				gettimeofday(&end, NULL);
				long timer = (end.tv_sec - start.tv_sec) * 1000000 + (end.tv_usec - start.tv_usec);
				if (timer >= 2 * 1000 * 1000)
				{
					//方向最左侧解锁电机
					if ((lock_status >> 2) & 0x1)
					{
						engine.lock = 0;
						break;
					}
					//方向最右侧锁定电机
					if ((lock_status >> 1) & 0x1)
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

//二次曲线函数
float engine_parabola(float x)
{
	float flag = x / engine_abs(x);
	return flag * (1.0 / 22.0) * (x * x);
}

//XY轴的欧拉角PID反馈控制
float engine_pid(float et, float et_1, float et_2)
{
	//增量式PID反馈控制
	return params.kp * (et - et_1) + (params.ki * et) + params.kd * (et - 2 * et_1 + et_2);
}

//Z轴的欧拉角PID反馈控制，参数与XY轴的PID不一样
float engine_pid_z(float et, float et_1, float et_2)
{
	//增量式PID反馈控制
	return params.kp_z * (et - et_1) + (params.ki_z * et) + params.kd_z * (et - 2 * et_1 + et_2);
}

//对旋转角速度做PID反馈控制
float engine_pid_v(float et, float et_1, float et_2)
{
	//增量式PID反馈控制
	return params.kp_v * (et - et_1) + (params.ki_v * et) + params.kd_v * (et - 2 * et_1 + et_2);
}

//对Z轴旋转角速度做PID反馈控制
float engine_pid_zv(float et, float et_1, float et_2)
{
	//增量式PID反馈控制
	return params.kp_zv * (et - et_1) + (params.ki_zv * et) + params.kd_zv * (et - 2 * et_1 + et_2);
}

//对XY轴加速度做PID反馈控制
float engine_pid_a(float et, float et_1, float et_2)
{
	et *= 40.0;
	et_1 *= 40.0;
	et_2 *= 40.0;
	//增量式PID反馈控制
	return params.kp_a * (et - et_1) + (params.ki_a * et) + params.kd_a * (et - 2 * et_1 + et_2);
}

//引擎重置
void engine_reset(s_engine *e)
{
	e->lock = 1;
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
	//XYZ轴加速度
	e->ax = 0;
	e->ay = 0;
	e->az = 0;
	//加速度修正补偿XYZ轴
	e->dax = 0;
	e->day = 0;
	e->daz = 0;
	//重置速度速度置为0
	e->v = 0;
	for (int i = 0; i < 4; i++)
	{
		//平衡补偿置为0
		e->v_devi[i] = 0;
		//电机实际速度置为0
		e->speed[i] = 0;
	}
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
	//补偿加速计仪读数，将3个轴的加速度都补偿为0
	e->dax = -e->ax;
	e->day = -e->ay;
	e->daz = -e->az;
}

//异常处理
void engine_exception()
{
	//引擎清理
	engine_clear();
	//清理动态链接库
	dlmod_destory();
	//退出
	exit(0);
}

//引擎清理
void engine_clear()
{
	//保存缓存中的参数
	params_save();
	//清理驱动
	driver_clear();
	//恢复控制台
	resetTermios();
	//重置引擎
	engine_reset(&engine);
	//关闭gy953
	//gy953_close();
}
