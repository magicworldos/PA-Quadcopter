/*
 * gy953.c
 *
 *  Created on: Sep 9, 2016
 *
 *  四轴飞行控制器  Copyright (C) 2016  李德强
 */

#include <gy953.h>

int fd = 0;
u32 init = 0;
int r = 0;
int st = 0;
pthread_t pthd;
s_engine *e = NULL;
s_params *p = NULL;

//初始化陀螺仪
int __init(s_engine *engine, s_params *params)
{
	fd = serialOpen("/dev/ttyS0", 115200);
	if (fd <= 0)
	{
		printf("error\n");
		return -1;
	}

	//加速计陀螺仪校准
	if ((init >> 0) & 0x1)
	{
		serialPutchar(fd, 0xa5);
		serialPutchar(fd, 0x57);
		serialPutchar(fd, 0xFC);
		usleep(100000);
	}

	//磁场校准
	if ((init >> 1) & 0x1)
	{
		serialPutchar(fd, 0xa5);
		serialPutchar(fd, 0x58);
		serialPutchar(fd, 0xFD);
		usleep(100000);
	}

	//设置速率200hz
	if ((init >> 2) & 0x1)
	{
		serialPutchar(fd, 0xa5);
		serialPutchar(fd, 0xa6);
		serialPutchar(fd, 0x4B);
		usleep(100000);
	}

	//加速计ON/OFF
	if ((init >> 3) & 0x1)
	{
		serialPutchar(fd, 0xa5);
		serialPutchar(fd, 0x51);
		serialPutchar(fd, 0xF6);
		usleep(100000);
	}
	//陀螺仪ON/OFF
	if ((init >> 4) & 0x1)
	{
		serialPutchar(fd, 0xa5);
		serialPutchar(fd, 0x52);
		serialPutchar(fd, 0xF7);
		usleep(100000);
	}

	//磁场传感器ON/OFF
	if ((init >> 5) & 0x1)
	{
		serialPutchar(fd, 0xa5);
		serialPutchar(fd, 0x53);
		serialPutchar(fd, 0xF8);
		usleep(100000);
	}

	//恢复出厂设置
	if ((init >> 6) & 0x1)
	{
		serialPutchar(fd, 0xa5);
		serialPutchar(fd, 0x59);
		serialPutchar(fd, 0xFE);
		usleep(100000);
	}

	struct timeval start;
	struct timeval end;
	gettimeofday(&start, NULL);
	//初始化
	printf("Init GY053 ....\n");
	while (1)
	{
		gettimeofday(&end, NULL);
		long timer = (end.tv_sec - start.tv_sec) * 1000000 + (end.tv_usec - start.tv_usec);
		if (timer > 3000000)
		{
			//自动输出欧拉角
			serialPutchar(fd, 0xa5);
			serialPutchar(fd, 0x45);
			serialPutchar(fd, 0xEA);
			usleep(100000);

			//自动输出陀螺仪
			serialPutchar(fd, 0xa5);
			serialPutchar(fd, 0x25);
			serialPutchar(fd, 0xCA);
			usleep(100000);

			//自动输出加速计
			serialPutchar(fd, 0xa5);
			serialPutchar(fd, 0x15);
			serialPutchar(fd, 0xBA);
			usleep(100000);

			printf("Open GY953 auto value.\n");

			return 0;
		}

		unsigned char byte0 = (unsigned char) serialGetchar(fd);
		if (byte0 != 0x5a)
		{
			continue;
		}

		unsigned char byte1 = (unsigned char) serialGetchar(fd);
		if (byte1 != 0x5a)
		{
			continue;
		}

		unsigned char byte2 = (unsigned char) serialGetchar(fd);
		if (byte2 == 0x15 || byte2 == 0x25 || byte2 == 45)
		{
			float x, y, z;
			int err = gy953_read(&x, &y, &z);
			if (err == 0)
			{
				return 0;
			}
		}
	}

	st = 1;
	r = 1;
	e = engine;
	p = params;

	pthread_create(&pthd, (const pthread_attr_t*) NULL, (void* (*)(void*)) &gy953_value, NULL);

	printf("[ OK ] GY-953 Init.\n");

	return 0;
}

int __destory(s_engine *e, s_params *p)
{
	r = 0;
	if (fd != 0)
	{
		serialClose(fd);
		fd = 0;
	}
	return 0;
}

int __status()
{
	return st;
}

//循环读取陀螺仪读数
void gy953_value()
{
	usleep(1000);

	while (r)
	{
		usleep(1);

		//帧头固定数值0x5a
		unsigned char byte0 = (unsigned char) serialGetchar(fd);
		if (byte0 != 0x5a)
		{
			continue;
		}
		//帧头固定数值0x5a
		unsigned char byte1 = (unsigned char) serialGetchar(fd);
		if (byte0 != 0x5a || byte1 != 0x5a)
		{
			continue;
		}
		//读取帧类型
		unsigned char byte2 = (unsigned char) serialGetchar(fd);
		//欧拉角读数
		if (byte2 == 0x45)
		{
			gy953_read(&e->x, &e->y, &e->z);
			continue;
		}
		//陀螺仪读数
		if (byte2 == 0x25)
		{
			gy953_read(&e->gx, &e->gy, &e->gz);
			continue;
		}
		//加速计
		if (byte2 == 0x15)
		{
			gy953_read(&e->ax, &e->ay, &e->az);
			continue;
		}
	}
	st = 0;
}

//读取一个类型的数值
int gy953_read(float *x, float *y, float *z)
{
	unsigned char byte3 = (unsigned char) serialGetchar(fd);
	if (byte3 != 6)
	{
		return -1;
	}

	short xh = (short) serialGetchar(fd);
	short xl = (short) serialGetchar(fd);
	short yh = (short) serialGetchar(fd);
	short yl = (short) serialGetchar(fd);
	short zh = (short) serialGetchar(fd);
	short zl = (short) serialGetchar(fd);

	short xx = xh << 8 | xl;
	short yy = yh << 8 | yl;
	short zz = zh << 8 | zl;

	*x = (float) xx / 100.0;
	*y = (float) yy / 100.0;
	*z = (float) zz / 100.0;

	return 0;
}
