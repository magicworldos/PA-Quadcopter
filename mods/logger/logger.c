/*
 * driver.c
 *
 *  Created on: Apr 16, 2016
 *
 *  四轴飞行控制器  Copyright (C) 2016  李德强
 */

#include <logger.h>

//环境变量，安装位置
s8 quad_home[MAX_PATH_NAME];

pthread_t pthddr;
s32 s = 0;
s32 r = 0;
pthread_t pthd;
s_engine* e = NULL;
s_params* p = NULL;
FILE* fp = NULL;
s8* buff = NULL;

s32 __init(s_engine* engine, s_params* params)
{
	e = engine;
	p = params;

	r = 1;
	s = 1;

	//读取环境变量，安装位置
	config_env(quad_home, "QUAD_HOME");

	time_t now;
	struct tm *timenow;
	time(&now);
	timenow = localtime(&now);
	s8 logname[MAX_PATH_NAME];
	snprintf(logname, MAX_PATH_NAME, "%s/%04d-%02d-%02d_%02d-%02d-%02d.csv", quad_home, timenow->tm_year + 1900, timenow->tm_mon + 1, timenow->tm_mday, timenow->tm_hour, timenow->tm_min, timenow->tm_sec);
	buff = malloc(BUFF_SIZE);
	if (buff == NULL)
	{
		printf("[ ERROR ] Malloc memory.\n");
		return -1;
	}
	fp = fopen(logname, "w+");
	if (fp == NULL)
	{
		printf("[ ERROR ] Open log error.\n");
		return -1;
	}

	pthread_create(&pthddr, (const pthread_attr_t*) NULL, (void* (*)(void*)) &logger_run, NULL);

	printf("[ OK ] Logger Init.\n");

	return 0;
}

s32 __destory(s_engine* e, s_params* p)
{
	if (fp != NULL)
	{
		fclose(fp);
	}
	if (buff != NULL)
	{
		free(buff);
	}
	r = 0;
	return 0;
}

s32 __status()
{
	return s;
}

void logger_run()
{
	while (r)
	{
		if (!e->lock)
		{
			s32 count = snprintf(buff, BUFF_SIZE, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", e->v, e->ax, e->ay, e->az, e->gx + e->dgx, e->gy + e->dgy, e->gz + e->dgz, e->x + e->dx, e->y + e->dy, e->z + e->dz);
			fwrite(buff, sizeof(s8), count, fp);
		}
		usleep(ENG_TIMER * 1000);
	}

	s = 0;
}

int config_env(char* value, char *name)
{
	if (name == NULL || value == NULL)
	{
		return -1;
	}
	char* v = getenv(name);
	memcpy(value, v, strlen(v) + 1);

	return 0;
}
