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
s32 s1 = 0;
s32 r = 0;
pthread_t pthd;
s_engine* e = NULL;
s_params* p = NULL;
FILE* fp = NULL;
s_buff_rw brw;
s8 logging = 0;
u64 logtime = 0;
struct timeval timer;
sem_t sem_write_buff;
sem_t sem_write_file;

s32 __init(s_engine* engine, s_params* params)
{
	e = engine;
	p = params;

	r = 1;
	s = 1;
	s1 = 1;

	sem_init(&sem_write_buff, 0, 1);
	sem_init(&sem_write_file, 0, 0);

	//读取环境变量，安装位置
	config_env(quad_home, "QUAD_HOME");

	brw.pos_w = 0;
	brw.pos_r = 0;
	brw.rw_size = 0;
	brw.buff_size = BUFF_SIZE;
	brw.buff = malloc(BUFF_SIZE);
	brw.data = malloc(WRITE_SIZE);

	if (brw.buff == NULL)
	{
		printf("[ ERROR ] Logger malloc memory error.\n");
		return -1;
	}
	if (brw.data == NULL)
	{
		printf("[ ERROR ] Logger malloc memory error.\n");
		return -1;
	}

	pthread_create(&pthddr, (const pthread_attr_t*) NULL, (void* (*)(void*)) &logger_run, NULL);
	pthread_create(&pthddr, (const pthread_attr_t*) NULL, (void* (*)(void*)) &logger_write, NULL);

	printf("[ OK ] Logger Init.\n");

	return 0;
}

s32 __destory(s_engine* e, s_params* p)
{
	r = 0;
	//wake up the write pthread.
	sem_post(&sem_write_file);
	usleep(10 * 1000);

	return 0;
}

s32 __status()
{
	return s + s1;
}

void logger_start()
{
	time_t now;
	struct tm *timenow;
	time(&now);
	timenow = localtime(&now);
	s8 logname[MAX_PATH_NAME];
	snprintf(logname, MAX_PATH_NAME, "%s/log/%04d-%02d-%02d_%02d-%02d-%02d.csv", quad_home, timenow->tm_year + 1900, timenow->tm_mon + 1, timenow->tm_mday, timenow->tm_hour, timenow->tm_min, timenow->tm_sec);
	fp = fopen(logname, "w+");
	if (fp == NULL)
	{
		printf("[ ERROR ] Open log error.\n");
		return;
	}
	gettimeofday(&timer, NULL);
	logtime = timer.tv_sec * 1000000 + timer.tv_usec;

	s32 count = snprintf(brw.data, WRITE_SIZE, "START_TIME");
	fwrite(brw.data, sizeof(u8), count, fp);

	count = snprintf(brw.data, WRITE_SIZE, ",E_LOCK");
	fwrite(brw.data, sizeof(u8), count, fp);

	count = snprintf(brw.data, WRITE_SIZE, ",E_TX,E_TY,E_TZ");
	fwrite(brw.data, sizeof(u8), count, fp);

	count = snprintf(brw.data, WRITE_SIZE, ",E_X,E_Y,E_Z");
	fwrite(brw.data, sizeof(u8), count, fp);

	count = snprintf(brw.data, WRITE_SIZE, ",E_CTLMX,E_CTLMY");
	fwrite(brw.data, sizeof(u8), count, fp);

	count = snprintf(brw.data, WRITE_SIZE, ",E_TGX,E_TGY,E_TGZ");
	fwrite(brw.data, sizeof(u8), count, fp);

	count = snprintf(brw.data, WRITE_SIZE, ",E_GX,E_GY,E_GZ");
	fwrite(brw.data, sizeof(u8), count, fp);

	count = snprintf(brw.data, WRITE_SIZE, ",E_AX,E_AY,E_AZ");
	fwrite(brw.data, sizeof(u8), count, fp);

	count = snprintf(brw.data, WRITE_SIZE, ",E_V");
	fwrite(brw.data, sizeof(u8), count, fp);

	count = snprintf(brw.data, WRITE_SIZE, ",E_XV_DEVI,E_YV_DEVI,E_ZV_DEVI");
	fwrite(brw.data, sizeof(u8), count, fp);

	count = snprintf(brw.data, WRITE_SIZE, ",E_CTL_FB,E->CTL_LR,E->CTL_PW,E->CTL_MD,E->CTL_UD,E->CTL_DI");
	fwrite(brw.data, sizeof(u8), count, fp);

	count = snprintf(brw.data, WRITE_SIZE, ",E_LOCK_STATUS,E_MODE");
	fwrite(brw.data, sizeof(u8), count, fp);

	count = snprintf(brw.data, WRITE_SIZE, ",P_KP,P_KI,P_KD");
	fwrite(brw.data, sizeof(u8), count, fp);

	count = snprintf(brw.data, WRITE_SIZE, ",P_V_KP,P_V_KI,P_V_KD");
	fwrite(brw.data, sizeof(u8), count, fp);

	count = snprintf(brw.data, WRITE_SIZE, ",P_VZ_KP,P_VZ_KI,P_VZ_KD");
	fwrite(brw.data, sizeof(u8), count, fp);

	count = snprintf(brw.data, WRITE_SIZE, ",P_CTL_FB_ZERO,P_CTL_LR_ZERO,P_CTL_PW_ZERO,P_CTL_MD_ZERO,P_CTL_UD_ZERO,P_CTL_DI_ZERO");
	fwrite(brw.data, sizeof(u8), count, fp);

	count = snprintf(brw.data, WRITE_SIZE, ",P_CTL_TYPE,P_CTL_DISPLAY");
	fwrite(brw.data, sizeof(u8), count, fp);

	count = snprintf(brw.data, WRITE_SIZE, "\n");
	fwrite(brw.data, sizeof(u8), count, fp);
}

void logger_stop()
{
	if (fp != NULL)
	{
		fclose(fp);
	}
	if (brw.buff != NULL)
	{
		free(brw.buff);
	}
	if (brw.data != NULL)
	{
		free(brw.data);
	}
}

void logger_write_data(int count)
{
	for (s32 i = 0; i < count; i++)
	{
		brw.buff[brw.pos_w++] = brw.data[i];
		brw.rw_size++;
		if (brw.pos_w >= brw.buff_size)
		{
			brw.pos_w = 0;
		}
	}
}

void logger_logging()
{
	sem_wait(&sem_write_buff);
	gettimeofday(&timer, NULL);
	u64 t = timer.tv_sec * 1000000 + timer.tv_usec;

	s32 count = snprintf(brw.data, WRITE_SIZE, "%u", t - logtime);
	logger_write_data(count);

	count = snprintf(brw.data, WRITE_SIZE, ",%d", e->lock);
	logger_write_data(count);

	count = snprintf(brw.data, WRITE_SIZE, ",%f,%f,%f", e->tx, e->ty, e->tz);
	logger_write_data(count);

	count = snprintf(brw.data, WRITE_SIZE, ",%f,%f,%f", e->x, e->y, e->z);
	logger_write_data(count);

	count = snprintf(brw.data, WRITE_SIZE, ",%f,%f", e->ctlmx, e->ctlmy);
	logger_write_data(count);

	count = snprintf(brw.data, WRITE_SIZE, ",%f,%f,%f", e->tgx, e->tgy, e->tgz);
	logger_write_data(count);

	count = snprintf(brw.data, WRITE_SIZE, ",%f,%f,%f", e->gx, e->gy, e->gz);
	logger_write_data(count);

	count = snprintf(brw.data, WRITE_SIZE, ",%f,%f,%f", e->ax, e->ay, e->az);
	logger_write_data(count);

	count = snprintf(brw.data, WRITE_SIZE, ",%f", e->v);
	logger_write_data(count);

	count = snprintf(brw.data, WRITE_SIZE, ",%f,%f,%f", e->xv_devi, e->yv_devi, e->zv_devi);
	logger_write_data(count);

	count = snprintf(brw.data, WRITE_SIZE, ",%u,%u,%u,%u,%u,%u", e->ctl_fb, e->ctl_lr, e->ctl_pw, e->ctl_md, e->ctl_ud, e->ctl_di);
	logger_write_data(count);

	count = snprintf(brw.data, WRITE_SIZE, ",%u,%u", e->lock_status, e->mode);
	logger_write_data(count);

	count = snprintf(brw.data, WRITE_SIZE, ",%f,%f,%f", p->kp, p->ki, p->kd);
	logger_write_data(count);

	count = snprintf(brw.data, WRITE_SIZE, ",%f,%f,%f", p->v_kp, p->v_ki, p->v_kd);
	logger_write_data(count);

	count = snprintf(brw.data, WRITE_SIZE, ",%f,%f,%f", p->vz_kp, p->vz_ki, p->vz_kd);
	logger_write_data(count);

	count = snprintf(brw.data, WRITE_SIZE, ",%u,%u,%u,%u,%u,%u", p->ctl_fb_zero, p->ctl_lr_zero, p->ctl_pw_zero, p->ctl_md_zero, p->ctl_ud_zero, p->ctl_di_zero);
	logger_write_data(count);

	count = snprintf(brw.data, WRITE_SIZE, ",%u,%u", p->ctl_type, p->ctl_display);
	logger_write_data(count);

	count = snprintf(brw.data, WRITE_SIZE, "\n");
	logger_write_data(count);

	if (brw.rw_size >= WRITE_SIZE)
	{
		sem_post(&sem_write_file);
	}
	sem_post(&sem_write_buff);
}

void logger_run()
{
	while (r)
	{
		//start logger
		if (!e->lock && !logging)
		{
			logger_start();
			logging = 1;
			printf("[ OK ] Logger started.\n");
		}
		//stop logger
		if (e->lock && logging)
		{
			logger_stop();
			logging = 0;
			printf("[ OK ] Logger stoped.\n");
		}
		//logging
		if (!e->lock)
		{
			logger_logging();
		}
		usleep(ENG_TIMER * 1000);
	}

	s = 0;
}

void logger_write()
{
	while (r)
	{
		sem_wait(&sem_write_file);
		if (!r)
		{
			break;
		}
		sem_wait(&sem_write_buff);
		s32 pos_w = brw.pos_w;
		s32 pos_r = brw.pos_r;
		s32 rw_size = brw.rw_size;
		brw.pos_r = brw.pos_w;
		brw.rw_size = 0;
		sem_post(&sem_write_buff);

		if (pos_r < pos_w)
		{
			fwrite(brw.buff + pos_r, sizeof(u8), pos_w - pos_r, fp);
		}
		else if (pos_r > pos_w)
		{
			fwrite(brw.buff + pos_r, sizeof(u8), brw.buff_size - pos_r, fp);
			fwrite(brw.buff, sizeof(u8), pos_w, fp);
		}
	}
	s1 = 0;
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
