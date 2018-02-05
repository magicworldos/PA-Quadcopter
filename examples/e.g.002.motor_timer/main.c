#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>
#include <time.h>

int port = 1;
int run = 1;
int speed = 0;
struct timeval timers;
struct timeval timere;
struct timeval timers2;
struct timeval timere2;
unsigned long long ts = 0;
unsigned long long te = 0;
void motor_run()
{
	while (run)
	{
		gettimeofday(&timers2, NULL);
		ts = timers2.tv_sec * 1000000 + timers2.tv_usec - (timere2.tv_sec * 1000000 + timere2.tv_usec);
		timers.tv_sec = 0;
		timers.tv_usec = (1000 + speed);
		while (select(0, 0, 0, 0, &timers) < 0)
			;
		gettimeofday(&timere2, NULL);
		te = timere2.tv_sec * 1000000 + timere2.tv_usec - (timers2.tv_sec * 1000000 + timers2.tv_usec);
		timere.tv_sec = 0;
		timere.tv_usec = (1000 - speed);
		while (select(0, 0, 0, 0, &timere) < 0)
			;
	}
}

void display_time()
{
	while (run)
	{
		printf("%16u %16u\n", ts, te);
		usleep(10 * 1000);
	}
}

void set_speed()
{
	while (run)
	{
		printf("SPEED = %4d, Enter new speed:\n", speed);
		int tmp = 0;
		scanf("%d", &tmp);
		if (tmp >= 0 && tmp <= 1000)
		{
			speed = tmp;
			timers.tv_sec = 0;
			timers.tv_usec = 1000 * (1000 + speed);

			timere.tv_sec = 0;
			timere.tv_usec = 1000 * (1000 - speed);
			continue;
		}
		speed = 0;
		usleep(100 * 1000);
		run = 0;
	}
}

int main(int argc, char *argv[])
{
	pthread_t p_run;
	pthread_t p_set;
	pthread_t p_pri;
	pthread_create(&p_run, (const pthread_attr_t*) NULL, (void* (*)(void*)) &motor_run, NULL);
	pthread_create(&p_set, (const pthread_attr_t*) NULL, (void* (*)(void*)) &set_speed, NULL);
	pthread_create(&p_pri, (const pthread_attr_t*) NULL, (void* (*)(void*)) &display_time, NULL);
	pthread_join(p_set, NULL);
	pthread_join(p_run, NULL);

	return 0;
}
