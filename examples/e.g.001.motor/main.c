#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <wiringPi.h>
#include <wiringSerial.h>

int port = 1;
int run = 1;
int speed = 0;

void motor_run()
{
	while (run)
	{
		digitalWrite(port, HIGH);
		usleep(1000 + speed);
		digitalWrite(port, LOW);
		usleep(1000 - speed);
	}
}

void set_speed()
{
	while (run)
	{
		printf("SPEED = %d, Enter new speed:\n", speed);
		int tmp = 0;
		scanf("%d", &tmp);
		if (tmp >= 0 && tmp <= 1000)
		{
			speed = tmp;
			continue;
		}
		run = 0;
	}
}

int main(int argc, char *argv[])
{
	wiringPiSetup();
	pinMode(port, OUTPUT);

	pthread_t p_run;
	pthread_t p_set;
	pthread_create(&p_run, (const pthread_attr_t*) NULL, (void* (*)(void*)) &motor_run, NULL);
	pthread_create(&p_set, (const pthread_attr_t*) NULL, (void* (*)(void*)) &set_speed, NULL);
	pthread_join(p_set, NULL);
	pthread_join(p_run, NULL);

	return 0;
}
