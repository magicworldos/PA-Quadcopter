MOD_NAME = mpu6050

all:
	gcc -I../../include/ -I./ -std=gnu11 -lm -fPIC -shared -o lib$(MOD_NAME).so	\
		 $(MOD_NAME).c														\
		 i2cdev.c
		 
	mv lib$(MOD_NAME).so ../../lib/
clean:
	rm ../../lib/lib$(MOD_NAME).so