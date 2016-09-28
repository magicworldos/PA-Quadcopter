MOD_NAME = mpu6050

all:
	gcc -I../../include/ -I./ -lm -fPIC -shared -o lib$(MOD_NAME).so	\
		 mpu6050.c														\
		 i2cdev.c
		 
	mv lib$(MOD_NAME).so ../../lib/
clean:
	rm ../../lib/lib$(MOD_NAME).so