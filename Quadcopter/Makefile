#工程
MOD_PROJECT			= quadcopter
#编译目录
MOD_MKDIR			= mkdir
#编译目录
RELEASE_PATH		= release
#头文件
MOD_INCLUDE			= -Iinclude
#编译选项
C_FLAGS				= -g -pthread -lm -lwiringPi

all:	$(MOD_MKDIR)	$(MOD_PROJECT)

run:	$(MOD_MKDIR)	$(MOD_PROJECT)

$(MOD_PROJECT):
	g++ $(C_FLAGS) -DDMP_FIFO_RATE=1 -o $(RELEASE_PATH)/bin/$(MOD_PROJECT) $(MOD_INCLUDE)			\
	mpu6050/MPU6050.cpp						\
	mpu6050/I2Cdev.cpp						\
	mpu6050/if_mpu6050.cpp					\
	main/main.cpp							\
	engine/engine.c							\
	engine/paramsctl.c						\
	engine/getch.c							\
	engine/driver.c

$(MOD_MKDIR):
	mkdir -p $(RELEASE_PATH)/bin/
	
clean:
	rm -rvf $(RELEASE_PATH)