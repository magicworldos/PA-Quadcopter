# Makefile
#
#  Created on: May 24, 2016
#
# 四轴飞行控制器  Copyright (C) 2016  李德强

#工程
MOD_PROJECT			= quadcopter
#编译目录
MOD_MKDIR			= mkdir
#编译目录
RELEASE_PATH		= release
#头文件
MOD_INCLUDE			= -Iinclude
#编译选项
C_FLAGS				= -g -pthread -lm -lwiringPi -std=gnu99

all:	$(MOD_MKDIR)	$(MOD_PROJECT)

run:	$(MOD_MKDIR)	$(MOD_PROJECT)

$(MOD_PROJECT):
	gcc $(C_FLAGS) -o $(RELEASE_PATH)/bin/$(MOD_PROJECT) $(MOD_INCLUDE)			\
	main/main.c								\
	engine/engine.c							\
	engine/paramsctl.c						\
	engine/getch.c							\
	mpu6050/i2cdev.c						\
	mpu6050/mpu6050.c						\
	engine/driver.c

$(MOD_MKDIR):
	mkdir -p $(RELEASE_PATH)/bin/
	
clean:
	rm -rvf $(RELEASE_PATH)