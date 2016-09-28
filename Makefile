# Makefile
#
#  Created on: May 24, 2016
#
# 四轴飞行控制器  Copyright (C) 2016  李德强

#工程
MOD_PROJECT			= quadcopter
#模块动态链接库
MOD_MODULES			= modlibs

#编译目录
MOD_MKDIR			= mkdir
#编译目录
RELEASE_PATH		= release
#头文件
MOD_INCLUDE			= -Iinclude
#编译选项
C_FLAGS				= -g -pthread -lm -ldl -lwiringPi -std=gnu11

all:	$(MOD_MKDIR)	$(MOD_PROJECT)	$(MOD_MODULES)

$(MOD_PROJECT):
	gcc $(C_FLAGS) -o $(RELEASE_PATH)/bin/$(MOD_PROJECT) $(MOD_INCLUDE)			\
	main/main.c								\
	engine/engine.c							\
	engine/dlmod.c							\
	engine/driver.c							\
	util/list.c
	
$(MOD_MODULES):
	cd mods/paramsctl/		&& make
	#cd mods/mpu6050/		&& make
	cd mods/display/		&& make
	#cd mods/command/		&& make

$(MOD_MKDIR):
	mkdir -p $(RELEASE_PATH)/bin/
	
clean:
	rm -rvf $(RELEASE_PATH)
	rm -rvf lib/*
