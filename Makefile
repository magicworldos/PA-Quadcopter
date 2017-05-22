# Makefile
#
#  Created on: May 24, 2016
#
# 四轴飞行控制器  Copyright (C) 2016  李德强

#工程
MOD_PROJECT			= quadcopter

#模块动态链接库
MOD_MODULES			= modlibs
MOD_MOTOR			= motor
MOD_PARAMSCTL		= paramsctl
MOD_CONTROLLER		= controller
MOD_MPU6050			= mpu6050
MOD_HCSR04			= hcsr04
MOD_FHEIGHT			= fheight
MOD_DISPLAY			= display
MOD_COMMAND			= command

#编译目录
MOD_MKDIR			= mkdir
#编译目录
RELEASE_PATH		= release
#头文件
MOD_INCLUDE			= -Iinclude
#编译选项
C_FLAGS				= -pthread -lm -ldl -lwiringPi -std=gnu11

all:	$(MOD_MKDIR)	$(MOD_PROJECT)	$(MOD_MODULES)

engine:	$(MOD_PROJECT)

$(MOD_PROJECT):
	gcc $(C_FLAGS) -o $(RELEASE_PATH)/bin/$(MOD_PROJECT) $(MOD_INCLUDE)			\
	main/main.c								\
	engine/engine.c							\
	engine/dlmod.c							\
	engine/emode.c							\
	util/list.c
	
$(MOD_MODULES):	$(MOD_MOTOR)	$(MOD_PARAMSCTL)	$(MOD_CONTROLLER)	$(MOD_MPU6050)	$(MOD_HCSR04)	$(MOD_FHEIGHT)	$(MOD_DISPLAY)

$(MOD_MOTOR):
	cd mods/motor/			&& make

$(MOD_PARAMSCTL):
	cd mods/paramsctl/		&& make

$(MOD_CONTROLLER):
	cd mods/controller/		&& make

$(MOD_MPU6050):
	cd mods/mpu6050/		&& make
	
$(MOD_HCSR04):
	cd mods/hcsr04/			&& make
	
$(MOD_FHEIGHT):
	cd mods/fheight/		&& make

$(MOD_DISPLAY):
	cd mods/display/		&& make

$(MOD_COMMAND):
	cd mods/command/		&& make

$(MOD_MKDIR):
	mkdir -p $(RELEASE_PATH)/bin/ lib/

clean:
	rm -rvf $(RELEASE_PATH)
	rm -rvf lib/*



#ctags
#ctags -R --c++-kinds=+p --fields=+iaS --extra=+q .
#
