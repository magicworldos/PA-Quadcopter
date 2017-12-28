# Makefile
#
#  Created on: May 24, 2016
#
# 四轴飞行控制器  Copyright (C) 2016  李德强

#安装路径
PATH_INSTALL		= /home/lidq/work/quadcopter

################################################################################

#X型 _FLY_MODE_X_ 
#I型 _FLY_MODE_I_
FLY_MODE			= _FLY_MODE_X_
#保护最低速度
PROCTED_SPEED		= (100)
#电机最大速度
MAX_SPEED_RUN_MAX	= (1000)
#电机最小速度
MAX_SPEED_RUN_MIN	= (0)
#10ms 100Hz
ENG_TIMER 			= (10)
#文件名及路径最大长度
MAX_PATH_NAME		= (0x200)
#重力读取
MAX_ACC				= (32.0)
#角度限幅
MAX_PALSTANCE		= (30.0)
#i2c设备路径
I2C_DEV 			= \"/dev/i2c-1\"
#4个电机的GPIO引脚
PORT_MOTOR0			= (27)
PORT_MOTOR1			= (26)
PORT_MOTOR2			= (28)
PORT_MOTOR3			= (25)
#摇控器接收机的6个通道引脚
#俯仰
GPIO_FB				= (2)
#横滚
GPIO_LR				= (12)
#油门
GPIO_PW				= (3)
#预留
GPIO_MD				= (0)
#预留
GPIO_UD				= (13)
#俯仰横滚灵敏度
GPIO_DI				= (14)

###############################################################################

#工程
MOD_PROJECT			= quadcopter
#模块动态链接库
MOD_MODULES			= modlibs
MOD_MODULES_IO		= mode_io
MOD_MOTOR			= motor
MOD_PARAMSCTL		= paramsctl
MOD_CONTROLLER		= controller
MOD_MPU6050			= mpu6050
MOD_HCSR04			= hcsr04
MOD_FHEIGHT			= fheight
MOD_DISPLAY			= display
MOD_COMMAND			= command
MOD_IO_STM32		= io_stm32

#编译目录
MOD_MKDIR			= mkdir
#编译目录
RELEASE_PATH		= release
#头文件
MOD_INCLUDE			= -Iinclude
#编译选项
C_FLAGS				= -pthread -lm -ldl -lwiringPi -std=gnu11

default: pi

pi:	need_wiringpi $(MOD_MKDIR)	$(MOD_PROJECT)	$(MOD_MODULES)

io:	noneed_wiringpi $(MOD_MKDIR)	$(MOD_PROJECT)	$(MOD_MODULES_IO)

install:
	./shell/install.sh $(PATH_INSTALL)

defconfig:
	echo "#define $(FLY_MODE)" >> include/defconfig.h
	echo "#define PROCTED_SPEED	$(PROCTED_SPEED)" >> include/defconfig.h
	echo "#define MAX_SPEED_RUN_MAX	$(MAX_SPEED_RUN_MAX)" >> include/defconfig.h
	echo "#define MAX_SPEED_RUN_MIN	$(MAX_SPEED_RUN_MIN)" >> include/defconfig.h
	echo "#define ENG_TIMER	$(ENG_TIMER)" >> include/defconfig.h
	echo "#define MAX_PATH_NAME	$(MAX_PATH_NAME)" >> include/defconfig.h
	echo "#define MAX_ACC	$(MAX_ACC)" >> include/defconfig.h
	echo "#define MAX_PALSTANCE	$(MAX_PALSTANCE)" >> include/defconfig.h
	echo "#define I2C_DEV	$(I2C_DEV)" >> include/defconfig.h
	echo "#define PORT_MOTOR0	$(PORT_MOTOR0)" >> include/defconfig.h
	echo "#define PORT_MOTOR1	$(PORT_MOTOR1)" >> include/defconfig.h
	echo "#define PORT_MOTOR2	$(PORT_MOTOR2)" >> include/defconfig.h
	echo "#define PORT_MOTOR3	$(PORT_MOTOR3)" >> include/defconfig.h
	echo "#define GPIO_FB	$(GPIO_FB)" >> include/defconfig.h
	echo "#define GPIO_LR	$(GPIO_LR)" >> include/defconfig.h
	echo "#define GPIO_PW	$(GPIO_PW)" >> include/defconfig.h
	echo "#define GPIO_MD	$(GPIO_MD)" >> include/defconfig.h
	echo "#define GPIO_UD	$(GPIO_UD)" >> include/defconfig.h
	echo "#define GPIO_DI	$(GPIO_DI)" >> include/defconfig.h
	
need_wiringpi:
	echo "#ifndef _DEFCONFIG_H_" > include/defconfig.h
	echo "#define _DEFCONFIG_H_" >> include/defconfig.h
	echo "" >> include/defconfig.h
	echo "#define _NEED_WIRINGPI_" >> include/defconfig.h
	make defconfig
	echo "" >> include/defconfig.h
	echo "#endif" >> include/defconfig.h
	
noneed_wiringpi:
	echo "#ifndef _DEFCONFIG_H_" > include/defconfig.h
	echo "#define _DEFCONFIG_H_" >> include/defconfig.h
	echo "" >> include/defconfig.h
	make defconfig
	echo "" >> include/defconfig.h
	echo "#endif" >> include/defconfig.h

engine:	$(MOD_PROJECT)

$(MOD_PROJECT):
	gcc $(C_FLAGS) -o $(RELEASE_PATH)/bin/$(MOD_PROJECT) $(MOD_INCLUDE)			\
	main/main.c								\
	engine/engine.c							\
	engine/dlmod.c							\
	engine/emode.c							\
	engine/config.c							\
	util/list.c
	
#default option
$(MOD_MODULES):	$(MOD_PARAMSCTL)	$(MOD_MOTOR)	$(MOD_CONTROLLER)	$(MOD_MPU6050)		$(MOD_DISPLAY)

#io use stm32
$(MOD_MODULES_IO):	$(MOD_PARAMSCTL)	$(MOD_IO_STM32)	$(MOD_MPU6050)		$(MOD_DISPLAY)

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
	
$(MOD_IO_STM32):
	cd mods/io_stm32/		&& make

$(MOD_MKDIR):
	mkdir -p $(RELEASE_PATH)/bin/ lib/

clean:
	rm -rvf $(RELEASE_PATH)
	rm -rvf lib/*



#ctags
#ctags -R --c++-kinds=+p --fields=+iaS --extra=+q .
#
