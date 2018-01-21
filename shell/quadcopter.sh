#!/bin/bash

# /home/pi/work/quadcopter/bin/quadcopter.sh
#
#  Created on: May 24, 2016
#
# 四轴飞行控制器  Copyright (C) 2016  李德强
# /etc/init.d/
# sudo update-rc.d quadcopter defaults
# sudo update-rc.d -f quadcopter remove

export QUAD_HOME=/home/pi/work/quadcopter
export PATH=$QUAD_HOME/bin:$PATH

case $1 in
	start)
		
		quadcopter --fly > /dev/null &
		ps -ef | grep quadcopter | grep -v grep | awk '{print $2}' | xargs chrt -p 99 > /dev/null &
		#ps -ef | grep quadcopter | grep -v grep | awk '{print $2}' | xargs renice -19 > /dev/null &
		#raspivid -o $(date +"%Y-%m-%d_%H-%M-%S").h264 -t 1200000 -w 1280 -h 1024 &
		;;

	stop)
		ps -ef | grep "quadcopter --fly" | grep -v grep | awk '{print $2}' | xargs kill -9
		;;

	restart)
		ps -ef | grep "quadcopter --fly" | grep -v grep | awk '{print $2}' | xargs kill -9
		quadcopter --fly > /dev/null &
		ps -ef | grep quadcopter | grep -v grep | awk '{print $2}' | xargs chrt -p 99 > /dev/null &
		#ps -ef | grep quadcopter | grep -v grep | awk '{print $2}' | xargs renice -19 > /dev/null &
		#raspivid -o $(date +"%Y-%m-%d_%H-%M-%S").h264 -t 1200000 -w 1280 -h 1024 &
		;;

	*)
		echo "Usage: $0 (start|stop|restart)"
		;;

esac
