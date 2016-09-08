#!/bin/bash

SH_PATH=$(dirname $0)
cd $SH_PATH/../

case $1 in
	start)
		
		release/bin/quadcopter --fly > /dev/null &
		;;

	stop)
		ps -ef | grep "release/bin/quadcopter --fly" | grep -v grep | awk '{print $2}' | xargs kill -9
		;;

	restart)
		ps -ef | grep "release/bin/quadcopter --fly" | grep -v grep | awk '{print $2}' | xargs kill -9
		release/bin/quadcopter --fly > /dev/null &
		;;

	*)
		echo "Usage: $0 (start|stop|restart)"
		;;

esac
