#!/bin/bash

path_install=/home/pi/work/quadcopter

if [ -n "$1" ]
then
{
	path_install=$1
}
fi

echo "[Install] "$path_install

mkdir -p $path_install/
mkdir -p $path_install/bin
mkdir -p $path_install/lib
mkdir -p $path_install/params

cp ./release/bin/quadcopter $path_install/bin
cp ./shell/* $path_install/bin
cp ./params/* $path_install/params
cp ./lib/*  $path_install/lib/

echo "export QUAD_HOME=$path_install" >> ~/.bash_profile
echo "export PATH=\$QUAD_HOME/bin:\$PATH" >> ~/.bash_profile

#echo "export QUAD_HOME=$path_install" >> /etc/profile
#echo "export PATH=\$QUAD_HOME/bin:\$PATH" >> /etc/profile

echo "Install finished."
echo "Please execute \"source /.bash_profile\" or reboot your computer."

exit 0