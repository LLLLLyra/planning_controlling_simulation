#!/bin/sh
source devel/setup.sh
sleep 1

read -p "enable parking: " parking

if [ $parking -eq 0 ]
then
    read -p "enable lattice: " lattice
    read -p "FILE NAME: " file_name
    file="path_$file_name.txt"
    echo "the recorded path file is $file"
fi

roslaunch simulation launch.launch file:=$file_name enable_parking:=$parking enable_lattice:=$lattice &
rosrun simulation display.py
