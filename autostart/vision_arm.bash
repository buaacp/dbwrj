#!/bin/bash

# 启动新窗口运行任务程序
gnome-terminal --tab -- roslaunch camera cube_test.launch
sleep 2
gnome-terminal --tab -- roslaunch arm_control control_arm_real.launch 

