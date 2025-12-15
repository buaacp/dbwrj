#!/bin/bash

# 启动新窗口运行任务程序
gnome-terminal --tab -- roslaunch le_arm le_arm_gazebo.launch
sleep 10
gnome-terminal --tab -- roslaunch test_fly get_ball_pos.launch
sleep 2
gnome-terminal --tab -- roslaunch arm_control control_arm_single.launch

