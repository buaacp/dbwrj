#!/bin/bash

# 启动新窗口运行任务程序
gnome-terminal --tab -- roslaunch le_arm le_arm_gazebo.launch
sleep 10
gnome-terminal --tab -- roslaunch test_fly get_ball_pos.launch
sleep 2
gnome-terminal --tab -- roslaunch arm_control control_arm_single.launch

# gnome-terminal --tab -- roslaunch subgroup_guarantee mission.launch UAV_ID:=2 if_simulation:=true
# sleep 2

# gnome-terminal --tab -- roslaunch subgroup_guarantee mission.launch UAV_ID:=3 if_simulation:=true
# sleep 2

# gnome-terminal --tab -- roslaunch subgroup_guarantee mission.launch UAV_ID:=4 if_simulation:=true
# sleep 2

# gnome-terminal --tab -- roslaunch subgroup_guarantee mission.launch UAV_ID:=5 if_simulation:=true
# sleep 2

# gnome-terminal --tab -- roslaunch subgroup_guarantee mission.launch UAV_ID:=6 if_simulation:=true
# sleep 2

# gnome-terminal --tab -- roslaunch subgroup_guarantee mission.launch UAV_ID:=7 if_simulation:=true
# sleep 2

# gnome-terminal --tab -- roslaunch subgroup_guarantee mission.launch UAV_ID:=8 if_simulation:=true
# sleep 2
