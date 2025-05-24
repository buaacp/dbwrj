同步方式：
git add .
git commit -m "2025/5/23提交(2.0)"
git push -u origin master

username :zhenglihaqi

仿真流程：
roslaunch le_arm iris_arm.launch
等一会儿。。。
roslaunch test_fly test_control.launch 
roslaunch arm_control control_arm_sim.launch 
实物流程：
roslaunch arm_control real_test_condition.launch 
roslaunch arm_control control_arm_real.launch 

单独对机械臂仿真：
roslaunch le_arm le_arm_gazebo.launch
roslaunch test_fly get_ball_pos.launch 
roslaunch arm_control control_arm_single.launch 

单独控制飞机：
cd ~/PX4_Firmware
roslaunch px4 indoor1.launch
roslaunch test_fly test_control.launch 


常用指令：
rostopic echo /joint_states 

