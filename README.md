同步方式：
git add .
git commit -m "2025/12/17提交(3.2)"
git push -u origin master

username :zhenglihaqi

仿真流程：
roslaunch le_arm iris_arm.launch
等一会儿。。。
roslaunch test_fly test_control.launch 
roslaunch arm_control control_arm_sim.launch 
实物流程(目标点为gazebo提供的虚拟点)：
roslaunch arm_control real_test_condition.launch 
roslaunch arm_control control_arm_real.launch 

单独对机械臂仿真：
roslaunch le_arm le_arm_gazebo.launch
roslaunch test_fly get_ball_pos.launch 
roslaunch arm_control control_arm_single.launch 

加入视觉定位后的机械臂抓取实验：
roslaunch camera cube_test.launch
roslaunch arm_control control_arm_real.launch 

单独控制飞机：
roslaunch px4 indoor1.launch
roslaunch test_fly test_control.launch 

单双目相机视觉识别：
roslaunch camera detect_test.launch
双目相机的识别+定位
roslaunch camera cube_test.launch

手眼标定
roslaunch camera cube_test.launch
roslaunch arm_control arm_vision_biaoding.launch 

常用指令：
rostopic echo /joint_states 



