同步方式：
git add .
git commit -m "2025/4/23提交(实物1.1)"
git push -u origin master

username :zhenglihaqi
zlhq001218

仿真流程：
roslaunch le_arm iris_arm.launch
             等一会儿。。。
roslaunch test_fly test_control.launch 
roslaunch arm_control control_arm_sim.launch 
实物流程：
roslaunch arm_control real_test_condition.launch 
roslaunch arm_control control_arm_real.launch 


常用指令：
rostopic echo /joint_states 

