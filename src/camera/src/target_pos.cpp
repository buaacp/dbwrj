// 1.包含头文件
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/Imu.h>

// 通用数据头文件
#include <common_defined.h>
#include <mavros_h.h>


// 相机
#include <pos_calculate.h>
#include <camera_class.h>
#include <camera/pixel_msg.h>

/* 无人机相对高度信息回调函数 */
void relative_altitude_info_callback(const std_msgs::Float64::ConstPtr &msg)
{
	cam.relative_altitude_info_update(msg);
}
void camera_detect_callback(const camera::pixel_msg &msg)
{
    cam.camera_detect_callback(msg);
}
void attitude_info_callback(const std_msgs::IMU &msg)
{
    cam.attitude_info_callback(msg);
}
/*无人机姿态回调函数*/
int main(int argc, char **argv)
{
    /* ----- 节点初始化 ----- */
    ros::init(argc, argv, "camera");
    ros::NodeHandle nh_camera;
    ros::Rate rate(ROS_RATE);

    ros::Subscriber relative_altitude_sub = nh_camera.subscribe("/UAV0/mavros/global_position/rel_alt", 10, relative_altitude_info_callback); // 获取无人机高度信息订阅器定义
    ros::Subscriber camera_detect_sub     = nh_camera.subscribe("/detect",10,camera_detect_callback);
}