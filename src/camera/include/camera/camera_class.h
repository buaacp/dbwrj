/* ***** 头文件引入 ***** */
// 输入输出标准头文件
#include <iostream>
#include <iomanip>


class CAMERA
{
    public:
        /* 属性 */
        int   CAMERA_ID;
        //目标像素位置
        double pixel_x;
        double pixel_y;
        //物体位置估计
        double UAV_OBJ_X;
        double UAV_OBJ_Y;
        /*无人机位置*/
        double latitude;
        double longitude;
        double altitude;

        /* 邻居无人机信息 */
        std::vector<neighbor_camera_info> camera_neighbor;

        /*云台信息*/
        double angle_0;
        double angle_1;
        double angle_2;

        /*无人机姿态信息*/
        double UAV_x;
        double UAV_y;
        double UAV_z;
        double UAV_w;
        /* 方法 */
        // 无人机类构造函数
        CAMERA();
        void relative_altitude_info_callback(const std_msgs::Float64::ConstPtr &msg);
        void camera_detect_callback(const camera::pixel_msg &msg)
};
// 初始化
CAMERA::CAMERA(){

    CAMERA_ID = 0;

    pixel_x = 0;
    pixel_y = 0;

    UAV_OBJ_X = 0;
    UAV_OBJ_Y = 0;

    latitude = 0;
    longitude = 0;
    altitude = 0;

        /* 邻居无人机信息 */
    for (int i = 0; i < UAV_num; i++)
    {
        neighbor_camera_info neighbor;
        neighbor.UAV_OBJ_X = 0;
        neighbor.UAV_OBJ_Y = 0;
        neighbor.latitude = 0;
        neighbor.longitude = 0;
        neighbor.altitude = 0;

        camera_neighbor.push_back(neighbor);
    }

    /*云台信息*/
    angle_0 = 0;
    angle_1 = 0;
    angle_2 = 0;

    /*无人机姿态信息*/
    UAV_x = 0;
    UAV_y = 0;
    UAV_z = 0;
    UAV_w = 0;
}
void relative_altitude_info_callback(const std_msgs::Float64::ConstPtr &msg)
{
	altitude = (double)msg.data;
}
void camera_detect_callback(const camera::pixel_msg &msg)
{
    pixel_x=(double)msg.pixel_x;
    pixel_y=(double)msg.pixel_y;
}


