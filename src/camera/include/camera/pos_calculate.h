
// 邻居无人机相机信息
#pragma pack(1)
struct neighbor_camera_info
{
    int   UAV_ID ;
    //物体相对位置估计
    double UAV_OBJ_X ;
    double UAV_OBJ_Y ;
    //无人机信息
    double latitude;
    double longitude;
    double altitude;
};
#pragma pack()
