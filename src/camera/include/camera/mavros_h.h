/* ***** 头文件引入 ***** */
// mavros相关头文件 
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointReached.h>
#include <mavros_msgs/WaypointSetCurrent.h>
#include <mavros_msgs/CommandHome.h>
#include <mavros_msgs/CommandInt.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/VehicleInfo.h>
#include <mavros_msgs/VehicleInfoGet.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/VFR_HUD.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/PoseStamped.h>

// 飞行模式相关头文件
#include <string>

// gps信息相关头文件
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Point.h>

// 速度消息相关头文件
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

// 相对高度相关头文件
#include <std_msgs/Float64.h>


/* ***** mavros相关变量声明及定义 ***** */
// mavros消息定义
mavros_msgs::Waypoint             wp_msg;                     // mavros航点信息定义
mavros_msgs::GlobalPositionTarget global_position_target_msg; // setpoint指令对应消息定义

// mavros服务定义
mavros_msgs::WaypointClear      srv_wp_clear;
mavros_msgs::WaypointPush       srv_wp_push;
mavros_msgs::WaypointSetCurrent srv_wp_set_current;
mavros_msgs::SetMode            srv_setMode;
mavros_msgs::CommandBool        srv_arming;
mavros_msgs::CommandHome        srv_set_home;
mavros_msgs::VehicleInfoGet     srv_vehicle_info;
mavros_msgs::CommandInt         srv_command_int;
mavros_msgs::CommandLong        srv_command_long;

// mavros消息发布器声明
// 下述发布器的作用是发布话题名为/mavros/setpoint_raw/global的mavros_msgs::GlobalPositionTarget消息
// mavros结点接收到此消息,通过set_position_target_global_int指令为飞控下达控制指令
// 在mavros_msgs::GlobalPositionTarget消息中通过合理设置mask选择想要控制的量
ros::Publisher setpoint_raw_pub;

// mavros消息订阅器声明
ros::Subscriber state_substate_sub;
ros::Subscriber wp_reached_sub;
ros::Subscriber gps_information_sub;
ros::Subscriber vel_information_sub;
ros::Subscriber relative_altitude_sub;
ros::Subscriber compass_hdg_sub;
ros::Subscriber vfr_hud_sub;

// mavros客户端声明
ros::ServiceClient wp_clear_cl;
ros::ServiceClient wp_push_cl;
ros::ServiceClient wp_set_current_cl;
ros::ServiceClient set_mode_cl;
ros::ServiceClient arming_cl;     
ros::ServiceClient set_home_cl;
ros::ServiceClient vehicle_info_cl;
ros::ServiceClient command_int_cl;
ros::ServiceClient command_long_cl;


/* ***** mavros相关函数 ***** */
// 1.wp_clear函数
bool wp_clear_function(ros::ServiceClient wp_clear_cl, mavros_msgs::WaypointClear srv_wp_clear)
{
    // waypoint clear首次尝试
    if(wp_clear_cl.call(srv_wp_clear) == true)
    {
        if(srv_wp_clear.response.success == true)
        {
            // wp_clear成功
            // std::cout << "[Waypoint process] Waypoint clear successfully!" << std::endl;
            return srv_wp_clear.response.success;
        }
    }

    // 迭代尝试直至失败
    for(int i = 0; i < 5; i++)
    {
        if(wp_clear_cl.call(srv_wp_clear) != true)
        {
            std::cout << "[Waypoint process] Waiting for waypoint clear... " << std::endl;
        }
        else
        {
            if(srv_wp_clear.response.success == true)
            {
                // wp_clear成功
                // std::cout << "[Waypoint process] Waypoint clear successfully!" << std::endl;
                return srv_wp_clear.response.success;
            }
        }
    }
    ROS_ERROR("Failed wp clear");
    return false;
} 


// 2.wp_push相关函数
// -- 2.1 wp_msg生成函数
mavros_msgs::WaypointPush wp_msg_function(mavros_msgs::WaypointPush srv_wp_push, int command, bool is_current, bool autocontinue, int frame,float param1, float param2, float param3, float param4, double x_lat, double y_long, double z_alt)
{
    mavros_msgs::Waypoint wp_msg;

    // 确定航点坐标系
    wp_msg.frame = frame;

    // 此command对应航点任务，可从Mavink官网中的MAV_CMD_NAV中查看各条指令
    wp_msg.command = command;
    
    // 航点参数
    wp_msg.is_current   = is_current;
    wp_msg.autocontinue = autocontinue;
    wp_msg.param1       = param1;
    wp_msg.param2       = param2;
    wp_msg.param3       = param3;
    wp_msg.param4       = param4;
    wp_msg.x_lat        = x_lat;
    wp_msg.y_long       = y_long;
    wp_msg.z_alt        = z_alt;

    srv_wp_push.request.waypoints.push_back(wp_msg);
    return(srv_wp_push);
}


// -- 2.2 wp_push函数
bool wp_push_funciton(ros::ServiceClient wp_push_cl, mavros_msgs::WaypointPush srv_wp_push)
{
    // waypoint push首次尝试
    if(wp_push_cl.call(srv_wp_push) == true)
    {
        if(srv_wp_push.response.success == true)
        {
            // wp_push成功
            // std::cout << "[Waypoint process] Waypoint push successfully!" << std::endl;
            return srv_wp_push.response.success;
        }
    }

    // 迭代尝试直至失败
    for(int i = 0; i < 5; i++)
    {
        if(wp_push_cl.call(srv_wp_push) != true)
        {
            std::cout << "[Waypoint process] Waiting for waypoint push... " << std::endl;
        }
        else
        {
            if(srv_wp_push.response.success == true)
            {
                // wp_push成功
                // std::cout << "[Waypoint process] Waypoint push successfully!" << std::endl;
                return srv_wp_push.response.success;
            }
        }
    }
    ROS_ERROR("Failed wp push");
    return false;
}


// 3.set_mode函数
bool set_mode_funciton(ros::ServiceClient set_mode_cl, mavros_msgs::SetMode srv_setMode, const char* mode)
{
    // 服务请求参数赋值
    srv_setMode.request.base_mode = 0;	
	srv_setMode.request.custom_mode = mode;

    // set_mode首次尝试
    if(set_mode_cl.call(srv_setMode) == true)
	{
        std::cout << "[Mode set] UAV mode is set as: " << mode << "." << std::endl;
        return srv_setMode.response.mode_sent;
    }

    // 迭代尝试直至失败
    for(int i = 0; i < 5; i++)
    {
        if(set_mode_cl.call(srv_setMode) != true)
        {
            std::cout << "[Setmode process] Waiting for mode set... " << std::endl;
        }
        else
        {
            std::cout << "[Mode set] UAV mode is set as: " << mode << "." << std::endl;
            return srv_setMode.response.mode_sent;
        }
    }
    ROS_ERROR("Failed set mode");
    return false;
}


// 4.arm/disarm函数
bool arm_disarm_funciton(ros::ServiceClient arming_cl, mavros_msgs::CommandBool srv_arming, bool arm_or_disarm) 
{
    // arm_or_disarm - 1 for arm, 0 for disarm
    srv_arming.request.value = arm_or_disarm;

    bool if_arming_result = arming_cl.call(srv_arming);
    if(if_arming_result == true)
        std::cout << "[arming call successfully!]" << std::endl;
    else
        std::cout << "[arming call failed!]" << std::endl;

    if(if_arming_result)
	{
        // 成功发送解锁/锁定指令
        if(srv_arming.response.success == true)
        {
            if(arm_or_disarm == true)
            {
                std::cout << "[Arm/disarm] UAV is armed!" << std::endl;
                std::cout << "[Takeoff process] Ready to takeoff!" << std::endl; // 默认整个任务仅有一个解锁环节,即:解锁起飞
            }
            else
            {
                std::cout << "[Arm/disarm] UAV is disarmed!" << std::endl;
            }
        }
        else
        {
            ROS_ERROR("Failed send arm/disarm command.");
        }
        return srv_arming.response.success;
    }
	else
	{
        ROS_ERROR("Failed call arming or disarming service.");
        return false;
    }
}


// 5.set_current函数
bool set_current_funciton(ros::ServiceClient wp_set_current_cl, mavros_msgs::WaypointSetCurrent srv_wp_set_current, int wp_seq_to_set)
{
    srv_wp_set_current.request.wp_seq = wp_seq_to_set;

    // waypoint set_current首次尝试
    if(wp_set_current_cl.call(srv_wp_set_current) == true)
	{
        // set_current成功
        // std::cout << "[Waypoint current set] Current waypoint is set as:" << wp_seq_to_set << "." << std::endl;
        return srv_wp_set_current.response.success;
    }

    // 迭代尝试直至失败
    for(int i = 0; i < 5; i++)
    {
        if(wp_set_current_cl.call(srv_wp_set_current) != true)
        {
            std::cout << "[Waypoint process] Waiting for waypoint set current... " << std::endl;
        }
        else
        {
        // set_current成功
        // std::cout << "[Waypoint current set] Current waypoint is set as:" << wp_seq_to_set << "." << std::endl;
        return srv_wp_set_current.response.success;
        }
    }
    ROS_ERROR("Failed wp set current");
    return false;
}


// 6.command_int函数
bool command_int_funciton(ros::ServiceClient command_int_cl, mavros_msgs::CommandInt srv_command_int, int frame, int command, bool is_current, bool autocontinue, float param1, float param2, float param3, float param4, double x_lat, double y_long, double z_alt )
{
    // MAV_CMD_DO_REPOSITION (192)
    // Param (:Label)	Description	Values	Units
    // 1: Speed	Ground  speed, less than 0 (-1) for default	min: -1	m/s
    // 2: Bitmask	    Bitmask of option flags.	MAV_DO_REPOSITION_FLAGS	
    // 3: Radius	    Loiter radius for planes. Positive values only, direction is controlled by Yaw value. A value of zero or NaN is ignored.		m
    // 4: Yaw	        Yaw heading. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.). For planes indicates loiter direction (0: clockwise, 1: counter clockwise)		deg
    // 5: Latitude	    Latitude		
    // 6: Longitude	    Longitude		
    // 7: Altitude	    Altitude m

    // MAV_CMD_DO_CHANGE_SPEED (178)
    // [Command] Change speed and/or throttle set points. The value persists until it is overridden or there is a mode change.
    // Param (:Label)	Description	Values	Units
    // 1: Speed Type	Speed type (0=Airspeed, 1=Ground Speed, 2=Climb Speed, 3=Descent Speed)	min:0 max:3 increment:1	
    // 2: Speed	Speed (-1 indicates no change, -2 indicates return to default vehicle speed)	min: -2	m/s
    // 3: Throttle	Throttle (-1 indicates no change, -2 indicates return to default vehicle throttle value)	min: -2	%
    // 4: Reserved (set to 0)		
    // 5: Reserved (set to 0)		
    // 6: Reserved (set to 0)		
    // 7: Reserved (set to 0)

    srv_command_int.request.broadcast    = 1;
    srv_command_int.request.frame        = frame; // frame0: Global
    srv_command_int.request.command      = command;
    srv_command_int.request.current      = is_current;
    srv_command_int.request.autocontinue = autocontinue;
    srv_command_int.request.param1       = param1;
    srv_command_int.request.param2       = param2;
    srv_command_int.request.param3       = param3;
    srv_command_int.request.param4       = param4;
    srv_command_int.request.x            = x_lat  * pow(10,7); // 注:经纬度为整数(x10^7)
    srv_command_int.request.y            = y_long * pow(10,7); 
    srv_command_int.request.z            = z_alt;

    if(command_int_cl.call(srv_command_int))
	{
        // std::cout << "[Command int] Command:" << command << "." << std::endl;
        return srv_command_int.response.success;
    }
	else
	{
        ROS_ERROR("Command int service.");
        return false;
    }
}


// 7.command_long函数
bool command_long_funciton(ros::ServiceClient command_long_cl, mavros_msgs::CommandLong srv_command_long, int command, float param1, float param2, float param3, float param4, float param5, float param6, float param7)
{
    // MAV_CMD_SET_MESSAGE_INTERVAL (511)
    // [Command] Set the interval between messages for a particular MAVLink message ID. This interface replaces REQUEST_DATA_STREAM.

    // Param (:Label)	  Description	                                                           Values	                                                                      Units
    // 1: Message ID	  The MAVLink message ID	                                               min:0 max:16777215 increment:1	                                              /
    // 2: Interval	      The interval between two messages. -1: disable.                          0: request default rate (which may be zero).	min: -1 increment:1	              us
    // 7: Response Target Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.  /

    // MAV_CMD_NAV_TAKEOFF (22)
    // [Command] Takeoff from ground / hand. Vehicles that support multiple takeoff modes (e.g. VTOL quadplane) should take off using the currently configured mode.

    // Param (:Label)  Description	                                                                                       Units
    // 1: Pitch        Minimum pitch (if airspeed sensor present), desired pitch without sensor	                           deg
    // 2: \            Empty
    // 3: \            Empty	
    // 4: Yaw	       Yaw angle (if magnetometer present), ignored without magnetometer. 
    //                 NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.). deg
    // 5: Latitude	   Latitude	
    // 6: Longitude	   Longitude	
    // 7: Altitude	   Altitude

    srv_command_long.request.broadcast    = 1;
    srv_command_long.request.command      = command;
    srv_command_long.request.confirmation = 1;
    srv_command_long.request.param1       = param1;
    srv_command_long.request.param2       = param2;
    srv_command_long.request.param3       = param3;
    srv_command_long.request.param4       = param4;
    srv_command_long.request.param5       = param5;
    srv_command_long.request.param6       = param6;
    srv_command_long.request.param7       = param7;

    if(command_long_cl.call(srv_command_long))
	{
        // if(command == 511)
        // {
        //     std::cout << "[Command long] MAV_CMD_SET_MESSAGE_INTERVAL." << std::endl;
        // }
        // else if(command == 22)
        // {
        //     std::cout << "[Command long] MAV_CMD_NAV_TAKEOFF." << std::endl;
        // }
        return srv_command_long.response.success;
    }
	else
	{
        ROS_ERROR("Command long service failed.");
        return false;
    }
}

// 8.GlobalPositionTarget消息设置
mavros_msgs::GlobalPositionTarget GlobalPositionTarget_msg_set_function(std::string control_mode, double latitude, double longitude, float altitude, geometry_msgs::Vector3 velocity, float heading_radians)
{
    mavros_msgs::GlobalPositionTarget goal_GlobalPositionTarget;
    
    // 参考系设置 - 选择相对高度
	goal_GlobalPositionTarget.coordinate_frame = goal_GlobalPositionTarget.FRAME_GLOBAL_REL_ALT;

    // 控制模式选择及期望控制目标设置
    if(control_mode == "Position")
    {
        goal_GlobalPositionTarget.type_mask = goal_GlobalPositionTarget.IGNORE_VX | goal_GlobalPositionTarget.IGNORE_VY | goal_GlobalPositionTarget.IGNORE_VZ | goal_GlobalPositionTarget.IGNORE_AFX | goal_GlobalPositionTarget.IGNORE_AFY | goal_GlobalPositionTarget.IGNORE_AFZ | goal_GlobalPositionTarget.IGNORE_YAW | goal_GlobalPositionTarget.IGNORE_YAW_RATE;

        goal_GlobalPositionTarget.latitude  = latitude;
        goal_GlobalPositionTarget.longitude = longitude;
        goal_GlobalPositionTarget.altitude  = altitude;
    }
    else if(control_mode == "Position+Velocity")
    {
        goal_GlobalPositionTarget.type_mask = goal_GlobalPositionTarget.IGNORE_AFX | goal_GlobalPositionTarget.IGNORE_AFY | goal_GlobalPositionTarget.IGNORE_AFZ | goal_GlobalPositionTarget.IGNORE_YAW | goal_GlobalPositionTarget.IGNORE_YAW_RATE;
        
        goal_GlobalPositionTarget.latitude  = latitude;
        goal_GlobalPositionTarget.longitude = longitude;
        goal_GlobalPositionTarget.altitude  = altitude;
        goal_GlobalPositionTarget.velocity  = velocity;
    }
    else if(control_mode == "Velocity")
    {
        goal_GlobalPositionTarget.type_mask = goal_GlobalPositionTarget.IGNORE_LATITUDE | goal_GlobalPositionTarget.IGNORE_LONGITUDE | goal_GlobalPositionTarget.IGNORE_AFX | goal_GlobalPositionTarget.IGNORE_AFY | goal_GlobalPositionTarget.IGNORE_AFZ | goal_GlobalPositionTarget.IGNORE_YAW | goal_GlobalPositionTarget.IGNORE_YAW_RATE;

        goal_GlobalPositionTarget.altitude  = altitude;
        goal_GlobalPositionTarget.velocity  = velocity;
    }
    else if(control_mode == "Velocity+Heading")
    {
        goal_GlobalPositionTarget.type_mask = goal_GlobalPositionTarget.IGNORE_LATITUDE | goal_GlobalPositionTarget.IGNORE_LONGITUDE | goal_GlobalPositionTarget.IGNORE_AFX | goal_GlobalPositionTarget.IGNORE_AFY | goal_GlobalPositionTarget.IGNORE_AFZ | goal_GlobalPositionTarget.IGNORE_YAW_RATE;
        
        goal_GlobalPositionTarget.altitude  = altitude;
        goal_GlobalPositionTarget.velocity  = velocity;
        goal_GlobalPositionTarget.yaw       = heading_radians;
    }

    return goal_GlobalPositionTarget;
}


// 9.setpoint_raw_attitude 消息设置
mavros_msgs::AttitudeTarget setpoint_raw_attitude_msg(double roll,double pitch,double yaw,double thrust){
    // 将欧拉角转换为弧度
    mavros_msgs::AttitudeTarget setpoint_raw_attitude;
    double roll_rad = roll * M_PI / 180.0;
    double pitch_rad = pitch * M_PI / 180.0;
    double yaw_rad = yaw * M_PI / 180.0;

    // 计算相应的旋转四元数
    double cy = cos(yaw_rad * 0.5);
    double sy = sin(yaw_rad * 0.5);
    double cp = cos(pitch_rad * 0.5);
    double sp = sin(pitch_rad * 0.5);
    double cr = cos(roll_rad * 0.5);
    double sr = sin(roll_rad * 0.5);

    double qw = cy * cp * cr + sy * sp * sr;
    double qx = cy * cp * sr - sy * sp * cr;
    double qy = sy * cp * sr + cy * sp * cr;
    double qz = sy * cp * cr - cy * sp * sr;

    setpoint_raw_attitude.header.stamp = ros::Time::now();
    //setpoint_raw_attitude.header.seq = 1; //消息戳
    setpoint_raw_attitude.type_mask = setpoint_raw_attitude.IGNORE_PITCH_RATE | setpoint_raw_attitude.IGNORE_ROLL_RATE | setpoint_raw_attitude.IGNORE_YAW_RATE;
    setpoint_raw_attitude.body_rate.x=1;
    setpoint_raw_attitude.body_rate.y=1;
    setpoint_raw_attitude.body_rate.z=1;
    setpoint_raw_attitude.thrust=thrust;
    setpoint_raw_attitude.orientation.x=qx;
    setpoint_raw_attitude.orientation.y=qy;
    setpoint_raw_attitude.orientation.z=qz;
    setpoint_raw_attitude.orientation.w=qw;

    std::cout<<setpoint_raw_attitude.orientation<<std::endl;

    return setpoint_raw_attitude;

}

// <geometry_msgs/PoseStamped.h>

geometry_msgs::PoseStamped setpoint_attitude_msg(double roll,double pitch,double yaw,double thrust){
    // 将欧拉角转换为弧度
    geometry_msgs::PoseStamped cmd_att;
    double roll_rad = roll * M_PI / 180.0;
    double pitch_rad = pitch * M_PI / 180.0;
    double yaw_rad = yaw * M_PI / 180.0;

    // 计算相应的旋转四元数
    double cy = cos(yaw_rad * 0.5);
    double sy = sin(yaw_rad * 0.5);
    double cp = cos(pitch_rad * 0.5);
    double sp = sin(pitch_rad * 0.5);
    double cr = cos(roll_rad * 0.5);
    double sr = sin(roll_rad * 0.5);

    double qw = cy * cp * cr + sy * sp * sr;
    double qx = cy * cp * sr - sy * sp * cr;
    double qy = sy * cp * sr + cy * sp * cr;
    double qz = sy * cp * cr - cy * sp * sr;

    cmd_att.header.stamp = ros::Time::now();
    cmd_att.header.seq = 0; //Ignore row rate
    cmd_att.pose.position.x = 0.0;//0.001*some_object.position_x;
    cmd_att.pose.position.y = 0.0;//0.001*some_object.position_y;
    cmd_att.pose.position.z = 0.0;//0.001*some_object.position_z;

    cmd_att.pose.orientation.x = qx;
    cmd_att.pose.orientation.y = qy;
    cmd_att.pose.orientation.z = qz;
    cmd_att.pose.orientation.w = qw;

    std::cout<<cmd_att.pose.orientation<<std::endl;

    return cmd_att;

}