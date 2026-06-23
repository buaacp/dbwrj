#! /usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import math
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode, ParamSet
from geometry_msgs.msg import PoseStamped, Pose, TwistStamped,Twist
from std_msgs.msg import String
from pyquaternion import Quaternion
from std_msgs.msg import Int32
import sys

class Communication:

    def __init__(self, vehicle_type, vehicle_id):
        
        self.vehicle_type = vehicle_type
        self.vehicle_id = vehicle_id
        self.vision_pose = None
        self.current_position = None
        self.has_local_position = False
        self.current_velocity = None  # 添加当前速度
        self.current_yaw = 0
        self.hover_flag = 0
        self.coordinate_frame = 1
        self.target_motion = PositionTarget()
        self.target_motion.coordinate_frame = self.coordinate_frame
        self.arm_state = False
        self.mavros_state = State()
        self.motion_type = 0
        self.flight_mode = None
        self.mission = None
        self.last_cmd = None
        self.rate = rospy.Rate(20)
        self.target_pose = Pose()
        self.target_poses = {
            'ball': Pose(),
            'bulb': Pose(),
            'socket': Pose(),
        }
        self.default_pose = Pose()
        self.default_pose.position.x = 0
        self.default_pose.position.y = 0
        self.default_pose.position.z = 2
        self.mission_state = 0
        self.target_topic = rospy.get_param('~target_topic', '/weightless_ball/pose')
        self.ball_target_topic = rospy.get_param('~ball_target_topic', self.target_topic)
        self.bulb_target_topic = rospy.get_param('~bulb_target_topic', '/light_bulb/body_pose')
        self.socket_target_topic = rospy.get_param('~socket_target_topic', '/light_bulb_fixture/socket_pose')
            
        '''
        ros subscribers
        '''
        self.vision_pose_sub = rospy.Subscriber(self.vehicle_type+'_'+self.vehicle_id+"/mavros/vision_pose/pose", PoseStamped, self.vision_pose_callback,queue_size=1)
        self.state_sub = rospy.Subscriber(self.vehicle_type+'_'+self.vehicle_id+"/mavros/state", State, self.state_callback, queue_size=1)
        self.local_pose_sub = rospy.Subscriber(self.vehicle_type+'_'+self.vehicle_id+"/mavros/local_position/pose", PoseStamped, self.local_pose_callback,queue_size=1)
        self.local_vel_sub = rospy.Subscriber(self.vehicle_type+'_'+self.vehicle_id+"/mavros/local_position/velocity_local", TwistStamped, self.local_velocity_callback, queue_size=1)  # 订阅速度信息
        self.cmd_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd",String,self.cmd_callback,queue_size=3)
        self.cmd_pose_flu_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd_pose_flu", Pose, self.cmd_pose_flu_callback,queue_size=1)
        self.cmd_pose_enu_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd_pose_enu", Pose, self.cmd_pose_enu_callback,queue_size=1)
        self.cmd_vel_flu_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd_vel_flu", Twist, self.cmd_vel_flu_callback,queue_size=1)
        self.cmd_vel_enu_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd_vel_enu", Twist, self.cmd_vel_enu_callback,queue_size=1)
        self.cmd_accel_flu_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd_accel_flu", Twist, self.cmd_accel_flu_callback,queue_size=1)
        self.cmd_accel_enu_sub = rospy.Subscriber("/xtdrone/"+self.vehicle_type+'_'+self.vehicle_id+"/cmd_accel_enu", Twist, self.cmd_accel_enu_callback,queue_size=1)
        self.ball_target_sub = rospy.Subscriber(self.ball_target_topic, PoseStamped, self.target_pos_callback, callback_args='ball')
        self.bulb_target_sub = rospy.Subscriber(self.bulb_target_topic, PoseStamped, self.target_pos_callback, callback_args='bulb')
        self.socket_target_sub = rospy.Subscriber(self.socket_target_topic, PoseStamped, self.target_pos_callback, callback_args='socket')
        self.mission_state_sub = rospy.Subscriber("/mission_state",Int32,self.mission_state_callback)
        ''' 
        ros publishers
        '''
        self.target_motion_pub = rospy.Publisher(self.vehicle_type+'_'+self.vehicle_id+"/mavros/setpoint_raw/local", PositionTarget, queue_size=1)

        '''
        ros services
        '''
        self.armService = rospy.ServiceProxy(self.vehicle_type+'_'+self.vehicle_id+"/mavros/cmd/arming", CommandBool)
        self.flightModeService = rospy.ServiceProxy(self.vehicle_type+'_'+self.vehicle_id+"/mavros/set_mode", SetMode)

        print(self.vehicle_type+'_'+self.vehicle_id+": "+"communication initialized")

    def finite_values(self, *values):
        try:
            return all(math.isfinite(float(value)) for value in values)
        except (TypeError, ValueError):
            return False

    def finite_position(self, position):
        return position is not None and self.finite_values(position.x, position.y, position.z)

    def finite_twist(self, twist):
        return twist is not None and self.finite_values(
            twist.linear.x,
            twist.linear.y,
            twist.linear.z,
            twist.angular.z,
        )

    def set_safe_zero_velocity_target(self):
        self.coordinate_frame = 1
        self.motion_type = 1
        yaw = self.current_yaw if math.isfinite(float(self.current_yaw)) else 0.0
        self.target_motion = self.construct_target(vx=0.0, vy=0.0, vz=0.0, yaw=yaw)

    def target_motion_is_finite(self, target):
        return self.finite_values(
            target.position.x,
            target.position.y,
            target.position.z,
            target.velocity.x,
            target.velocity.y,
            target.velocity.z,
            target.acceleration_or_force.x,
            target.acceleration_or_force.y,
            target.acceleration_or_force.z,
            target.yaw,
            target.yaw_rate,
        )

    def start(self):
        '''
        main ROS thread
        '''
        while not rospy.is_shutdown():
            if not self.target_motion_is_finite(self.target_motion):
                rospy.logerr_throttle(
                    1.0,
                    "%s_%s: 拒绝发布 NaN/Inf setpoint，改发 ENU 零速度",
                    self.vehicle_type,
                    self.vehicle_id,
                )
                self.set_safe_zero_velocity_target()
            self.target_motion_pub.publish(self.target_motion)
            self.rate.sleep()

    def mission_state_callback(self, msg):
        self.mission_state = msg.data

    def state_callback(self, msg):
        self.mavros_state = msg

    def local_pose_callback(self, msg):
        if not self.finite_position(msg.pose.position):
            rospy.logwarn_throttle(
                1.0,
                "%s_%s: local_position 含 NaN/Inf，忽略该帧",
                self.vehicle_type,
                self.vehicle_id,
            )
            return
        self.current_position = msg.pose.position
        self.has_local_position = True
        self.current_position_time = msg.header.stamp
        self.current_yaw = self.q2yaw(msg.pose.orientation)

    def vision_pose_callback(self, msg):
        if not self.finite_position(msg.pose.position):
            rospy.logwarn_throttle(
                1.0,
                "%s_%s: vision_pose 含 NaN/Inf，忽略该帧",
                self.vehicle_type,
                self.vehicle_id,
            )
            return
        self.vision_pose = msg.pose.position

    def local_velocity_callback(self, msg):
        if not self.finite_values(
                msg.twist.linear.x,
                msg.twist.linear.y,
                msg.twist.linear.z):
            rospy.logwarn_throttle(
                1.0,
                "%s_%s: local_velocity 含 NaN/Inf，忽略该帧",
                self.vehicle_type,
                self.vehicle_id,
            )
            return
        self.current_velocity = msg  # 更新当前速度
        # self.current_velocity_time = msg.header.stamp

    def target_pos_callback(self, msg, target_name='ball'):
        if not self.finite_position(msg.pose.position):
            rospy.logwarn_throttle(
                1.0,
                "%s 目标位姿含 NaN/Inf，忽略该帧",
                target_name,
            )
            return
        pose = self.target_poses[target_name]
        pose.position.x = msg.pose.position.x
        pose.position.y = msg.pose.position.y
        pose.position.z = msg.pose.position.z
        pose.orientation = msg.pose.orientation
        if target_name == 'ball':
            self.target_pose.position.x = msg.pose.position.x
            self.target_pose.position.y = msg.pose.position.y
            self.target_pose.position.z = msg.pose.position.z
            self.target_pose.orientation = msg.pose.orientation
    def construct_target(self, x=0, y=0, z=0, vx=0, vy=0, vz=0, afx=0, afy=0, afz=0, yaw=0, yaw_rate=0):
        target_raw_pose = PositionTarget()
        target_raw_pose.header.stamp = rospy.Time.now()
        target_raw_pose.header.frame_id = 'map'
        target_raw_pose.coordinate_frame = self.coordinate_frame
        
        target_raw_pose.position.x = x
        target_raw_pose.position.y = y
        target_raw_pose.position.z = z

        target_raw_pose.velocity.x = vx
        target_raw_pose.velocity.y = vy
        target_raw_pose.velocity.z = vz
        
        target_raw_pose.acceleration_or_force.x = afx
        target_raw_pose.acceleration_or_force.y = afy
        target_raw_pose.acceleration_or_force.z = afz

        target_raw_pose.yaw = yaw
        target_raw_pose.yaw_rate = yaw_rate

        if(self.motion_type == 0):
            # 位置控制：忽略速度、加速度和航向速度
            target_raw_pose.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                            + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                            + PositionTarget.IGNORE_YAW_RATE
        elif(self.motion_type == 1):
            # 速度控制：忽略位置和加速度
            target_raw_pose.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ \
                            + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                            + PositionTarget.IGNORE_YAW_RATE
        elif(self.motion_type == 2):
            # 加速度控制：忽略位置和速度
            target_raw_pose.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ \
                            + PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                            + PositionTarget.IGNORE_YAW
        
        return target_raw_pose

    def cmd_pose_flu_callback(self, msg):
        if not self.finite_position(msg.position):
            rospy.logwarn_throttle(1.0, "拒绝 NaN/Inf cmd_pose_flu")
            return
        self.coordinate_frame = 9
        self.motion_type = 0
        yaw = self.q2yaw(msg.orientation)
        self.target_motion = self.construct_target(x=msg.position.x,y=msg.position.y,z=msg.position.z,yaw=yaw)

    def cmd_pose_enu_callback(self, msg):
        if not self.finite_position(msg.position):
            rospy.logwarn_throttle(1.0, "拒绝 NaN/Inf cmd_pose_enu")
            return
        self.coordinate_frame = 1
        self.motion_type = 0
        yaw = self.q2yaw(msg.orientation)
        self.target_motion = self.construct_target(x=msg.position.x,y=msg.position.y,z=msg.position.z,yaw=yaw)
        
    def cmd_vel_flu_callback(self, msg):
        if not self.finite_twist(msg):
            rospy.logwarn_throttle(1.0, "拒绝 NaN/Inf cmd_vel_flu")
            return
        self.hover_state_transition(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
        if self.hover_flag == 0:
            self.coordinate_frame = 8
            self.motion_type = 1
            self.target_motion = self.construct_target(vx=msg.linear.x,vy=msg.linear.y,vz=msg.linear.z,yaw_rate=msg.angular.z)  

    def cmd_vel_enu_callback(self, msg):
        if not self.finite_twist(msg):
            rospy.logwarn_throttle(1.0, "拒绝 NaN/Inf cmd_vel_enu")
            return
        self.hover_state_transition(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
        if self.hover_flag == 0:
            self.coordinate_frame = 1
            self.motion_type = 1
            self.target_motion = self.construct_target(vx=msg.linear.x,vy=msg.linear.y,vz=msg.linear.z,yaw_rate=msg.angular.z)    

    def cmd_accel_flu_callback(self, msg):
        if not self.finite_twist(msg):
            rospy.logwarn_throttle(1.0, "拒绝 NaN/Inf cmd_accel_flu")
            return
        self.hover_state_transition(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
        if self.hover_flag == 0:
            self.coordinate_frame = 8
            self.motion_type = 2
            self.target_motion = self.construct_target(afx=msg.linear.x,afy=msg.linear.y,afz=msg.linear.z,yaw_rate=msg.angular.z)
            
    def cmd_accel_enu_callback(self, msg):
        if not self.finite_twist(msg):
            rospy.logwarn_throttle(1.0, "拒绝 NaN/Inf cmd_accel_enu")
            return
        self.hover_state_transition(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
        if self.hover_flag == 0:
            self.coordinate_frame = 1 
            self.motion_type = 2
            self.target_motion = self.construct_target(afx=msg.linear.x,afy=msg.linear.y,afz=msg.linear.z,yaw_rate=msg.angular.z)
            
    def hover_state_transition(self,x,y,z,w):
        if abs(x) > 0.02 or abs(y)  > 0.02 or abs(z)  > 0.02 or abs(w)  > 0.005:
            self.hover_flag = 0
            self.flight_mode = 'OFFBOARD'
        elif not self.flight_mode == "HOVER":
            self.hover_flag = 1
            self.flight_mode = 'HOVER'
            self.hover()
            
    def cmd_callback(self, msg):
        if msg.data == self.last_cmd or msg.data == '' or msg.data == 'stop controlling':
            return

        elif msg.data == 'ARM':
            self.arm_state =self.arm()
            print(self.vehicle_type+'_'+self.vehicle_id+": Armed "+str(self.arm_state))

        elif msg.data == 'DISARM':
            self.arm_state = not self.disarm()
            print(self.vehicle_type+'_'+self.vehicle_id+": Armed "+str(self.arm_state))

        elif msg.data[:-1] == "mission" and not msg.data == self.mission:
            self.mission = msg.data
            print(self.vehicle_type+'_'+self.vehicle_id+": "+msg.data)

        else:
            self.flight_mode = msg.data
            self.flight_mode_switch()

        self.last_cmd = msg.data

    def q2yaw(self, q):
        try:
            if isinstance(q, Quaternion):
                rotate_z_rad = q.yaw_pitch_roll[0]
            else:
                if not self.finite_values(q.w, q.x, q.y, q.z):
                    return 0.0
                q_ = Quaternion(q.w, q.x, q.y, q.z)
                rotate_z_rad = q_.yaw_pitch_roll[0]
        except (ValueError, ZeroDivisionError):
            return 0.0

        return rotate_z_rad if math.isfinite(float(rotate_z_rad)) else 0.0
    
    def arm(self):
        try:
            res = self.armService(True)
            if not res.success:
                rospy.logwarn("%s_%s: arming rejected by PX4, result=%s",
                              self.vehicle_type, self.vehicle_id, res.result)
            return res.success
        except rospy.ServiceException as e:
            print(self.vehicle_type+'_'+self.vehicle_id+": arming failed! Exception: {}".format(e))
            return False
    
    def disarm(self):
        try:
            res = self.armService(False)
            if not res.success:
                rospy.logwarn("%s_%s: disarming rejected by PX4, result=%s",
                              self.vehicle_type, self.vehicle_id, res.result)
            return res.success
        except rospy.ServiceException as e:
            print(self.vehicle_type+'_'+self.vehicle_id+": disarming failed! Exception: {}".format(e))
            return False
    
    def hover(self):
        self.coordinate_frame = 1
        self.motion_type = 0
        self.target_motion = self.construct_target(x=self.current_position.x,y=self.current_position.y,z=self.current_position.z,yaw=self.current_yaw)
        print(self.vehicle_type+'_'+self.vehicle_id+":"+self.flight_mode)

    def flight_mode_switch(self):
        if self.flight_mode == 'HOVER':
            self.hover_flag = 1
            self.hover()
        else:
            try:
                res = self.flightModeService(custom_mode=self.flight_mode)
                if res.mode_sent:
                    print(self.vehicle_type+'_'+self.vehicle_id+": "+self.flight_mode)
                    return True
                else:
                    print(self.vehicle_type+'_'+self.vehicle_id+": "+self.flight_mode+" failed to set")
                    return False
            except rospy.ServiceException as e:
                print(self.vehicle_type+'_'+self.vehicle_id+": "+self.flight_mode+" failed! Exception: {}".format(e))
                return False

if __name__ == '__main__':
    communication = Communication(sys.argv[1],sys.argv[2])
    communication.start()
