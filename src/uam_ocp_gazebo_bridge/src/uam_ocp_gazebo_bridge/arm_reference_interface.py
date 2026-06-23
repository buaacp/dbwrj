"""Adapter for the mixed ros_control interfaces in iris_arm.launch."""

import numpy as np
import rospy
from std_msgs.msg import Float64, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ArmReferenceInterface(object):
    def __init__(self, topics, config, all_joint_names):
        self.all_joint_names = list(all_joint_names); self.cfg = config
        self.group_names = list(config["group_velocity_joints"])
        self.wrist_name = config["wrist_position_joint"]
        self.knuckle_name = config["fixed_knuckle_joint"]
        self.group_pub = rospy.Publisher(topics["arm_group_velocity_command"], Float64MultiArray, queue_size=10)
        self.wrist_pub = rospy.Publisher(topics["wrist_position_command"], Float64, queue_size=10)
        self.knuckle_pub = rospy.Publisher(topics["gripper_trajectory_command"], JointTrajectory, queue_size=10)

    def publish(self, q_cmd, qd_cmd, q_measured):
        q_cmd=np.asarray(q_cmd); qd_cmd=np.asarray(qd_cmd); q_measured=np.asarray(q_measured)
        index=dict((name,i) for i,name in enumerate(self.all_joint_names)); gain=float(self.cfg["position_feedback_gain_s_inv"])
        velocities=[]
        for name in self.group_names:
            i=index[name]; limit=float(self.cfg["velocity_limit_rad_s"][name])
            velocities.append(float(np.clip(qd_cmd[i]+gain*(q_cmd[i]-q_measured[i]),-limit,limit)))
        msg=Float64MultiArray();msg.data=velocities;self.group_pub.publish(msg)
        self.wrist_pub.publish(Float64(data=float(q_cmd[index[self.wrist_name]])))
        trajectory=JointTrajectory();trajectory.header.stamp=rospy.Time.now();trajectory.joint_names=[self.knuckle_name]
        point=JointTrajectoryPoint();point.positions=[float(self.cfg["fixed_knuckle_position_rad"])];point.time_from_start=rospy.Duration(0.2)
        trajectory.points=[point];self.knuckle_pub.publish(trajectory)
