"""ROS1 telemetry aggregation for G1."""

import math
import threading
import time
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState


def quaternion_to_rpy(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    roll = math.atan2(2.0 * (w*x + y*z), 1.0 - 2.0 * (x*x + y*y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w*y - z*x))))
    yaw = math.atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z))
    return np.array([roll, pitch, yaw])


class StateMonitor(object):
    def __init__(self, topics, joint_names):
        self.lock = threading.Lock(); self.joint_names = list(joint_names)
        self.pose = None; self.velocity = None; self.imu = None; self.state = None; self.joints = None
        self.last_receive = {}; self.first_receive = {}; self.arrivals = {}
        self.clock_msg = None
        rospy.Subscriber(topics["uav_pose"], PoseStamped, self._set, "pose", queue_size=1)
        rospy.Subscriber(topics["uav_velocity"], TwistStamped, self._set, "velocity", queue_size=1)
        rospy.Subscriber(topics["uav_imu"], Imu, self._set, "imu", queue_size=1)
        rospy.Subscriber(topics["uav_state"], State, self._set, "mavros_state", queue_size=1)
        rospy.Subscriber(topics["arm_state"], JointState, self._set, "joint_state", queue_size=1)
        rospy.Subscriber(topics["simulation_clock"], Clock, self._set, "clock", queue_size=1)

    def _set(self, msg, key):
        wall = time.monotonic()
        attribute = {"mavros_state":"state", "joint_state":"joints", "clock":"clock_msg"}.get(key,key)
        with self.lock:
            setattr(self, attribute, msg)
            self.first_receive.setdefault(key, wall)
            self.last_receive[key] = wall
            self.arrivals.setdefault(key, []).append(wall)

    def ready(self):
        with self.lock:
            return all(value is not None for value in (self.pose, self.velocity, self.imu, self.state, self.joints,self.clock_msg))

    def snapshot(self):
        with self.lock:
            if not self.ready_unlocked(): return None
            index = dict((name, i) for i, name in enumerate(self.joints.name))
            missing = [name for name in self.joint_names if name not in index]
            if missing: return {"missing_joints": missing}
            q = np.array([self.joints.position[index[n]] for n in self.joint_names])
            qd = np.array([self.joints.velocity[index[n]] if index[n] < len(self.joints.velocity) else float("nan") for n in self.joint_names])
            effort_available = all(index[n] < len(self.joints.effort) for n in self.joint_names)
            effort = np.array([self.joints.effort[index[n]] for n in self.joint_names]) if effort_available else None
            position = np.array([self.pose.pose.position.x,self.pose.pose.position.y,self.pose.pose.position.z])
            velocity = np.array([self.velocity.twist.linear.x,self.velocity.twist.linear.y,self.velocity.twist.linear.z])
            omega = np.array([self.imu.angular_velocity.x,self.imu.angular_velocity.y,self.imu.angular_velocity.z])
            rpy = quaternion_to_rpy(self.pose.pose.orientation)
            return {"position":position,"velocity":velocity,"omega":omega,"rpy":rpy,
                    "q":q,"qd":qd,"effort":effort,"effort_available":effort_available,
                    "offboard":self.state.mode=="OFFBOARD","armed":bool(self.state.armed),
                    "connected":bool(self.state.connected),"mode":self.state.mode,"missing_joints":[]}

    def ready_unlocked(self):
        return all(value is not None for value in (self.pose,self.velocity,self.imu,self.state,self.joints,self.clock_msg))

    def receive_times(self):
        with self.lock:
            return dict(self.last_receive)

    def first_receive_times(self):
        with self.lock:
            return dict(self.first_receive)

    def topic_rates(self):
        """Compute callback rates after a run from recorded monotonic arrivals."""
        with self.lock:
            arrivals = dict((key, list(values)) for key, values in self.arrivals.items())
        result = {}
        for key in ("mavros_state","pose","velocity","imu","joint_state","clock"):
            values = np.asarray(arrivals.get(key, []), dtype=float)
            intervals = np.diff(values)
            if len(intervals) == 0:
                result[key] = {"samples":int(len(values)),"mean_hz":None,"min_hz":None,"max_hz":None,"max_interarrival_s":None}
                continue
            rates = 1.0 / intervals[intervals > 0.0]
            if len(rates) == 0:
                result[key] = {"samples":int(len(values)),"mean_hz":None,"min_hz":None,"max_hz":None,"max_interarrival_s":float(np.max(intervals))}
                continue
            result[key] = {"samples":int(len(values)),"mean_hz":float(1.0/np.mean(intervals)),
                           "min_hz":float(np.min(rates)),"max_hz":float(np.max(rates)),
                           "max_interarrival_s":float(np.max(intervals))}
        return result
