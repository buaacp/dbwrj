"""MAVROS position/yaw Offboard interface; never publishes actuator commands."""

import rospy
import time
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import CommandBool, SetMode


class HoverOffboardInterface(object):
    def __init__(self, topics, services, converter):
        self.converter = converter
        self.publisher = rospy.Publisher(topics["uav_setpoint"], PositionTarget, queue_size=10)
        self.arm_service = rospy.ServiceProxy(services["arming"], CommandBool)
        self.mode_service = rospy.ServiceProxy(services["set_mode"], SetMode)
        self.last_publish_wall_time = None
        self.publish_arrivals = []

    def describe_position_setpoint(self, position_world, yaw_world):
        """Return the exact MAVROS setpoint semantics used by publish()."""
        position = self.converter.world_to_setpoint_position(position_world)
        return {
            "message_type": "mavros_msgs/PositionTarget",
            "coordinate_frame": PositionTarget.FRAME_LOCAL_NED,
            "coordinate_frame_name": "FRAME_LOCAL_NED",
            "type_mask": (PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ |
                          PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                          PositionTarget.IGNORE_YAW_RATE),
            "ignored_fields": ["vx", "vy", "vz", "afx", "afy", "afz", "yaw_rate"],
            "position_ignored": False,
            "requested_position_enu": list(position_world),
            "outgoing_position": list(position),
            "requested_yaw_rad": self.converter.world_to_setpoint_yaw(yaw_world),
            "source_frame": getattr(self.converter, "source_frame", "WORLD_ENU"),
            "destination_frame": getattr(self.converter, "destination_frame", "MAVROS_LOCAL_ENU"),
        }

    def publish(self, position_world, yaw_world):
        position = self.converter.world_to_setpoint_position(position_world)
        msg = PositionTarget(); msg.header.stamp = rospy.Time.now()
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = (PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ |
                         PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                         PositionTarget.IGNORE_YAW_RATE)
        msg.position.x,msg.position.y,msg.position.z = position
        msg.yaw = self.converter.world_to_setpoint_yaw(yaw_world)
        self.publisher.publish(msg)
        self.last_publish_wall_time = time.monotonic()
        self.publish_arrivals.append(self.last_publish_wall_time)

    def publish_zero_velocity(self, yaw_world):
        """Prestream a safe zero-velocity reference before the first pose."""
        msg = PositionTarget(); msg.header.stamp = rospy.Time.now()
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ |
                         PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                         PositionTarget.IGNORE_YAW_RATE)
        velocity = self.converter.world_to_setpoint_velocity([0.0, 0.0, 0.0])
        msg.velocity.x,msg.velocity.y,msg.velocity.z = velocity
        msg.yaw = self.converter.world_to_setpoint_yaw(yaw_world)
        self.publisher.publish(msg)
        self.last_publish_wall_time = time.monotonic()
        self.publish_arrivals.append(self.last_publish_wall_time)

    def wait_for_services(self, timeout):
        rospy.wait_for_service(self.arm_service.resolved_name, timeout=timeout)
        rospy.wait_for_service(self.mode_service.resolved_name, timeout=timeout)

    def request_offboard(self):
        return bool(self.mode_service(base_mode=0, custom_mode="OFFBOARD").mode_sent)

    def request_arm(self):
        return bool(self.arm_service(value=True).success)
