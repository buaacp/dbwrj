from typing import Iterable, Optional, Tuple

import numpy as np

from .joint_mapping import JointMapping


class ArmVelocityAdapter:
    def __init__(self, mapping: JointMapping, message_type: str = "std_msgs/Float64MultiArray",
                 wrist_joint_name: Optional[str] = None,
                 wrist_message_type: str = "std_msgs/Float64"):
        self.mapping = mapping
        self.message_type = message_type
        self.wrist_joint_name = wrist_joint_name
        self.wrist_message_type = wrist_message_type

    def command_array(self, command_radps: Iterable[float]) -> np.ndarray:
        return self.mapping.command_to_message_order(command_radps)

    def split_command_arrays(self, command_radps: Iterable[float]) -> Tuple[np.ndarray, Optional[float]]:
        values = self.command_array(command_radps)
        if self.wrist_joint_name is None:
            return values, None
        if self.wrist_joint_name not in self.mapping.joint_names:
            raise KeyError("wrist_joint_name not in mapping: %s" % self.wrist_joint_name)
        wrist_index = self.mapping.joint_names.index(self.wrist_joint_name)
        arm_values = np.delete(values, wrist_index)
        return arm_values, float(values[wrist_index])

    def make_message(self, command_radps: Iterable[float]):
        values = self.command_array(command_radps)
        if self.message_type == "std_msgs/Float64MultiArray":
            from std_msgs.msg import Float64MultiArray
            msg = Float64MultiArray()
            msg.data = [float(v) for v in values]
            return msg
        raise ValueError("unsupported arm velocity message_type: %s" % self.message_type)

    def make_split_messages(self, command_radps: Iterable[float]):
        arm_values, wrist_value = self.split_command_arrays(command_radps)
        if self.message_type != "std_msgs/Float64MultiArray":
            raise ValueError("unsupported arm velocity message_type: %s" % self.message_type)
        from std_msgs.msg import Float64MultiArray
        arm_msg = Float64MultiArray()
        arm_msg.data = [float(v) for v in arm_values]
        wrist_msg = None
        if wrist_value is not None:
            if self.wrist_message_type != "std_msgs/Float64":
                raise ValueError("unsupported wrist velocity message_type: %s" % self.wrist_message_type)
            from std_msgs.msg import Float64
            wrist_msg = Float64(data=float(wrist_value))
        return arm_msg, wrist_msg
