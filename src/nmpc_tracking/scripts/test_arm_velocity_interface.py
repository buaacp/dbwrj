#!/usr/bin/env python3
import argparse

import yaml

from nmpc_tracking.joint_mapping import mapping_from_config
from nmpc_tracking.arm_velocity_adapter import ArmVelocityAdapter


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="src/nmpc_tracking/config/dual_mpc_pregrasp.yaml")
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    with open(args.config) as f:
        config = yaml.safe_load(f)
    mapping = mapping_from_config(config)
    adapter = ArmVelocityAdapter(
        mapping,
        config["arm"].get("command_message_type", "std_msgs/Float64MultiArray"),
        config["arm"].get("wrist_joint_name"),
        config["arm"].get("wrist_command_message_type", "std_msgs/Float64"),
    )
    command = [0.1] + [0.0] * (mapping.size - 1)
    print("joint_names", mapping.joint_names)
    print("single positive command maps to", mapping.command_to_message_order(command).tolist())
    arm_values, wrist_value = adapter.split_command_arrays(command)
    print("main arm command array", arm_values.tolist())
    print("wrist command", wrist_value)
    if not args.execute:
        print("dry-run: no ROS publisher created")
        return
    raise SystemExit("explicit execution publishing is not implemented in this validation scaffold")


if __name__ == "__main__":
    main()
