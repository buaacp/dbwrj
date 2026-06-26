#!/usr/bin/env python3
"""ROS entry point for independent dual-layer pregrasp tracking.

The default configuration is dry-run only. It validates topics/configuration,
loads the offline reference, and refuses to construct runtime NMPC when acados
is unavailable.
"""
import os
import sys

import yaml


def _load_config_from_ros():
    import rospy
    cfg = {}
    for section in ["runtime", "topics", "trajectory", "planner", "controller", "vehicle", "arm", "safety"]:
        cfg[section] = rospy.get_param("~" + section, rospy.get_param("/" + section, {}))
    return cfg


def main():
    import rospy
    from nmpc_tracking.acados_controller import AcadosNmpcController, AcadosUnavailableError
    from nmpc_tracking.config_validation import validate_config
    from nmpc_tracking.joint_mapping import mapping_from_config
    from nmpc_tracking.px4_rate_thrust_adapter import ThrustMapper
    from nmpc_tracking.trajectory_reference import TrajectoryReference

    rospy.init_node("dual_mpc_pregrasp")
    config = _load_config_from_ros()
    dims = validate_config(config)
    rospy.loginfo("nmpc_tracking dimensions: %s", dims)
    ref = TrajectoryReference.from_npz(
        config["trajectory"]["offline_npz"],
        hold_after_s=float(config["trajectory"].get("hold_after_s", 5.0)),
    )
    mapping = mapping_from_config(config)
    vehicle = config["vehicle"]
    ThrustMapper(vehicle["mass_kg"], vehicle["gravity_mps2"], vehicle["hover_thrust_norm"],
                 vehicle.get("thrust_norm_min", 0.0), vehicle.get("thrust_norm_max", 1.0))
    try:
        AcadosNmpcController(config)
    except AcadosUnavailableError as exc:
        rospy.logerr(str(exc))
        if not bool(config["runtime"].get("dry_run", True)):
            raise
    rospy.loginfo("Loaded reference %.3f s, %d arm joints: %s",
                  ref.snapshot.duration, mapping.size, ", ".join(mapping.joint_names))
    rospy.logwarn("Dry-run scaffold active: no arming, offboard switch, takeoff, or tracking command loop is started.")
    rospy.spin()


if __name__ == "__main__":
    main()
