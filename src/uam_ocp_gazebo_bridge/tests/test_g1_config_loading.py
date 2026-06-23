import unittest
from pathlib import Path
from uam_ocp_gazebo_bridge.config_loader import resolve_g1_config


class TestConfig(unittest.TestCase):
    def test_existing_scenario_sources(self):
        root=Path(__file__).resolve().parents[3]
        cfg=resolve_g1_config(root/"src/uam_ocp_gazebo_bridge/config/g1_static_hold.yaml",root)
        self.assertEqual(cfg["configurations"],["neutral","left_offset","fully_extended","p2_terminal"])
        for name in cfg["configurations"]:self.assertEqual(cfg["scenarios"][name]["status"],"RESOLVED")
        self.assertEqual(cfg["scenarios"]["left_offset"]["joints"]["shoulder_pan_joint"],0.8)
        self.assertEqual(cfg["scenarios"]["p2_terminal"]["joints"]["left_knuckle_joint"],0.0)
        self.assertEqual(cfg["startup"]["initial_telemetry_grace_s"],10.0)
        self.assertTrue(cfg["startup"]["require_clock_before_execution"])
        self.assertEqual(cfg["offboard_transition"]["offboard_confirm_timeout_s"],8.0)
        self.assertEqual(cfg["takeoff"]["transition_duration_s"],8.0)
        self.assertEqual(cfg["takeoff"]["transition_position_error_m"],0.5)
        self.assertEqual(cfg["takeoff"]["no_ascent_check_after_s"],2.0)
        self.assertEqual(cfg["takeoff"]["min_ascent_after_check_m"],0.08)
        self.assertEqual(cfg["absolute_safety"]["max_distance_from_initial_m"],2.0)
        self.assertTrue(cfg["arm_neutralize"]["enabled"])
        self.assertEqual(cfg["arm_neutralize"]["tracking_error_deg"],15.0)
        self.assertEqual(cfg["joint_limits"]["shoulder_pan_joint"]["lower_rad"],-1.57)


if __name__=="__main__":unittest.main()
