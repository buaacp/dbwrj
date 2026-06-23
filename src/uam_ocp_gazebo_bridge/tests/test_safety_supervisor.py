import math
import unittest
import numpy as np
from uam_ocp_gazebo_bridge.safety_supervisor import SafetySupervisor, StartupTelemetryGate


WATCHDOG={"mavros_state_timeout_s":2.5,"pose_timeout_s":.5,"velocity_timeout_s":.5,"imu_timeout_s":.3,"joint_state_timeout_s":.5,"clock_timeout_s":1.,"setpoint_publish_timeout_s":.2,"stale_confirm_cycles":2,"watchdog_rate_hz":20.}
SAFETY={"position_error_m":.5,"roll_pitch_deg":30,"joint_tracking_error_deg":15}
ABSOLUTE={"max_distance_from_initial_m":2.,"max_altitude_above_initial_m":2.,"min_altitude_below_initial_m":.3,"max_roll_pitch_deg":30.,"max_joint_tracking_error_deg":15.}


class TestSafety(unittest.TestCase):
    def setUp(self):
        self.safety=SafetySupervisor(SAFETY,WATCHDOG,ABSOLUTE);self.now=100.
        self.times=dict((key,self.now) for key in ("mavros_state","pose","velocity","imu","joint_state","clock","setpoint_publish"))

    def evaluate(self,state="ARM_DEPLOY",offboard=True,armed=True,**kwargs):
        values={"wall_now":self.now,"state":state,"receive_times":self.times,"position_error_m":.1,"roll_rad":0.,"pitch_rad":0.,"joint_error":np.zeros(6),"offboard":offboard,"armed":armed,"actual_position":np.zeros(3),"initial_position":np.zeros(3)}
        values.update(kwargs);return self.safety.evaluate(**values)

    def test_one_hz_state_is_healthy_at_2p5_timeout(self):
        for age in (1.,2.49):self.times["mavros_state"]=self.now-age;self.assertIsNone(self.evaluate())

    def test_state_stale_requires_confirmation(self):
        self.times["mavros_state"]=self.now-2.6
        self.assertIsNone(self.evaluate());self.assertEqual(self.evaluate(),"MAVROS_STATE_STALE")

    def test_high_rate_stale_is_immediate(self):
        self.times["pose"]=self.now-.51;self.assertEqual(self.evaluate(),"POSE_STALE")
        self.times["pose"]=self.now;self.times["imu"]=self.now-.31;self.assertEqual(self.evaluate(),"IMU_STALE")
        self.times["imu"]=self.now;self.times["velocity"]=self.now-.51;self.assertEqual(self.evaluate(),"VELOCITY_STALE")
        self.times["velocity"]=self.now;self.times["joint_state"]=self.now-.51;self.assertEqual(self.evaluate(),"JOINT_STATE_STALE")

    def test_mode_and_arming_loss(self):
        self.assertEqual(self.evaluate(offboard=False),"MAVROS_STATE_MODE_OR_ARMING_LOST")
        self.assertEqual(self.evaluate(armed=False),"MAVROS_STATE_MODE_OR_ARMING_LOST")

    def test_clock_and_setpoint_stale(self):
        self.times["clock"]=self.now-1.01;self.assertEqual(self.evaluate(),"SIM_CLOCK_STALE_OR_GAZEBO_PAUSED")
        self.times["clock"]=self.now;self.times["setpoint_publish"]=self.now-.21;self.assertEqual(self.evaluate(),"SETPOINT_PUBLISH_STALE")

    def test_prestream_allows_not_armed_or_offboard(self):
        self.assertIsNone(self.evaluate(state="PRESTREAM_SETPOINTS",offboard=False,armed=False))
        self.assertIsNone(self.safety.evaluate(self.now+2.7,"PRESTREAM_SETPOINTS",{},.1,0.,0.,np.zeros(6),False,False,startup_complete=False,offboard_confirmed=False,armed_confirmed=False))
        self.times["mavros_state"]=self.now-3.0
        self.assertIsNone(self.evaluate(state="PRESTREAM_SETPOINTS",offboard=False,armed=False))

    def test_high_rate_stale_after_startup_ready_is_still_checked(self):
        self.times["pose"]=self.now-.51
        self.assertEqual(self.evaluate(state="PRESTREAM_SETPOINTS",offboard=False,armed=False),"POSE_STALE")

    def test_arm_transition_does_not_require_confirmation_yet(self):
        self.assertIsNone(self.evaluate(state="ARM_AND_OFFBOARD",offboard=False,armed=False,offboard_confirmed=False,armed_confirmed=False))

    def test_physical_limits_are_immediate(self):
        self.assertEqual(self.evaluate(position_error_m=.51),"POSITION_ERROR")
        self.assertEqual(self.evaluate(roll_rad=math.radians(31)),"BASE_ATTITUDE")
        self.assertEqual(self.evaluate(joint_error=np.array([math.radians(16),0,0,0,0,0])),"JOINT_TRACKING_ERROR")

    def test_arm_transition_disables_final_hold_tracking_not_absolute_safety(self):
        self.assertIsNone(self.evaluate(state="ARM_AND_OFFBOARD",position_error_m=.8,
            position_tracking_enabled=False,offboard_confirmed=False,armed_confirmed=False))
        self.assertEqual(self.evaluate(state="ARM_AND_OFFBOARD",position_error_m=.8,
            position_tracking_enabled=False,offboard_confirmed=False,armed_confirmed=False,
            actual_position=np.array([2.1,0,0])),"ABSOLUTE_DISTANCE_FROM_INITIAL")

    def test_takeoff_transition_uses_stage_specific_reason(self):
        self.assertIsNone(self.evaluate(state="TAKEOFF_TRANSITION",position_error_m=.49,
            position_error_reason="TAKEOFF_TRANSITION_POSITION_ERROR"))
        self.assertEqual(self.evaluate(state="TAKEOFF_TRANSITION",position_error_m=.51,
            position_error_reason="TAKEOFF_TRANSITION_POSITION_ERROR"),"TAKEOFF_TRANSITION_POSITION_ERROR")

    def test_joint_tracking_is_stage_gated_and_named(self):
        large=np.array([math.radians(20),0,0,0,0,0])
        self.assertIsNone(self.evaluate(state="PRESTREAM_SETPOINTS",joint_error=large,joint_tracking_enabled=False,offboard_confirmed=False,armed_confirmed=False))
        self.assertIsNone(self.evaluate(state="ARM_NEUTRALIZE",joint_error=np.array([math.radians(14),0,0,0,0,0]),joint_error_reason="ARM_NEUTRALIZE_TRACKING_ERROR"))
        self.assertEqual(self.evaluate(state="ARM_NEUTRALIZE",joint_error=large,joint_error_reason="ARM_NEUTRALIZE_TRACKING_ERROR"),"ARM_NEUTRALIZE_TRACKING_ERROR")
        self.assertEqual(self.evaluate(state="ARM_AND_OFFBOARD",joint_error=large,offboard_confirmed=False,armed_confirmed=False),"JOINT_TRACKING_ERROR")

    def test_absolute_joint_limits_are_independent_of_tracking(self):
        self.assertEqual(self.evaluate(state="PRESTREAM_SETPOINTS",joint_error=np.zeros(6),joint_tracking_enabled=False,
            joint_position=np.array([1.6,0,0,0,0,0]),joint_lower=np.full(6,-1.57),joint_upper=np.full(6,1.57)),"JOINT_LIMIT_VIOLATION")

    def test_startup_gate_wait_ready_and_timeout(self):
        config={"initial_telemetry_grace_s":10.,"initial_state_grace_s":10.,"require_clock_before_execution":True}
        gate=StartupTelemetryGate(config,100.)
        gate.update({"pose":101.,"imu":101.,"joint_state":101.})
        self.assertFalse(gate.ready());self.assertIsNone(gate.failure_reason(102.7))
        gate.update({"velocity":103.,"mavros_state":103.,"clock":103.})
        self.assertTrue(gate.ready());self.assertIsNone(gate.failure_reason(111.))
        self.assertEqual(gate.diagnostics(111.)["initial_telemetry_wait_s"],3.)
        missing=StartupTelemetryGate(config,200.);missing.update({"pose":201.,"velocity":201.,"imu":201.,"joint_state":201.,"clock":201.})
        self.assertEqual(missing.failure_reason(210.01),"INITIAL_TELEMETRY_NOT_RECEIVED")

    def test_startup_clock_has_first_frame_gate(self):
        config={"initial_telemetry_grace_s":10.,"initial_state_grace_s":10.,"require_clock_before_execution":True}
        gate=StartupTelemetryGate(config,0.);gate.update(dict((key,1.) for key in ("pose","velocity","imu","joint_state","mavros_state")))
        self.assertIsNone(gate.failure_reason(1.1));self.assertEqual(gate.failure_reason(10.1),"INITIAL_CLOCK_NOT_RECEIVED")


if __name__=="__main__":unittest.main()
