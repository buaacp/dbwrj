import unittest
from pathlib import Path
import numpy as np
from uam_ocp_gazebo_bridge.trajectory_profile import (post_offboard_state,
    neutralization_timed_out, post_telemetry_state, ready_duration_satisfied, required_duration, sample_profile,
    takeoff_abort_reason, takeoff_hold_ready, takeoff_reference, update_ready_since)


class TestProfiles(unittest.TestCase):
    def test_endpoints_and_velocity_limit(self):
        q0=np.array([0.,0.]);q1=np.array([2.,-1.]);limits=np.array([.5,.5])
        duration=required_duration(q0,q1,limits,1.0)
        self.assertAlmostEqual(duration,7.5)
        np.testing.assert_allclose(sample_profile(q0,q1,0.,duration)[0],q0)
        np.testing.assert_allclose(sample_profile(q0,q1,duration,duration)[0],q1)
        np.testing.assert_allclose(sample_profile(q0,q1,0.,duration)[1],0.)
        np.testing.assert_allclose(sample_profile(q0,q1,duration,duration)[1],0.,atol=1e-14)
        self.assertLessEqual(np.max(np.abs(sample_profile(q0,q1,duration/2.,duration)[1])/limits),1.0+1e-12)

    def test_profile_does_not_overshoot_joint_endpoints(self):
        q0=np.array([-.2,.5]);q1=np.array([1.0,-.7])
        samples=np.array([sample_profile(q0,q1,t,4.)[0] for t in np.linspace(0,4,101)])
        self.assertTrue(np.all(samples>=np.minimum(q0,q1)-1e-12));self.assertTrue(np.all(samples<=np.maximum(q0,q1)+1e-12))

    def test_takeoff_reference_endpoints_and_state_selection(self):
        p0=np.array([.2,-.1,.25]);p1=np.array([0.,0.,1.])
        np.testing.assert_allclose(takeoff_reference(p0,p1,0.,8.),p0)
        np.testing.assert_allclose(takeoff_reference(p0,p1,8.,8.),p1)
        middle=takeoff_reference(p0,p1,4.,8.)
        np.testing.assert_allclose(middle,0.5*(p0+p1))
        self.assertEqual(post_offboard_state(np.linalg.norm(p1-p0),.02),"TAKEOFF_TRANSITION")
        self.assertEqual(post_offboard_state(.01,.02),"TAKEOFF_HOLD")

    def test_takeoff_abort_reason_separates_no_ascent_from_tracking_error(self):
        p0=np.array([0.,0.,-0.116]);pref=takeoff_reference(p0,np.array([0.,0.,1.]),2.1,8.)
        self.assertEqual(takeoff_abort_reason(p0,pref,p0,2.1,2.0,.08,.5),"TAKEOFF_NO_ASCENT")
        actual=p0+np.array([.6,0.,.2])
        self.assertEqual(takeoff_abort_reason(actual,pref,p0,2.1,2.0,.08,.5),"TAKEOFF_TRANSITION_POSITION_ERROR")
        self.assertIsNone(takeoff_abort_reason(pref,pref,p0,2.1,2.0,.08,.5))

    def test_takeoff_hold_requires_continuous_readiness(self):
        self.assertTrue(takeoff_hold_ready(.049,.049,0.,0.,np.deg2rad(2.9)))
        since=update_ready_since(10.,True,None)
        self.assertFalse(ready_duration_satisfied(11.99,since,2.0))
        self.assertTrue(ready_duration_satisfied(12.0,since,2.0))
        self.assertIsNone(update_ready_since(12.1,False,since))

    def test_arm_neutralization_profile_and_transition(self):
        q_initial=np.array([.6,-.4,.2,.1,-.2,.05]);q_neutral=np.zeros(6)
        np.testing.assert_allclose(sample_profile(q_initial,q_neutral,0.,3.)[0],q_initial)
        np.testing.assert_allclose(sample_profile(q_initial,q_neutral,3.,3.)[0],q_neutral,atol=1e-14)
        self.assertEqual(post_telemetry_state(True),"ARM_NEUTRALIZE")
        since=update_ready_since(4.,True,None)
        self.assertFalse(ready_duration_satisfied(4.99,since,1.0));self.assertTrue(ready_duration_satisfied(5.,since,1.0))
        self.assertFalse(neutralization_timed_out(8.,8.));self.assertTrue(neutralization_timed_out(8.01,8.))

    def test_normal_runner_logs_do_not_use_warn(self):
        root=Path(__file__).resolve().parents[1]
        text=(root/"scripts/g1_static_hold_runner.py").read_text()
        logwarn_lines=[line.strip() for line in text.splitlines() if "rospy.logwarn" in line]
        self.assertEqual(logwarn_lines, ['rospy.logwarn("G1 service request failed: %s", exc)'])


if __name__=="__main__":unittest.main()
