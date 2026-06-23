import csv
from pathlib import Path
import tempfile
import unittest
import yaml
from uam_ocp_gazebo_bridge.offline_g1_analyzer import analyze_run


FIELDS=["sim_time_s","state","position_error_m","roll_rad","pitch_rad","speed_mps","max_joint_error_rad","offboard"]


class TestOfflineAnalyzer(unittest.TestCase):
    def make_run(self,root,rows,status="PASS",abort=None):
        root=Path(root);root.mkdir(exist_ok=True)
        with (root/"telemetry.csv").open("w",newline="") as stream:
            writer=csv.DictWriter(stream,fieldnames=FIELDS);writer.writeheader();writer.writerows(rows)
        (root/"result.yaml").write_text(yaml.safe_dump({"status":status,"abort":abort}))
        (root/"topic_rates.yaml").write_text(yaml.safe_dump({"topic_rates_hz":{"mavros_state":{"max_interarrival_s":1.0}}}))
        return root

    def test_empty_normal_and_aborted_logs(self):
        with tempfile.TemporaryDirectory() as temp:
            base=Path(temp)
            empty=self.make_run(base/"empty",[],"NOT_RUN_ENVIRONMENT_UNAVAILABLE")
            self.assertEqual(analyze_run(empty,base/"empty_analysis")["status"],"NOT_RUN_ENVIRONMENT_UNAVAILABLE")
            rows=[dict(zip(FIELDS,[float(i),"CONFIGURATION_HOLD",.01,0.,0.,.01,.01,True])) for i in range(5)]
            normal=self.make_run(base/"normal",rows);self.assertEqual(analyze_run(normal,base/"normal_analysis")["status"],"PASS")
            aborted=self.make_run(base/"aborted",rows,"ABORTED",{"reason":"POSE_STALE"})
            self.assertEqual(analyze_run(aborted,base/"aborted_analysis")["status"],"ABORTED")
            old=self.make_run(base/"old",rows,"ABORTED",{"reason":"STATE_TIMEOUT"})
            self.assertEqual(analyze_run(old,base/"old_analysis")["historical_classification"],"ABORTED_FALSE_STALE_STATE")
            startup=self.make_run(base/"startup",rows,"ABORTED",{"reason":"POSITION_ERROR","state_machine_state":"ARM_AND_OFFBOARD"})
            self.assertEqual(analyze_run(startup,base/"startup_analysis")["historical_classification"],"HISTORICAL_STARTUP_REFERENCE_BUG")
            takeoff=self.make_run(base/"takeoff",rows,"ABORTED",{"reason":"TAKEOFF_TRANSITION_POSITION_ERROR","state_machine_state":"TAKEOFF_TRANSITION"})
            self.assertEqual(analyze_run(takeoff,base/"takeoff_analysis")["abort"]["reason"],"TAKEOFF_TRANSITION_POSITION_ERROR")
            hold=self.make_run(base/"hold",rows,"ABORTED",{"reason":"POSITION_ERROR","state_machine_state":"CONFIGURATION_HOLD"})
            self.assertIsNone(analyze_run(hold,base/"hold_analysis")["historical_classification"])


if __name__=="__main__":unittest.main()
