"""P2.7 target, IK, static-trim seed, and strategy regression tests."""
import unittest
import numpy as np
from pathlib import Path
from uam_ocp.actuation import UamActuation
from uam_ocp.bulb_pregrasp import *
from uam_ocp.bulb_pregrasp_planner import BulbPregraspPlanner
from uam_ocp.bulb_pregrasp_results import evaluate_solution
from uam_ocp.model_loader import load_uam_model
from uam_ocp.prediction_model import UAMPredictionModel

class TestBulbPregrasp(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  cls.robot=load_uam_model();cls.actuation=UamActuation(cls.robot);cls.prediction=UAMPredictionModel(cls.robot,cls.actuation);cls.planner=BulbPregraspPlanner(cls.robot,cls.actuation,cls.prediction)
 def test_scene_target_and_ik(self):
  self.assertEqual(self.planner.bulb_diagnostics["pose_source"],"SCENE_FILE")
  np.testing.assert_allclose(self.planner.target_pose.translation,[.6,-.35,.265])
  self.assertTrue(self.planner.ik_report["success"])
  self.assertEqual(self.planner.ik_report["fixed_joints"],{"left_knuckle_joint":0.0})
 def test_three_strategies(self):
  for name in ("arm_dominant","uav_dominant","whole_body"):
   solution=self.planner.solve_strategy(name);metrics,_=evaluate_solution(self.robot,self.actuation,self.planner,solution)
   self.assertTrue(metrics["pass"],(name,metrics))
   self.assertTrue(metrics["terminal_reference_trim_strict"])
   self.assertEqual(solution.costs,solution.costs_pass_2)
   self.assertEqual(solution.iterations,solution.iterations_pass_2)
   self.assertEqual(solution.total_iterations,solution.iterations_pass_1+solution.iterations_pass_2)
   self.assertEqual(metrics["iterations"],solution.iterations_pass_2)
   self.assertEqual(metrics["pass_1_iterations"],solution.iterations_pass_1)
   self.assertEqual(metrics["pass_2_iterations"],solution.iterations_pass_2)
   self.assertEqual(metrics["total_fddp_iterations"],solution.total_iterations)
   self.assertNotEqual(solution.total_iterations,solution.iterations)
 def test_pass_two_cost_plot_label(self):
  text=(Path(__file__).resolve().parents[1]/"uam_ocp"/"visualization.py").read_text()
  self.assertIn("BoxFDDP pass 2 cost convergence",text)
  self.assertIn("Pass-2 BoxFDDP iteration",text)
if __name__=="__main__":unittest.main()
