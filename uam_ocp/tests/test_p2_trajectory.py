"""P2 regression test for Box-FDDP convergence and hard control bounds."""

import unittest

from uam_ocp.actuation import UamActuation
from uam_ocp.model_loader import load_uam_model
from uam_ocp.p2_planner import P2Planner
from uam_ocp.trajectory_io import terminal_metrics, trajectory_samples


class TestP2Trajectory(unittest.TestCase):
    def test_pregrasp_solution(self) -> None:
        robot = load_uam_model()
        actuation = UamActuation(robot)
        solution = P2Planner(robot, actuation).solve("pregrasp")
        samples = trajectory_samples(robot, solution)
        metrics = terminal_metrics(robot, actuation, solution, samples)
        self.assertTrue(metrics["pass"], metrics)
        self.assertLessEqual(solution.rollout_error, solution.scenario["tolerances"]["rollout_state"])


if __name__ == "__main__":
    unittest.main()

