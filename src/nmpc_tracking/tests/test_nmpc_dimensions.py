import os
import sys
import unittest

import yaml

PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PKG not in sys.path:
    sys.path.insert(0, PKG)

from nmpc_tracking.acados_controller import AcadosUnavailableError, AcadosNmpcController, acados_available
from nmpc_tracking.acados_model import build_interface_dynamics, controller_dimensions
from nmpc_tracking.robot_layout import load_robot_layout


class TestNmpcDimensions(unittest.TestCase):
    def setUp(self):
        cfg = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "config", "dual_mpc_pregrasp.yaml"))
        with open(cfg) as f:
            self.config = yaml.safe_load(f)
        self.layout = load_robot_layout(self.config)

    def test_dimensions(self):
        dims = controller_dimensions(self.config, self.layout)
        self.assertEqual(dims["joint_count"], 4)
        self.assertEqual(dims["command_dim"], 8)
        self.assertEqual(dims["control_rate_dim"], 8)
        self.assertEqual(dims["state_dim"], 28)

    def test_casadi_interface_model_dimensions(self):
        model = build_interface_dynamics(self.config, self.layout)
        dims = model["dimensions"]
        self.assertEqual(model["z"].shape[0], dims["state_dim"])
        self.assertEqual(model["nu"].shape[0], dims["control_rate_dim"])
        self.assertEqual(model["z_dot"].shape[0], dims["state_dim"])

    def test_acados_missing_is_explicit(self):
        if not acados_available():
            with self.assertRaises(AcadosUnavailableError):
                AcadosNmpcController(self.config)


if __name__ == "__main__":
    unittest.main()
