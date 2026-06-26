import os
import sys
import unittest

import yaml

PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PKG not in sys.path:
    sys.path.insert(0, PKG)

from nmpc_tracking.config_validation import validate_config


class TestConfigValidation(unittest.TestCase):
    def test_default_config_validates(self):
        cfg = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "config", "dual_mpc_pregrasp.yaml"))
        with open(cfg) as f:
            config = yaml.safe_load(f)
        dims = validate_config(config)
        self.assertEqual(dims["joint_count"], len(config["arm"]["joint_names"]))

    def test_bad_rate_rejected(self):
        cfg = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "config", "dual_mpc_pregrasp.yaml"))
        with open(cfg) as f:
            config = yaml.safe_load(f)
        config["controller"]["dt"] = 0.03
        with self.assertRaises(ValueError):
            validate_config(config)


if __name__ == "__main__":
    unittest.main()
