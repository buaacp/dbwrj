#!/usr/bin/env python3
import os
import sys

import yaml

from nmpc_tracking.acados_model import controller_dimensions, hover_command
from nmpc_tracking.robot_layout import assert_layout_consistency, load_robot_layout


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
CONFIG = os.path.join(ROOT, "src", "nmpc_tracking", "config", "dual_mpc_pregrasp.yaml")
RESULTS = os.path.join(ROOT, "results", "nmpc_smoke")


def main():
    with open(CONFIG, "r") as f:
        config = yaml.safe_load(f)
    layout = load_robot_layout(config)
    dims = controller_dimensions(config, layout)
    assert_layout_consistency(layout, dims["state_dim"], dims["command_dim"], dims["control_rate_dim"])
    hover = hover_command(config, layout)[0]
    report = layout.as_dict()
    report["controller_dimensions"] = {
        "state_dim": dims["state_dim"],
        "command_dim": dims["command_dim"],
        "control_rate_dim": dims["control_rate_dim"],
        "state_dim_from_definition": 16 + 3 * layout.arm_dof,
        "user_formula_13_plus_3na": 13 + 3 * layout.arm_dof,
        "formula_note": "Given z=[p,v,rpy,omega,q,dq,u_c], the dimension is 16+3*na.",
    }
    report["hover_thrust_n"] = float(hover)
    os.makedirs(RESULTS, exist_ok=True)
    with open(os.path.join(RESULTS, "robot_layout_audit.yaml"), "w") as f:
        yaml.safe_dump(report, f, sort_keys=False)
    print(yaml.safe_dump(report, sort_keys=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
