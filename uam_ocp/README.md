# UAM full-state trajectory optimization (P0-P2)

This module builds a floating-base Pinocchio model from the `le_arm` Gazebo
Xacro, validates coupled rigid-body dynamics, and solves an offline Crocoddyl
free-flight pre-grasp problem. It does not provide PX4, MAVROS, contact, vision,
or closed-loop Gazebo integration.

The tested environment is the existing `eagle_mpc` micromamba environment:

```bash
export UAM_PYTHON=/home/zlhq/micromamba-root/envs/eagle_mpc/bin/python
export MPLCONFIGDIR=/tmp/uam_ocp_mpl

./uam_ocp/scripts/build_pinocchio_model.py
$UAM_PYTHON uam_ocp/scripts/validate_p0_model.py
$UAM_PYTHON uam_ocp/scripts/validate_p1_dynamics.py
$UAM_PYTHON uam_ocp/scripts/run_p2_pregrasp.py
$UAM_PYTHON uam_ocp/scripts/validate_prediction_model.py
$UAM_PYTHON uam_ocp/scripts/demo_prediction_optimization.py
$UAM_PYTHON uam_ocp/scripts/validate_static_trim.py
$UAM_PYTHON uam_ocp/scripts/validate_local_trim_stability.py
$UAM_PYTHON uam_ocp/scripts/run_p2_bulb_pregrasp.py
$UAM_PYTHON -m unittest discover -s uam_ocp/tests -v
```

Configuration is under `uam_ocp/config`. Values marked
`TODO_NEEDS_CALIBRATION` are explicit placeholders and must not be interpreted
as measured hardware parameters.

The reusable optimizer transition is `UAMPredictionModel.step()`. Its
`build_prediction_action_model()`, `rollout()`, and `linearize()` methods all
use the same Crocoddyl free-forward-dynamics/Euler action model as P2.
