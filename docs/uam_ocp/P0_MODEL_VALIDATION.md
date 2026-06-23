# P0 model validation

Status: **PASS**

- Generated URDF loaded with `pin.buildModelFromUrdf(...,
  pin.JointModelFreeFlyer())`.
- `StateMultibody` created successfully: `nq=13`, `nv=12`, `nx=25`, `ndx=24`.
- Six independent joints and all q/v indices were discovered from the model.
- Neutral quaternion norm is 1.0.
- Total URDF mass is 1.717 kg.
- `gripper_base_link` exists and is used as the end-effector frame.
- Neutral and nonzero arm configurations both produced finite FK, 6-by-12 EE
  Jacobian, positive-definite CRBA matrix, and finite nonlinear effects.
- Neutral EE position relative to a base at the world origin is
  `[0.110667, -0.003836, -0.210263] m`.
- Source inertial scan found no massive link with zero mass or a non-positive
  inertia eigenvalue.

Machine-readable details, including matrices' dimensions, condition numbers,
joint limits, source link inertias, and nonlinear terms:

- `uam_ocp/results/p0/p0_model_summary.json`
- `uam_ocp/results/p0/p0_model_summary.yaml`

Numerical validity is not equivalent to identified physical accuracy. The arm
mass values, wrist range, and mimic-gripper reduction remain calibration risks.

