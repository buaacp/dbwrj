# P0-P2 implementation report

## Scope and architecture

An isolated `uam_ocp/` module now derives an aerial-manipulator model from the
`le_arm` Xacro and implements validated Pinocchio/Crocoddyl offline trajectory
optimization. `eagle-mpc-python/` was read only for architectural/API patterns;
no reference file was modified and no S500 physical parameter was copied.

The implementation uses this project's Iris geometry, six independent arm and
gripper joints, actual motor IDs/spins, and source URDF inertias. Dimensions and
indices are discovered from Pinocchio rather than hard-coded.

## Main files

- Configuration: `uam_ocp/config/uam_model.yaml`, `uam_actuation.yaml`,
  `p2_scenarios.yaml`
- Reproducible model: `uam_ocp/scripts/build_pinocchio_model.py` and
  `uam_ocp/models/generated/uam_pinocchio.urdf`
- P0: `model_loader.py`, `model_validation.py`, `validate_p0_model.py`
- P1: `actuation.py`, `dynamics.py`, `validate_p1_dynamics.py`
- P2: `p2_costs.py`, `p2_planner.py`, `trajectory_io.py`, `visualization.py`,
  `run_p2_pregrasp.py`
- Regression tests: `uam_ocp/tests/test_p0_model.py`,
  `test_p1_actuation.py`, `test_p2_trajectory.py`
- Canonical reports: `docs/uam_ocp/`

## Results

- P0 PASS: 1.717 kg free-flyer model, `nq=13`, `nv=12`, six model-derived
  joints, valid FK/Jacobian/CRBA/nonlinear terms at two configurations.
- P1 PASS: native Crocoddyl map equals the independently built 12-by-10 physical
  map; static trim balances the offset COM; single-rotor signs match Gazebo;
  arm torque produces nonzero base reaction.
- P2 PASS: BoxFDDP converged in three iterations; independent dynamics rollout
  agrees exactly; hard control bounds and terminal tolerances pass.
- Four regression tests ran successfully with Python `unittest` in 0.07 s.
  Tests are pytest-compatible, but pytest itself was unavailable and attempted
  network installation could not be completed in this environment.

## Parameter provenance and limitations

Rotor position, ID, direction, spin, Gazebo thrust coefficient, maximum rotor
speed, and moment/thrust ratio come from `le_arm` Xacro and PX4 Gazebo plugin
source. Link mass, COM, and inertia come from the expanded project Xacro.

Still unidentified and explicitly marked `TODO_NEEDS_CALIBRATION`:

- Real rotor thrust limits and moment/thrust ratio
- Real motor ID/wiring correspondence
- All arm actuator torque limits
- Arm/gripper measured masses, COMs, and inertias
- Continuous wrist operational range
- Correct dependent mimic-gripper dynamics
- Task-calibrated lamp pre-grasp pose and tolerances

The model is internally dynamic and coupled but its physical accuracy is
limited by those source-model parameters. The P2 result uses a nearby smoke
target and must not be represented as lamp-task validation.

## Difference from eagle-mpc-python

Both use URDF -> free-flyer Pinocchio -> StateMultibody -> free forward dynamics
-> Euler shooting -> FDDP. This module differs by generating its model from
`le_arm`, discovering six independent joints, building the asymmetric Iris
rotor map from local files, verifying Crocoddyl's mixed map numerically, using
total mass and offset-COM static trim, and centralizing all actuator parameters
in YAML. It does not use S500 dimensions, inertia, boundaries, joint count,
frames, or rotor ordering.

## Commands

```bash
source /home/zlhq/catkin_ws/devel/setup.bash
./uam_ocp/scripts/build_pinocchio_model.py

export PYTHONPATH=$PWD/uam_ocp
export MPLCONFIGDIR=/tmp/uam_ocp_mpl
PY=/home/zlhq/micromamba-root/envs/eagle_mpc/bin/python
$PY uam_ocp/scripts/validate_p0_model.py
$PY uam_ocp/scripts/validate_p1_dynamics.py
$PY uam_ocp/scripts/run_p2_pregrasp.py
$PY -m unittest discover -s uam_ocp/tests -v
```

## P3 interface checklist

- Calibrated mapping from optimizer rotor thrusts to PX4 normalized actuator
  commands, including motor order and saturation
- Timestamped desired base pose/twist/acceleration and arm q/dq/torque export
- A trajectory sampler and state-estimate time synchronization contract
- PX4 attitude/rate/thrust tracking interface and arm-controller command bridge
- Frame conversion contract between Pinocchio `base_link`, PX4 FRD/NED, Gazebo
  ENU, and arm-controller frames
- Online safety monitor for state, tilt, thrust, torque, and tracking error
- SITL-only rollout comparison before any hardware use

## Canonical prediction model extension

The P0-P2 dynamics are now exposed by
`uam_ocp.prediction_model.UAMPredictionModel`. P2 constructs all running and
terminal dynamics through this class, so there is one Crocoddyl
`FreeFwdDynamics -> IntegratedActionModelEuler` construction path. The reusable
interface provides one-step prediction, rollout, `calcDiff()` linearization,
control bounds, nominal equal-thrust hover input, and a separate full-model
static trim diagnostic. Validation and usage are documented in
`PREDICTION_MODEL.md` and `PREDICTION_MODEL_VALIDATION.md`.

## P2.6 configuration-dependent static trim

`StaticTrimSolver` now computes `h(q)=RNEA(q,0,0)`, solves the bounded strict
equality QP against the existing P1 map, and independently verifies each result
with ABA and Crocoddyl rollout. The previous clipped least-squares helper now
delegates to this strict solver and raises if strict trim is unavailable. P2
retains `fixed_initial_trim` by default and supports explicit per-reference-node
`static_trim` mode with no silent approximate fallback. Validation covered 17
horizontal body/arm configurations and 40 P2 reference nodes. Details are in
`STATIC_TRIM.md` and `STATIC_TRIM_VALIDATION.md`.

## P2.6b local trim stability

`LocalTrimLQR` linearizes the canonical prediction model at strict trim points
in the 24-dimensional `StateMultibody` tangent space, solves the discrete DARE,
checks controllability/PBH stabilizability, and applies clipped physical-input
feedback in nonlinear Crocoddyl rollouts. Automatic tests include neutral,
fully extended, P2 terminal, and minimum-margin configurations. Results and
limitations are documented in `LOCAL_TRIM_STABILITY.md` and
`LOCAL_TRIM_STABILITY_VALIDATION.md`.

## P2.7 bulb pregrasp

P2.7 resolves the bulb pose from the simulation world, constructs a bulb-axis
pregrasp frame, generates a five-active-joint damped-IK seed while retaining
the open knuckle in full dynamics, and compares three soft coordination
strategies. Every running reference uses strict configuration-dependent trim.
Details are in `P2_BULB_PREGRASP.md` and its validation report.
