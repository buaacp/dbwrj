# P2 trajectory optimization

Status: **PASS** for the configured offline smoke-test scenario.

Implemented chain:

`StateMultibody -> verified ActuationModelFloatingBaseThrusters -> DifferentialActionModelFreeFwdDynamics -> IntegratedActionModelEuler -> ShootingProblem -> SolverBoxFDDP`

- Horizon: 2.0 s, `dt=0.05 s`, 40 controls and 41 states.
- Warm start: `StateMultibody.diff/integrate` interpolation, normalized
  quaternion, and full-model static hover trim.
- Running costs: EE position/orientation, base attitude, state, and control
  around trim.
- Terminal costs: EE position/orientation/velocity, base attitude, and state.
- Hard bounds: all four thrusts and all six joint torques from
  `uam_actuation.yaml` using model `u_lb/u_ub` and `SolverBoxFDDP`.
- Joint limits, joint velocity, and +/-30 degree body roll/pitch use quadratic
  barrier soft penalties; they are not strict guarantees. Obstacle distance,
  collision, and contact are absent by scope.
- `Delta u` is explicitly P2.1 pending; no false claim is made.

SolverBoxFDDP converged in three iterations. Rollout inconsistency was zero.
Terminal metrics were:

- EE position error: `5.599e-7 m`
- EE rotation error: `7.352e-7 rad`
- EE linear speed: approximately `4.05e-6 m/s`
- EE angular speed: approximately `1.31e-6 rad/s`
- Terminal base roll/pitch below `4e-6 rad`
- All physical controls inside configured bounds

The target is a nearby URDF-derived smoke-test pose marked
`TODO_NEEDS_CALIBRATION`, not the final lamp pre-grasp pose. Passing this test
validates architecture and numerics, not task reachability at a calibrated
lamp pose.

Outputs are under `uam_ocp/results/p2_pregrasp/`, including NPZ, three CSVs,
summary YAML, terminal report, 3D trajectory, full state/control time series,
and cost convergence plots.
