# Full-body optimizer prediction model

## Definition and dimensions

`uam_ocp.prediction_model.UAMPredictionModel` is the canonical nonlinear
prediction model used by the offline optimizer. It loads the generated
`le_arm` URDF through the existing P0 model loader and reuses the P1 actuation
mapping. It does not contain a second hand-written dynamics or integration
implementation.

The state is `x=[q,v]`, with:

- `q=[p_B, q_B, q_a]`, `nq=13`: world position, normalized XYZW floating-base
  quaternion, and six independent arm/gripper coordinates.
- `v=[v_B, omega_B, dq_a]`, `nv=12`: free-flyer linear and angular velocity in
  the Pinocchio free-flyer local/body tangent convention, followed by six joint
  velocities.
- Crocoddyl storage dimension `nx=25`; manifold tangent dimension `ndx=24`.
- Control `u=[T1,T2,T3,T4,tau_a1,...,tau_a6]`, `nu=10`.
- End-effector frame: `gripper_base_link`.

State differences and perturbations use `StateMultibody.diff()` and
`StateMultibody.integrate()`. Quaternion components must not be subtracted or
integrated as unconstrained Euclidean coordinates.

## Actuation and continuous dynamics

The configuration-derived matrix `W_thrust` has shape 12-by-10. Its first four
columns map each actual Iris rotor thrust into body force and moment:

```text
f_i  = T_i d_i
mu_i = r_i x f_i + s_i k_tau T_i d_i
```

The final six columns are identity channels at Pinocchio-discovered joint
velocity indices. Crocoddyl 3.2.1's `ActuationModelFloatingBaseThrusters` map is
numerically checked against this independent matrix before use.

The continuous rigid-body dynamics are:

```text
tau_g = W_thrust u
dv/dt = ABA(q, v, tau_g)
dq/dt = T(q) v
```

Pinocchio supplies URDF kinematics, inertias, gravity, Coriolis/centrifugal
terms, multibody coupling, ABA, and manifold operations. No mass matrix,
gravity term, Coriolis term, or arm/base coupling is hand-coded.

## Discrete f_pred

The sole main prediction transition is:

```text
x[k+1] = f_pred(x[k], u[k], dt)
```

constructed as:

```python
DifferentialActionModelFreeFwdDynamics(state, actuation, costs)
IntegratedActionModelEuler(differential, dt)
```

`step()` invokes the action model's `calc()`. `rollout()` repeatedly invokes the
same Crocoddyl action model. `linearize()` invokes `calc()` followed by
`calcDiff()` and returns tangent-space `Fx` (24-by-24) and `Fu` (24-by-10).
Finite differences are used only as an independent validation experiment, not
as the derivative implementation.

P2 now calls `UAMPredictionModel.build_action_model(dt, costs)`, adding its own
costs while retaining exactly the same actuation and dynamics. Hard physical
control bounds come from `uam_actuation.yaml` for both prediction and P2.

## Hover inputs

`hover_control()` intentionally implements the specified nominal input:

```text
T_i = m_total*g/4
tau_arm = 0
```

For this model it is not a true equilibrium: the arm COM is offset and free arm
joints need gravity torque. `static_trim_control(q)` is therefore separately
provided for optimizer warm starts and diagnostics. It uses Pinocchio RNEA and
the verified mixed actuation map; it does not alter the required definition of
`hover_control()`.

## Prediction model versus plant

`f_pred != f_plant`. The prediction is a conservative rigid-body optimization
model derived from the current URDF. A future Gazebo/PX4 plant includes
additional states, controllers, delays, saturations, and modeling errors.

The current prediction omits:

- wind and unmodeled external disturbances;
- PX4 attitude/rate/thrust inner loops;
- real motor lag, rotor-speed state, and thrust identification errors;
- mechanical-arm servo position/velocity loops and damping;
- hard joint-stop reactions and dependent mimic-gripper dynamics;
- contact, friction, gripping, force control, and bulb screwing;
- D435i and other vision estimation errors.

These omissions explain why equal `mg/4` plus zero arm torque drifts strongly in
open-loop prediction while full-model static trim remains near equilibrium.

## API

```python
prediction = UAMPredictionModel(robot, actuation)
action = prediction.build_prediction_action_model(dt)
x_next = prediction.step(x, u, dt)
states = prediction.rollout(x0, controls, dt)
Fx, Fu = prediction.linearize(x, u, dt)
u_lb, u_ub = prediction.get_control_bounds()
u_hover = prediction.hover_control()
```

## Upgrade path

```text
current direct thrust/torque prediction
-> actuator first-order-lag augmented state
-> receding-horizon FDDP NMPC
-> PX4 SITL
```

Before SITL, real actuator bounds, motor ordering, arm inertial parameters,
servo-loop behavior, and coordinate conversions must be calibrated.

