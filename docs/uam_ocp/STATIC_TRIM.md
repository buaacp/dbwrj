# Configuration-dependent static trim

## Purpose

`StaticTrimSolver` computes the physical rotor thrusts and six arm torques that
balance the current URDF-derived floating-base model at a specified complete
configuration `q*` and zero velocity. It reuses the verified P1 actuation matrix
and does not alter Pinocchio rigid-body dynamics.

The simple nominal input

```text
u_simple = [m_total*g/4, ..., m_total*g/4, 0, ..., 0]
```

only balances total vertical weight for a symmetric idealization. It does not
balance the gravity torque caused by the offset arm COM and does not provide
arm-joint gravity torque. It is retained as the QP bias and as the explicitly
defined `hover_control()`, not treated as the actual equilibrium.

## Static equations

For `v=0` and desired acceleration zero, Pinocchio RNEA computes:

```text
h*(q*) = RNEA(q*, 0, 0)
```

The existing P1 physical map supplies:

```text
tau_generated = B u
B shape = 12 x 10
u = [T1,T2,T3,T4,tau_arm_1,...,tau_arm_6]
```

The strict problem minimizes weighted distance from equal rotor loading subject
to `B u = h*` and all YAML actuator bounds. SciPy SLSQP receives an independent
rank-revealed subset of equality rows because the full system has 12 rows and
10 controls. The returned solution is accepted only after the complete
12-dimensional equality residual and every bound are independently checked.

RNEA defines the required static generalized force. Independent Pinocchio ABA
then evaluates:

```text
dv/dt = ABA(q*, 0, B u_eq)
```

No mass matrix, gravity, Coriolis term, or manipulator coupling term is written
by this module.

## Strict versus approximate

`strict_feasible=True` means both conditions hold:

- complete generalized-force infinity residual is below the configured
  tolerance;
- every rotor thrust and joint torque is inside its configured bound.

If strict equality is impossible, bounded `scipy.optimize.lsq_linear` minimizes
a dimensionless weighted generalized-force residual plus a small bias term. The
result may have `success=True` as an approximate numerical solution but always
has `strict_feasible=False` and an explicit status:

- `APPROXIMATE_ONLY`
- `ROTOR_SATURATION`
- `JOINT_TORQUE_SATURATION`
- `UNACTUATED_WRENCH`
- `NUMERICAL_FAILURE`

For example, a tilted body requires a lateral body-force component to oppose
gravity. The current vertical fixed rotors cannot generate that component, so
the solver correctly returns `UNACTUATED_WRENCH`, not strict equilibrium.

Other strict-infeasibility causes include rotor saturation, arm torque
saturation, an out-of-range requested configuration, or numerical/rank issues.

## Margins and rollout

The result reports each rotor's lower and upper thrust margin and each arm
channel's nearest torque-bound margin. A fixed-input 2-second rollout uses the
canonical Crocoddyl prediction model, not a separate integrator.

Static balance does not imply Lyapunov stability. The `fully_extended` test has
an ABA acceleration below numerical tolerance at its exact initial state but is
open-loop unstable in the current undamped model: roundoff grows into large arm
motion over two seconds. This is reported as a model/control limitation, not
misclassified as failure of the static equality.

## P2 integration

The default P2 mode remains:

```yaml
control_reference_mode: fixed_initial_trim
```

which preserves the previous single initial-configuration trim reference. To
enable P2.6 explicitly:

```yaml
control_reference_mode: static_trim
```

P2 then constructs its manifold reference-state sequence and computes
`u_eq(q_ref[k])` independently at every running node. Each control residual uses
that node's trim input. Warm-start controls use the same sequence. Any node that
is not strictly feasible raises an error containing the node index, status, and
residual; no silent fallback is permitted.

This is still offline BoxFDDP. It does not add online NMPC, PX4, ROS, Gazebo,
contact, or actuator dynamics.

## API

```python
solver = StaticTrimSolver(robot, actuation, prediction_model=prediction)
result = solver.solve_trim(q)
acceleration = solver.validate_with_aba(q, result.u_eq)
drift = solver.rollout_validation(q, result.u_eq, 2.0, 0.01)
margins = solver.compute_margins(result.u_eq)
```

Scenario definitions are in `uam_ocp/config/static_trim_scenarios.yaml`.
Machine-readable and plotted results are under
`uam_ocp/results/static_trim/`.

