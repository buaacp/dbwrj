# Local closed-loop stability around static trim

## Scope

P2.6b tests whether the existing nominal multibody prediction model can locally
stabilize selected configuration-dependent static trim points. It uses a
discrete LQR regulator only for offline model analysis. It is not PX4, a flight
controller, online NMPC, or evidence of hardware stability.

## Static balance versus stability

Static trim enforces:

```text
B u_eq = RNEA(q*, 0, 0)
ABA(q*, 0, B u_eq) ~= 0
```

This only makes `(x_eq,u_eq)` an equilibrium. It does not require nearby states
to return to that equilibrium. The current free arm has no modeled servo
damping or joint-stop reaction, so unstable arm modes can amplify numerical or
physical perturbations. This is why `fully_extended` can satisfy strict static
balance yet drift strongly under fixed `u_eq`.

## Manifold linearization

The stored state has 25 values because the free-flyer quaternion uses four
components. Its valid local perturbation has 24 values:

```text
dx = [dp_B, dtheta_B, dq_arm, dv_B, domega_B, ddq_arm]
```

All feedback errors use:

```python
dx = StateMultibody.diff(x_eq, x)
x0 = StateMultibody.integrate(x_eq, dx0)
```

No quaternion component is directly subtracted. `A_d` and `B_d` are the
24-by-24 and 24-by-10 derivatives returned by
`UAMPredictionModel.linearize()`, which itself uses Crocoddyl `calcDiff()`.

## Discrete LQR

For each strict trim point, SciPy solves the discrete algebraic Riccati
equation using YAML-defined positive-definite `Q` and `R`:

```text
du = -K dx
u = clip(u_eq + du, u_min, u_max)
```

The configured weights are marked `INITIAL_TUNING_NOT_IDENTIFIED`. They are a
conservative numerical starting point, not identified controller gains.

The open-loop spectral radius is `rho(A_d)`. A value greater than one means at
least one discrete local mode grows. The closed-loop radius is
`rho(A_d-B_d K)`; a value below one indicates asymptotic stability of the
unconstrained linear local model. Nonlinear clipped rollouts are still needed
because spectral radius alone does not prove recovery under saturation.

The implementation reports both controllability-matrix numerical rank and PBH
stabilizability. A rank-deficient controllability matrix would not by itself
prove failure: only an uncontrollable unstable or marginal mode prevents
stabilization.

## Disturbance recovery

Each selected trim point is tested for:

- 5 cm position and approximately 3 degree attitude perturbations;
- safe +/-5 degree arm-joint perturbations automatically reduced near limits;
- base linear/angular and small arm-joint velocity perturbations;
- a combined perturbation.

Open loop holds `u_eq`. Closed loop applies clipped nonlinear LQR feedback for
four seconds. Recovery requires position below 1 cm, attitude and arm-position
errors below 1 degree, and tangent velocity norm below 0.02, continuously for
the remainder of the rollout. Frequent saturation or non-finite dynamics is a
failure with an explicit reason.

## Automatic configuration selection

The validator always includes `neutral`, `fully_extended`, and `p2_terminal`,
then parses static-trim results to add the lowest rotor-margin and lowest
joint-torque-margin configurations. Duplicate names are removed. It reports
the exact rotor or joint channel instead of only the aggregate margin.

## Limitations

- The result applies only to the current URDF-derived nominal model and tested
  local perturbation sizes.
- LQR has no integral action, disturbance observer, actuator lag state, or
  robustness guarantee.
- Rotor thrust bounds, arm torque bounds, arm masses, and inertias remain
  calibration placeholders.
- The model omits motor delay, PX4 inner loops, arm servo loops, aerodynamic
  disturbances, contact, and sensor/state-estimation errors.
- Zero saturation in these tests does not establish hardware actuator margin.

Outputs are under `uam_ocp/results/local_trim_stability/`; the complete numeric
summary is in `LOCAL_TRIM_STABILITY_VALIDATION.md` and `summary.yaml`.

