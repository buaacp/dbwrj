# Static trim validation

Overall: **PASS**

- Strict feasible: `17 / 17`
- Skipped invalid configurations: `0`
- Maximum generalized-force infinity residual: `3.552714e-15`
- Maximum ABA base linear acceleration: `3.553643e-15 m/s^2`
- Maximum ABA base angular acceleration: `1.165010e-14 rad/s^2`
- Maximum ABA joint acceleration norm: `7.383580e-13 rad/s^2`
- Maximum two-second rollout position drift: `6.072912e-02 m`
- Minimum rotor thrust margin: `3.990211e+00 N`
- Minimum joint torque margin: `1.998464e-01 N m`

All strict solutions were independently checked with Pinocchio ABA and rolled out through the canonical Crocoddyl prediction model. A strict result means the full 12-dimensional equality and bounds passed; approximate solutions would not be counted as strict.

Static equality is not a stability guarantee. The longest rollout drift occurs at the undamped fully-extended equilibrium: numerical perturbations excite arm motion even though its initial RNEA residual and ABA acceleration pass strict tolerances.

## P2 static-trim reference mode

- Status: **PASS**
- Strict trim references: `40 / 40`
- BoxFDDP converged: `True` in `3` iterations
- Dynamics rollout state error: `0.000000e+00`
