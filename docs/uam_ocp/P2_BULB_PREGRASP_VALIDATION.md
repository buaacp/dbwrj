# P2.7 bulb pregrasp validation

Overall: **PASS**

- Bulb pose source: `SCENE_FILE` (simulation world, not physical measurement)
- Pregrasp distance: `0.10 m`
- IK seed: PASS, 13 iterations, `0.002852 m`, `0.010079 rad`
- Strict trim warm-start/reference nodes: `40/40`
- Dynamics rollout inconsistency: zero for all strategies
- Input saturation: none after conservative trim/control-rate regularization

| Strategy | FDDP | Position [m] | Rotation [rad] | EE linear/angular speed | Max tilt [rad] | Rotor/joint minimum margin | sum ||du||^2 |
|---|---|---:|---:|---|---:|---|---:|
| ARM_DOMINANT_SOFT_BASE_HOLD | PASS, 9 iters | 2.736e-4 | 1.053e-3 | 2.846e-3 / 9.288e-4 | 0.1693 | 0.2779 N / 0.199816 N m | 16.276 |
| UAV_DOMINANT_SOFT_ARM_HOLD | PASS, 8 iters | 2.472e-4 | 2.287e-3 | 3.130e-3 / 1.069e-3 | 0.1535 | 0.3642 N / 0.199828 N m | 14.395 |
| WHOLE_BODY | PASS, 8 iters | 1.915e-4 | 1.081e-3 | 1.946e-3 / 7.256e-4 | 0.1670 | 0.2275 N / 0.199825 N m | 15.644 |

All terminal errors satisfy the configured tolerances. None of the trajectories
passes within `0.15 rad` of the fully-extended arm configuration. Maximum
`left_knuckle_joint` torque is below `0.0002 N m`, far from its placeholder
bound, but that bound and all inertial parameters still require calibration.

The comparison demonstrates nominal-model whole-body dynamic reachability, not
contact readiness or real-flight stability. The three strategies differ only
through soft costs; none is a strict arm-only or UAV-only optimization.

