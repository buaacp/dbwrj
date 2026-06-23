# P2.7 bulb pregrasp optimization

## Frames and target source

The bulb pose is parsed from the `light_bulb` model in
`/home/zlhq/catkin_ws/src/le_arm/worlds/weightless_ball.world`:
world position `[0.6,-0.35,0.165] m`, identity orientation. Its provenance is
`SCENE_FILE`, not a physical measurement. The bulb axis is local `-Z`, matching
the existing bulb-control convention. The gripper approach axis is EE local
`+Z`. The configured proper rotation maps gripper `+Z` onto bulb `-Z`.

With `d_pre=0.10 m`, the local offset is `-d_pre*a_bulb=[0,0,0.10]`, giving
the world pregrasp point `[0.6,-0.35,0.265] m`. No world approach direction is
hard-coded.

## Joint semantics and IK seed

The full dynamics retain all six independent joints. Five joints are active in
terminal IK: shoulder pan/lift, elbow, wrist pitch, and wrist roll.
`left_knuckle_joint` remains in Pinocchio dynamics but is fixed open at zero in
the seed and strongly penalized during optimization. A horizontal candidate
base seed is selected, then damped least-squares IK operates only on the five
declared active joints using Pinocchio frame Jacobians and exact URDF limits.

The resolved seed converged in 13 iterations with 2.85 mm position error and
0.0101 rad orientation error. Every manifold warm-start state is generated with
`StateMultibody.diff/integrate`. Every node has a strict
`StaticTrimSolver(q_ref[k])` input used for both warm start and control
regularization.

## Optimization

All strategies use the unchanged chain:

`StateMultibody -> verified mixed actuation -> FreeFwdDynamics -> Euler -> ShootingProblem -> SolverBoxFDDP`.

Terminal EE position, rotation, and world velocity costs are strongly weighted.
Physical thrust and joint-torque bounds remain active. The three comparisons
are soft-cost strategies, not locked degrees of freedom:

- `ARM_DOMINANT_SOFT_BASE_HOLD`: high base pose cost, normal arm motion.
- `UAV_DOMINANT_SOFT_ARM_HOLD`: high arm pose/velocity cost, movable base.
- `WHOLE_BODY`: balanced base and arm costs.

Standard unaugmented Crocoddyl shooting nodes cannot directly couple `u[k]` to
`u[k-1]`. P2.7 therefore uses two proximal BoxFDDP passes: the second pass
penalizes each control against the preceding control from the first solution.
The exact realized `sum ||u[k]-u[k-1]||^2` is always reported. An exact
cross-node delta-u cost requires previous-control state augmentation and remains
a documented limitation rather than being falsely claimed.

## Scope and limitations

This is free-flight pregrasp only. It contains no contact, clamping, screwing,
vision uncertainty, PX4 inner loop, motor lag, or arm servo dynamics. Results
apply only to the nominal URDF model and scene-file pose. The most constrained
configured joint remains `left_knuckle_joint`, with static margin about
`0.199846 N m`. `fully_extended` is avoided by all solutions and should not be
used as an unconstrained long-duration operating configuration despite local
LQR stabilizability.

