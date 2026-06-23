# P1 dynamics validation

Status: **PASS**

The physical control has ten channels: four rotor thrusts followed by six
independent arm torques. The resulting generalized force has dimension 12.
Crocoddyl 3.2.1's `ActuationModelFloatingBaseThrusters` reported `nu=10` and a
12-by-10 map. Its entire map was numerically equal to the YAML-derived physical
map, including all six joint identity channels, so no custom Crocoddyl
actuation class was required.

## P1-A hover

- Equal thrust from total mass: `T_i = 1.717*9.81/4 = 4.2109425 N`.
- Equal-thrust base linear acceleration norm: `0.01013 m/s^2`.
- Equal-thrust base angular acceleration norm: `0.98680 rad/s^2`.
- Cause: the arm center of mass is offset from `base_link`; equal rotor thrust
  cannot cancel its gravity moment.
- Bounded static trim thrusts:
  `[4.276921, 4.144964, 4.271350, 4.150535] N`.
- Trim base linear/angle acceleration norms:
  `1.79e-15 m/s^2`, `2.79e-14 rad/s^2`.

The equal-`mg/4` discrepancy is retained and reported rather than hidden. P2
uses the full-model static trim as its control reference.

## P1-B single rotor

For `Delta T=0.5 N`, each rotor produces `Delta Fz=0.5 N`. The measured moment
changes are stored in `single_rotor_directions.csv`. CCW rotors produce negative
body-z reaction torque and CW rotors positive body-z reaction torque, matching
`gazebo_motor_model.cpp`. The roll and pitch signs match each rotor's actual
asymmetric position.

## P1-C arm reaction

A `0.1 N m` step on `shoulder_pan_joint` changed base angular acceleration by
`[2.27016, -0.01009, 1.02649] rad/s^2`, norm `2.49147 rad/s^2`. This proves the
URDF model contains manipulator-base inertial coupling.

Artifacts:

- `uam_ocp/results/p1/p1_summary.json`
- `uam_ocp/results/p1/single_rotor_directions.csv`
- `uam_ocp/results/p1/single_rotor_moments.png`

