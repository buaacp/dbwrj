# UAM model discovery

## Source chain

- Simulation launch: `/home/zlhq/catkin_ws/src/le_arm/launch/iris_arm.launch`
- Robot-description entry point: `/home/zlhq/catkin_ws/src/le_arm/urdf/iris_arm_base.xacro`
- Airframe mechanics: `/home/zlhq/catkin_ws/src/le_arm/urdf/iris.xacro`
- Arm mechanics: `/home/zlhq/catkin_ws/src/le_arm/urdf/le_arm.urdf.xacro`
- Gripper mechanics: `/home/zlhq/catkin_ws/src/le_arm/urdf/le_arm_gripper.urdf.xacro`
- Rotor plugin macro: `/home/zlhq/catkin_ws/src/le_arm/urdf/multirotor_base.xacro`
- PX4/Gazebo motor reaction sign: `/home/zlhq/PX4_Firmware/Tools/sitl_gazebo/src/gazebo_motor_model.cpp`
- MoveIt semantics: `/home/zlhq/catkin_ws/src/le_arm_moveit_config/config/le_arm.srdf`
- Selected optimization input: `uam_ocp/models/generated/uam_pinocchio.urdf`
- Reproducible generator: `uam_ocp/scripts/build_pinocchio_model.py`

The generator expands the source Xacro. It does not copy the reference S500
URDF. Gazebo tags and transmissions are removed because Pinocchio does not use
them; inertial tags and kinematic origins are preserved.

## Frames and dimensions

- Floating-base body frame: `base_link`
- Massive body link: `base_link_inertia`, fixed to `base_link`
- Mechanical-arm root link: `arm_base_link`, mounted at `[-0.1, 0, -0.05]`
  with RPY `[0, 3.14, 3.14]` relative to `base_link`
- End-effector frame: `gripper_base_link`; confirmed by the MoveIt SRDF chain
  and end-effector declaration
- Independent optimization joints, in Pinocchio order:
  `shoulder_pan_joint`, `shoulder_lift_joint`, `elbow_joint`,
  `wrist_1_joint`, `wrist_roll_joint`, `left_knuckle_joint`
- Expected and observed: `n_a=6`, `nq=7+n_a=13`, `nv=6+n_a=12`
- Crocoddyl state: `nx=25`, tangent dimension `ndx=24`

The raw expanded URDF has `nq=30`, `nv=24`: it incorrectly treats four rotor
spin joints, zero-range mounting joints, and five mimic joints as independent
dynamic coordinates. The optimization generator fixes rotor joints because
rotor thrust is modeled as a wrench, fixes zero-range joints, and fixes mimic
joints at the open pre-grasp configuration. The Gazebo-continuous wrist roll is
converted to a scalar revolute coordinate with a `[-pi, pi]` placeholder range
to preserve the requested `nq=7+n_a` form. These reductions are P0-P2 modeling
choices, not claims about the hardware.

## Rotor configuration

All coordinates are expressed in `base_link`; thrust direction is body `+z`.

| ID | Name | Position [m] | Gazebo spin | Reaction yaw sign |
|---:|---|---|---|---:|
| 0 | front_right | `[0.13,-0.22,0.023]` | CCW | negative |
| 1 | back_left | `[-0.13,0.20,0.023]` | CCW | negative |
| 2 | front_left | `[0.13,0.22,0.023]` | CW | positive |
| 3 | back_right | `[-0.13,-0.20,0.023]` | CW | positive |

The Xacro gives `motorConstant=8.54858e-06`, `maxRotVelocity=1100 rad/s`, and
`momentConstant=0.06 m`; therefore the Gazebo maximum thrust is
`8.54858e-06 * 1100^2 = 10.3437818 N`. This is a simulator parameter, marked
`TODO_NEEDS_CALIBRATION` for real hardware. Motor numbering and spin were read
directly from `iris.xacro`, not inferred from S500.

## Mass and inertia inventory

Expanded-link masses are:

| Group/link | Mass [kg] |
|---|---:|
| base_link_inertia | 1.500 |
| four rotor links | 0.020 total |
| IMU link | 0.015 |
| arm_base_link | 0.040 |
| shoulder_link | 0.025 |
| upper_arm_link | 0.020 |
| forearm_link | 0.025 |
| wrist_1_link | 0.020 |
| wrist_roll_link | 0.005 |
| gripper_base_link | 0.025 |
| six gripper moving links | 0.022 total |
| **Total** | **1.717** |

Every massive source link has a positive mass and positive-definite diagonal
inertia. `base_link` itself has no inertial tag, but is fixed to
`base_link_inertia`; Pinocchio merges that inertia. All arm Gazebo blocks use
`turnGravityOff=false`; gravity is not disabled.

## Risks and calibration backlog

- `TODO_NEEDS_CALIBRATION`: arm link masses are only 5-40 g. No bill of
  materials or identified inertial source was found; physical credibility is
  unknown even though the tensors are numerically valid.
- `TODO_NEEDS_CALIBRATION`: arm URDF effort limits are `1000 N m`, physically
  implausible for this arm. P1/P2 use explicit conservative YAML placeholders.
- `TODO_NEEDS_CALIBRATION`: `wrist_roll_joint` is continuous; its optimization
  interval is a single-turn placeholder.
- `TODO_NEEDS_CALIBRATION`: mimic gripper dynamics are frozen for P0-P2.
- Rotor inertia in the Gazebo Xacro is multiplied by
  `rotor_velocity_slowdown_sim=10`; it is simulation-oriented, not an identified
  physical rotor inertia.
- Rotor geometry is asymmetric front/back (`0.22` versus `0.20 m` in y), so
  motor mixing must use the actual table rather than a symmetric X frame.
- The arm is offset from the body origin; equal `mg/4` thrust is not an exact
  attitude equilibrium.
- PX4 motor ID correspondence is explicit in the loaded Xacro and the Gazebo
  command topic, but no hardware wiring/mixer calibration was found. Hardware
  correspondence remains a P3 blocking item.

## Available software

- Default ROS Python 3.6: no Pinocchio/Crocoddyl/matplotlib/pytest
- Existing environment: `/home/zlhq/micromamba-root/envs/eagle_mpc/bin/python`
- Python 3.10, Pinocchio 4.0.0, Crocoddyl 3.2.1, NumPy 2.2.6,
  SciPy 1.15.2, matplotlib 3.10.9, PyYAML 6.0.3
- pytest is absent; compatible tests were actually run with `unittest`

