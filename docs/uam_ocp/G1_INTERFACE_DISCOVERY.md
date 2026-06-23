# G1 interface discovery

## Launch chain

- System: ROS1 Melodic (`rosversion 1.14.13`).
- Simulator: Gazebo Classic 9.19.0.
- Required model launch: `/home/zlhq/catkin_ws/src/le_arm/launch/iris_arm.launch`.
- Actual command: `roslaunch le_arm iris_arm.launch`.
- World: `/home/zlhq/catkin_ws/src/le_arm/worlds/weightless_ball.world`.
- The launch includes `gazebo_ros/empty_world.launch`, PX4
  `single_vehicle_with_arm_spawn.launch`, MAVROS `px4.launch`, and all arm
  ros_control spawners. It sets `/use_sim_time=true` through Gazebo.
- PX4 source commit observed at implementation time: `746b3124ab`.

The PX4 package must be visible in `ROS_PACKAGE_PATH`, and the existing
`PX4_Firmware/Tools/setup_gazebo.bash` must be sourced. No alternate model or
standalone bridge is used.

## Resolved flight interface

PX4 SITL is connected through MAVROS under `/iris_0/mavros`.

| Function | Actual interface |
|---|---|
| FCU state/mode/armed | `/iris_0/mavros/state`, `mavros_msgs/State` |
| Local pose | `/iris_0/mavros/local_position/pose`, `geometry_msgs/PoseStamped` |
| Local velocity | `/iris_0/mavros/local_position/velocity_local`, `geometry_msgs/TwistStamped` |
| Attitude/angular velocity | `/iris_0/mavros/imu/data`, `sensor_msgs/Imu` |
| Position/yaw setpoint | `/iris_0/mavros/setpoint_raw/local`, `mavros_msgs/PositionTarget` |
| Arm | `/iris_0/mavros/cmd/arming`, `mavros_msgs/CommandBool` |
| Mode | `/iris_0/mavros/set_mode`, `mavros_msgs/SetMode` |
| Simulation time | `/clock`, `rosgraph_msgs/Clock` |

The node prestreams at 30 Hz, requests `OFFBOARD`, then arms. It uses only
position and yaw fields; velocity, acceleration, and yaw-rate fields are
masked. Existing position, velocity, and acceleration MAVROS topics are
available, but G1 intentionally uses the resolved raw-local position path.

Trajectory phase time comes from `/clock`. Callback health uses independent
`time.monotonic()` arrival timestamps; low-rate `/state` never shares a timeout
with high-rate pose, velocity, IMU, or joint state.

At startup, all six interfaces have explicit first-seen gates. PRESTREAM sends
setpoints but does not require `armed` or `OFFBOARD`, and it cannot report stale.
The fixed startup-unavailable status is `NOT_RUN_INTERFACE_UNAVAILABLE`.

## Resolved arm interface

Runtime `/controller_manager/list_controllers` confirmed all controllers are
`running`:

| Joints | Controller | Command |
|---|---|---|
| shoulder_pan/lift, elbow, wrist_1 | `JointGroupVelocityController` | `/le_arm_controller/command`, `Float64MultiArray` |
| wrist_roll | `JointPositionController` | `/wrist_roll_controller/command`, `Float64` |
| left_knuckle | `JointTrajectoryController` | `/gripper_controller/command`, `JointTrajectory` |

State is `/joint_states`. Its order is runtime-dependent, so G1 maps every
sample by joint name and rejects missing joints as `CONFIGURATION_UNRESOLVED`.
The first four velocity-controlled joints use quintic velocity feedforward plus
bounded position-error feedback. The wrist is sent its quintic position, and
the knuckle remains fixed at zero while staying in Gazebo dynamics.

## Actuator observability

- MAVROS actuator/ESC topic names are registered, but no messages were emitted
  in the running launch: `MOTOR_OUTPUT_UNAVAILABLE`.
- `/joint_states.effort` exists but was identically zero and has not been
  validated as measured Gazebo torque: `JOINT_EFFORT_UNAVAILABLE`.
- Pinocchio static-trim thrusts/torques are exported only as comparison data and
  are never sent to these interfaces.

## Frames

Gazebo world and MAVROS local ROS messages are ENU. `PositionTarget` uses MAVLink
frame id `LOCAL_NED`, but MAVROS `setpoint_raw` performs the ROS ENU to PX4 NED
conversion internally. The existing flight nodes use the same local payload
convention. Therefore `frame_converter.py` performs an explicit ENU-to-ENU
identity mapping at the application boundary; conversion tests cover all unit
axes and yaw 0/+-90 degrees. No axis swap appears in the runner.

## Unresolved interfaces

- `UNRESOLVED_INTERFACE`: no validated measured motor-output stream.
- `UNRESOLVED_INTERFACE`: no validated measured joint-torque stream.
