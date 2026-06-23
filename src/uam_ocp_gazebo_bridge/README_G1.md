# G1 Gazebo static hold

G1 moves one arm configuration while PX4 holds a constant MAVROS local ENU
position and yaw. It never publishes rotor thrust, joint torque, or raw
actuator commands. Trajectory time is Gazebo `/clock`; watchdog time is
`time.monotonic()`.

After initial telemetry, G1 first moves the measured Gazebo arm configuration
to neutral through `ARM_NEUTRALIZE`. Offboard and arming requests are not sent
until neutral tracking has settled.

Build and offline tests:

```bash
catkin build uam_ocp_gazebo_bridge
PYTHONPATH=src/uam_ocp_gazebo_bridge/src python3 -m unittest discover \
  -s src/uam_ocp_gazebo_bridge/tests -v
```

User terminal 1, start the existing simulation:

```bash
source /home/zlhq/px4_fly_ws/devel/setup.bash
source /home/zlhq/PX4_Firmware/Tools/setup_gazebo.bash \
  /home/zlhq/PX4_Firmware /home/zlhq/PX4_Firmware/build/px4_sitl_default
export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:/home/zlhq/PX4_Firmware:/home/zlhq/PX4_Firmware/Tools/sitl_gazebo"
roslaunch le_arm iris_arm.launch
```

User terminal 2, run one configuration:

```bash
source /home/zlhq/px4_fly_ws/devel/setup.bash
/home/zlhq/px4_fly_ws/src/uam_ocp_gazebo_bridge/scripts/run_g1_static_hold.sh \
  --config /home/zlhq/px4_fly_ws/src/uam_ocp_gazebo_bridge/config/g1_static_hold.yaml \
  --scenario neutral
```

Repeat the second command with `left_offset`, `fully_extended`, and
`p2_terminal`. Each run writes a timestamped directory under
`results/g1_gazebo_static_hold/`.

Before rerunning, the user may perform these two finite checks (not continuous
monitoring):

```bash
rostopic info /iris_0/mavros/state
timeout 6 rostopic echo -n 1 /iris_0/mavros/state
```

If one message is received, the resolved interface exists and `neutral` can be
rerun. If no message arrives, check the MAVROS namespace, launch ordering, and
FCU connection before starting G1.

After a run has ended, analyze only its local files:

```bash
/home/zlhq/px4_fly_ws/src/uam_ocp_gazebo_bridge/scripts/analyze_g1_results.sh \
  --input /home/zlhq/px4_fly_ws/results/g1_gazebo_static_hold/<run_id> \
  --output /home/zlhq/px4_fly_ws/results/g1_gazebo_static_hold/<run_id>/analysis
```

The analysis wrapper uses the project's existing `eagle_mpc` Python environment
for Matplotlib; it does not install dependencies or connect to ROS.

The scripts never start, stop, or retry Gazebo/PX4. Missing ROS state or
another required startup topic after the 10 s grace period produces
`NOT_RUN_INTERFACE_UNAVAILABLE`, never PASS.
