# NMPC acados Runtime Notes

This workspace uses ROS1 Melodic. The default system `python3` is Python 3.6.9,
while the available `acados_template` checkout requires Python 3.8 or newer.

The validated acados runtime is:

```bash
source scripts/setup_acados_env.bash
"$ACADOS_PYTHON" scripts/check_acados_install.py
```

`$ACADOS_PYTHON` currently points to:

```text
/home/zlhq/micromamba-root/envs/eagle_mpc/bin/python
```

This is Python 3.10.20 and can import `acados_template`, `libacados.so`, `rospy`,
and the generated ROS message packages after `scripts/setup_acados_env.bash` is
sourced.

Current validation scope:

```bash
source scripts/setup_acados_env.bash
"$ACADOS_PYTHON" -m compileall src/nmpc_tracking
"$ACADOS_PYTHON" -m unittest discover -s src/nmpc_tracking/tests -v
"$ACADOS_PYTHON" src/nmpc_tracking/scripts/one_step_nmpc_smoke_test.py
"$ACADOS_PYTHON" src/nmpc_tracking/scripts/warm_start_replay_test.py
```

Do not assume `rosrun` or `roslaunch` will use Python 3.10. Before running the
Gazebo/ROS node, the Catkin and ROS Python interpreter path must be handled
explicitly. The current stage only validates no-ROS NMPC code generation and
solve behavior through `$ACADOS_PYTHON`.

The local acados build needs:

```bash
export LD_PRELOAD=/home/zlhq/acados/lib/libosqp.so
```

This is already set by `scripts/setup_acados_env.bash`.
