# G1 Gazebo/PX4 static configuration hold

G1 does not replay the P2.7 trajectory. It holds one base position and yaw with
PX4 while the Gazebo arm moves neutral -> target -> neutral using simulation
time. The targets are read from P2.6 static-trim scenarios and the P2.7 terminal
trajectory, not duplicated in G1 configuration.

## State machine and safety

The implemented sequence is `IDLE -> PRESTREAM_SETPOINTS -> ARM_NEUTRALIZE -> ARM_AND_OFFBOARD ->
TAKEOFF_TRANSITION -> TAKEOFF_HOLD -> ARM_DEPLOY -> CONFIGURATION_HOLD ->
ARM_RETRACT -> FINAL_HOLD -> COMPLETE`, with `ABORT` from active flight states. Readiness must hold for two
simulation seconds. Deploy/retract uses quintic time scaling, and duration is
increased automatically if its analytical peak speed exceeds any configured
joint limit.

`PRESTREAM_SETPOINTS` starts immediately and publishes the hover setpoint while
waiting for the first MAVROS state, pose, velocity, IMU, joint-state, and clock
messages. No stale, mode, or arming fault is legal in this phase. First-seen
arrival times use `time.monotonic()` and are persisted in both metadata and the
result. If a required interface is still absent after the 10 s startup grace,
the run ends as `NOT_RUN_INTERFACE_UNAVAILABLE` with
`INITIAL_TELEMETRY_NOT_RECEIVED` (or `INITIAL_CLOCK_NOT_RECEIVED` when only the
required clock is missing); it is not treated as a flight abort.

The first complete joint sample is preserved with joint names, radians,
degrees, and neutral differences. `ARM_NEUTRALIZE` then reuses the same quintic
profile implementation to move from that measured configuration to neutral
before any Offboard or arming request. Tracking is measured against the moving
profile and uses `ARM_NEUTRALIZE_TRACKING_ERROR`; neutral must remain within 3
degrees for one simulation second before flight setup. Failure to settle within
8 seconds produces `ARM_NEUTRALIZE_TIMEOUT` and a per-joint diagnostic. During
PRESTREAM ordinary joint tracking is disabled, while URDF joint limits and
joint-state interface checks remain active.

`ARM_AND_OFFBOARD` begins only after all required first frames and the prestream
duration. It permits the expected non-Offboard/unarmed state while requesting
the transition. Separate 8 s confirmation deadlines produce
`OFFBOARD_NOT_CONFIRMED` or `ARMING_NOT_CONFIRMED`. State stale and explicit
mode/arming loss become active only after both transitions were observed.

The first complete telemetry sample defines `takeoff_start_position`. PRESTREAM
and ARM_AND_OFFBOARD hold that initial position and disable only reference
tracking error; independent absolute distance, relative altitude, attitude,
joint, and telemetry boundaries remain active. Once Offboard and arming are
confirmed, a four-second quintic `TAKEOFF_TRANSITION` moves the active reference
from the measured initial point to the configured final hold. Its tracking
failure is `TAKEOFF_TRANSITION_POSITION_ERROR`. Normal `POSITION_ERROR` against
the final hold is enabled only in TAKEOFF_HOLD and later states.

TAKEOFF_HOLD requires position below 0.05 m, speed below 0.05 m/s, roll/pitch
below 10 degrees, and neutral-arm error below 3 degrees continuously for two
simulation seconds before arm deployment.

Safety checks position error, roll/pitch, joint tracking, Offboard/armed state,
telemetry freshness, and advancing Gazebo clock. ABORT freezes deployment,
continues the same safe hover reference, requests neutral arm posture, and
writes the first trigger. It does not kill PX4 or Gazebo.

Arm reference progression uses Gazebo simulation time. Watchdog ages use
`time.monotonic()` so a paused simulation cannot freeze health checking. Each
callback owns an independent arrival timestamp. MAVROS `/state` uses a 2.5 s
timeout and two-cycle stale confirmation because it is approximately 1 Hz;
pose, velocity, IMU, and joint state retain 0.3-0.5 s high-rate limits. Explicit
mode/arming loss after takeoff and physical violations remain immediate.

## Model versus plant

Static trim gives the direct rotor thrust and joint torque that balances the
Pinocchio model. G1 must not send those values: PX4 owns motor control and the
Gazebo ros_control controllers own arm actuation. The trim is stored beside each
run only for model/plant comparison.

`fully_extended` being strictly trimmable and locally stabilizable means a
feedback law can stabilize small perturbations in the nominal model. It does
not prove PX4's cascaded controller, simulated motors, or ros_control arm can do
so. G1 measures that separate closed-loop question.

The simulation still differs from hardware in inertial calibration, motor and
servo dynamics, flex, latency, sensing, aerodynamic disturbance, and actuator
limits. No G1 result establishes real-flight stability.

## Reproducibility

Start the existing `roslaunch le_arm iris_arm.launch`, then run one scenario
with `run_g1_static_hold.sh --config ... --scenario neutral`. If Gazebo/PX4 or
`/clock` is unavailable, the status must remain
`NOT_RUN_ENVIRONMENT_UNAVAILABLE`; passing Pinocchio tests cannot substitute for
this experiment.

Each run writes `results/g1_gazebo_static_hold/<run_id>/`. Runtime logging is
periodically flushed; numerical judgment and plots are generated afterward by
`analyze_g1_results.sh`. The revised watchdog has not been run by Codex against
a live simulation, so its status is `CODE_AND_OFFLINE_ANALYSIS_READY /
SIMULATION_NOT_RUN_BY_CODEX`.
