# G1 static hold validation status

## Current status

`CODE_AND_OFFLINE_ANALYSIS_READY`; `SIMULATION_NOT_RUN_BY_CODEX` for the revised
per-topic watchdog implementation. No new G1 PASS is claimed.

The historical second `neutral` attempt did not violate position, attitude, or
joint limits. It was stopped by the former unified 1.0 s freshness threshold
applied to MAVROS `/state`, whose observed rate was approximately 1 Hz. That
historical event is classified as `ABORTED_FALSE_STALE_STATE` for root-cause
review and is not counted as a flight-control failure. The label is not emitted
by new runs.

An older `POSITION_ERROR` emitted in `ARM_AND_OFFBOARD` while the vehicle was
still at its spawn position is classified offline as
`HISTORICAL_STARTUP_REFERENCE_BUG`; it is not a neutral hold failure. The active
runner now inserts `TAKEOFF_TRANSITION` and records both active-reference and
final-hold errors separately.

## Revised validation

- MAVROS state timeout: 2.5 s and two consecutive stale watchdog checks.
- Pose/velocity/joint state timeout: 0.5 s.
- IMU timeout: 0.3 s.
- Clock timeout: 1.0 s.
- Setpoint publish timeout: 0.2 s.
- Explicit Offboard/arming loss and physical limits remain immediate.
- PRESTREAM is permitted to be unarmed and outside Offboard.
- PRESTREAM waits up to 10 monotonic wall-clock seconds for first frames and
  never applies stale checks before a topic has been seen.
- ARM/OFFBOARD confirmation uses dedicated 8 s deadlines; it is not classified
  as MAVROS state loss while the requests are pending.

All 23 watchdog, startup-gate, arm-neutralization, takeoff-transition, trajectory, frame, config,
and offline-analysis tests are run
without starting or reading a live Gazebo/PX4 process. Numerical flight metrics
remain unavailable until the user manually executes the revised experiment and
then runs the offline analyzer.
