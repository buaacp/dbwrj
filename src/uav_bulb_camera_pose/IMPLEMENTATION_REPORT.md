# Implementation Report

Date: 2026-06-19

## Environment Detected

Commands were run in `/home/zlhq/px4_fly_ws`.

```text
ROS_VERSION=1
ROS_DISTRO=melodic
rosversion -d: melodic
gazebo --version: Gazebo 9.19.0
gz --version: Gazebo 9.19.0
which gazebo: /usr/bin/gazebo
which gz: /usr/bin/gz
which roscore: /opt/ros/melodic/bin/roscore
which catkin_make: /opt/ros/melodic/bin/catkin_make
catkin_tools: 0.6.1
OpenCV: 4.12.0
cv2.aruco: available
DICT_APRILTAG_36h11: available
apriltag Python module: not installed
dt_apriltags Python module: not installed
```

Implementation choice: OpenCV `cv2.aruco` AprilTag 36h11 detector, because it is already present in the local Python 3 environment. No system packages were installed or changed.

## Delivered Files

- `config/bundles.yaml`: single geometry source for socket and bulb Tag bundles.
- `config/perception.yaml`: visual estimator gates and runtime parameters.
- `config/sim_d435i_profile.yaml`: simulated RGB profile.
- `config/real_d435i_profile.yaml`: real D435i topic/profile template.
- `uav_bulb_camera_pose/bundle_geometry.py`: YAML geometry and transform helpers.
- `uav_bulb_camera_pose/bundle_pose_core.py`: AprilTag detection and joint PnP core.
- `scripts/bundle_pose_node.py`: ROS Image + CameraInfo perception node.
- `scripts/pose_evaluator.py`: independent Gazebo truth evaluator.
- `scripts/image_impairment_node.py`: optional RGB degradation node.
- `scripts/run_offline_regression.py`: reproducible regression and CSV generator.
- `sim/models/*`, `sim/worlds/bulb_pose_test.world`: Gazebo camera, target, occluder assets.
- `test/test_*.py`: geometry, synthetic PnP, CameraInfo adaptation tests.
- `results/*.csv`, `results/*_summary.txt`: generated regression outputs.

## Verified Commands

Build:

```bash
cd /home/zlhq/px4_fly_ws
catkin build uav_bulb_camera_pose
```

Result: succeeded. Only warning was a system googletest CMake deprecation warning.

Unit tests:

```bash
PYTHONPATH=/home/zlhq/px4_fly_ws/src/uav_bulb_camera_pose:$PYTHONPATH \
python3 -m unittest discover -s src/uav_bulb_camera_pose/test -p 'test_*.py'
```

Result:

```text
Ran 4 tests in 0.036s
OK
```

Offline regression:

```bash
PYTHONPATH=/home/zlhq/px4_fly_ws/src/uav_bulb_camera_pose:$PYTHONPATH \
python3 src/uav_bulb_camera_pose/scripts/run_offline_regression.py \
  --bundle-yaml src/uav_bulb_camera_pose/config/bundles.yaml \
  --result-dir src/uav_bulb_camera_pose/results
```

Result: generated T1/T2/T3/T4/T5/T6/T9/T10 CSV and summary files under `results/`.

Launch parse:

```bash
source devel/setup.bash
roslaunch uav_bulb_camera_pose sim_perception.launch --nodes
```

Result:

```text
/gazebo
/gazebo_gui
/bundle_pose_node
```

Headless Gazebo smoke run:

```bash
source devel/setup.bash
timeout 25s roslaunch uav_bulb_camera_pose sim_perception.launch gui:=false
```

Result: `gzserver`, `gazebo_ros_camera`, and `bundle_pose_node` started. The command exited with code 124 because of the intentional 25 second timeout.

Local model SDF checks:

```bash
for f in src/uav_bulb_camera_pose/sim/models/*/model.sdf; do gz sdf -k "$f"; done
```

Result: all local model SDF files returned `rc=0`.

World-level `gz sdf -k src/uav_bulb_camera_pose/sim/worlds/bulb_pose_test.world` did not return promptly and was killed; this is recorded as not verified.

## Regression Summary

Offline regression uses synthetic projected Tag corners generated from `bundles.yaml`; it verifies the PnP chain and quality gates, but it is not a substitute for full rendered Gazebo image evaluation.

Key results:

```text
T1_static_unoccluded:
  socket valid_ratio=1.000000, median position=0.000033 m, median axis=0.014638 deg
  bulb   valid_ratio=1.000000, median position=0.000091 m, median axis=0.030752 deg

T3_socket_one_tag_occluded:
  socket valid_ratio=1.000000

T4_socket_two_tags_occluded:
  socket valid_ratio=1.000000

T5_socket_single_tag:
  socket valid_ratio=0.000000, false_valid_ratio=0.000000

T6_socket_all_occluded:
  socket valid_ratio=0.000000, false_valid_ratio=0.000000

T9_resolution_1280x720:
  socket valid_ratio=1.000000
  bulb   valid_ratio=1.000000

T10_noise_blur_proxy:
  socket valid_ratio=1.000000
  bulb   valid_ratio=1.000000
```

## Gazebo 在线 T1 实测

Run directory:

```text
results/gazebo_online/T1_static_clear_20260619_224543/
```

Artifacts:

```text
raw_image.png
debug_image.png
T1_static_clear_20260619_224543.csv
T1_static_clear_20260619_224543_summary.txt
summary.csv
key_topic_dump.txt
run_commands.txt
```

Commands actually used are recorded in `run_commands.txt`. The evaluator was launched with:

```bash
roslaunch uav_bulb_camera_pose evaluator.launch \
  scenario_name:=T1_static_clear_20260619_224543 \
  result_dir:=/home/zlhq/px4_fly_ws/src/uav_bulb_camera_pose/results/gazebo_online/T1_static_clear_20260619_224543
```

Result: **T1 did not pass**. No result is fabricated as a pass.

Measured topic rates and status from `key_topic_dump.txt`:

```text
RGB image rate:             ~30.12 Hz on /camera/camera/color/image_raw
socket pose output rate:    ~9.50 Hz on /socket/pose_camera
bulb pose output rate:      no new messages during the dump window
socket visible_tag_count:   1
bulb visible_tag_count:     2
socket inlier_corner_count: 4
bulb inlier_corner_count:   0
socket RMSE:                0.0148 px
bulb RMSE:                  -1.0 in topic dump; diagnostics reported solvePnPRansac failed
socket precision_valid:     false
bulb precision_valid:       false
```

Evaluator summary from `summary.csv`:

```text
socket samples:                257
socket position RMSE:          0.150018565 m
socket position mean/max:      0.150018565 / 0.150018565 m
socket rotation RMSE:          179.999271 deg
socket rotation mean/max:      179.999271 / 179.999271 deg
socket axis angle RMSE:        0.039961 deg
socket axis angle mean/max:    0.039961 / 0.039961 deg
socket precision_valid ratio:  0.0
bulb samples:                  0, because /bulb/pose_camera was not published in the dump/evaluator interval
```

Observed behavior:

- Gazebo did publish real rendered RGB images at about 30 Hz.
- `bundle_pose_node` processed Gazebo images and detected real rendered AprilTags; the debug image and diagnostics were produced from the rendered image path.
- The visual estimator did not reach precision-valid output for either object.
- Socket was single-Tag degraded only, so `precision_valid=false` by design.
- Bulb had two visible Tags in diagnostics, but `solvePnPRansac` failed in the final T1 layout.
- No jump/loss analysis can be claimed for precision-valid poses because there were no precision-valid poses.

Root causes found and fixed during the T1 attempt:

- ROS Melodic `cv_bridge` is Python 2 while the OpenCV AprilTag backend runs under Python 3. The node now uses local `sensor_msgs/Image` to NumPy conversion instead of `cv_bridge`.
- Initial Gazebo camera looked along link +X while the targets were placed along world +Z. The T1 world now places targets in front of the Gazebo camera, and evaluator applies the Gazebo camera-link to optical-frame transform internally.
- Initial Gazebo material paths and plane UVs did not render real detectable Tag textures. The generator now creates RGB textures with white border and Collada meshes with explicit UV coordinates.
- Initial per-Tag SDF poses used RPY converted from `T_object_tag`, which hit gimbal singularities and made Gazebo geometry disagree with `bundles.yaml`. The generator now writes Tag mesh vertices already transformed by `T_object_tag`, so Gazebo and PnP share the same YAML geometry.
- `CameraInfo` was 30 Hz, but the 1920x1080 AprilTag processing callback lagged behind the newest CameraInfo. The node now keeps a CameraInfo timestamp buffer and selects the closest CameraInfo for each image.

Remaining failure for T1:

- The current static clear camera/object layout still gives insufficient accepted Tag geometry: socket sees only one stable Tag and bulb PnP fails with two visible Tags. This is a scene/view-layout problem, not a PX4/arm/control issue.
- Minimal next fix: create a dedicated T1 camera pose or target yaw where at least two socket Tags and two bulb Tags are front-facing and fully inside the image, then rerun only T1. Keep the Collada UV mesh path and CameraInfo buffering changes.
- Alternative minimal fix: add a small scripted T1 world generator sweep over object yaw/viewpoint, using the existing evaluator to select a static clear pose with `visible_tag_count >= 2`, `inlier_corner_count >= 8`, and RMSE below 2.5 px.

## 第二次 T1（修复后）

Run directory:

```text
results/gazebo_online/T1_static_clear_fixed_20260619_232342/
```

Artifacts:

```text
raw_image.png
debug_image.png
T1_static_clear_fixed_20260619_232342.csv
summary.csv
key_topic_dump.txt
run_commands.txt
geometry_consistency.csv
geometry_consistency.md
visibility_report.csv
visibility_report.md
layout_probe_front_facing/pnp_debug.json
```

This run only covers T1. T2-T11 were not started.

Changes made before this run:

- Added `check_bundle_geometry_consistency.py`, which checks the generated Gazebo Collada Tag visual geometry against `bundles.yaml`.
- Added `capture_pnp_debug_frame.py`, which records one real rendered RGB frame, detections, image corners, object points, single-Tag IPPE results, joint PnP results, and Gazebo truth for debugging only.
- Kept the visual estimator interface unchanged: it still subscribes only to RGB `Image` and `CameraInfo`; Gazebo truth is used only by the evaluator/debug scripts.
- Fixed `pose_evaluator.py` shutdown and latch initialization so rows are not written before quality metrics arrive.
- Updated `bundles.yaml` and regenerated Gazebo meshes from it. The T1 static clear layout now uses a front-facing 4-Tag rigid pattern for each object so Gazebo and PnP share exactly the same `T_object_tag` geometry and the static camera can see all Tags clearly.

Geometry consistency:

```text
geometry_consistency.md:
translation_error_m ~= 0 for all socket/bulb Tags
rotation_error_deg  = 0 for all socket/bulb Tags
Tag front normal is local +Z.
PnP corner order is [-x,-y], [+x,-y], [+x,+y], [-x,+y] in Tag frame.
OpenCV aruco corner order is top-left, top-right, bottom-right, bottom-left in the rendered Tag image.
```

Visibility report:

```text
socket: 4/4 Tags detected, mean pixel edge about 116 px, viewing angle 6.85-15.90 deg
bulb:   4/4 Tags detected, mean pixel edge about 70 px,  viewing angle 11.47-16.66 deg
decision_margin/hamming: OpenCV aruco backend does not expose real decision margin/hamming here, so debug records 0.0/0.
```

Topic rates from `key_topic_dump.txt`:

```text
RGB image rate:          ~30.10 Hz
socket pose rate:        ~4.96 Hz
bulb pose rate:          ~5.04 Hz
debug image rate:        ~5.02 Hz
```

Online diagnostics from the final T1 run:

```text
socket_visible_tag_count:   4
socket_inlier_corner_count: 8
socket_rmse_px:             0.317546
socket_precision_valid:     True

bulb_visible_tag_count:     4
bulb_inlier_corner_count:   8
bulb_rmse_px:               0.120685
bulb_precision_valid:       True
```

Evaluator summary from `summary.csv`:

```text
socket samples:                 145
socket position mean/RMSE/max:  0.049585031 / 0.049585031 / 0.049585031 m
socket rotation mean/RMSE/max:  0.151561 / 0.151561 / 0.151561 deg
socket axis mean/RMSE/max:      0.151559 / 0.151559 / 0.151559 deg
socket precision_valid ratio:   1.0
socket visible Tag min/mean:    4 / 4
socket inlier corner min/mean:  8 / 8

bulb samples:                   145
bulb position mean/RMSE/max:    0.029666400 / 0.029666400 / 0.029666400 m
bulb rotation mean/RMSE/max:    0.858955 / 0.858955 / 0.858955 deg
bulb axis mean/RMSE/max:        0.858954 / 0.858954 / 0.858954 deg
bulb precision_valid ratio:     1.0
bulb visible Tag min/mean:      4 / 4
bulb inlier corner min/mean:    8 / 8
```

Pass/fail:

```text
T1_static_clear_fixed_20260619_232342: PASS
```

No jumps or losses were observed in the fixed static run: every evaluator row for socket and bulb had `precision_valid=1`, `visible_tag_count=4`, and `inlier_corner_count=8`. The output rate is about 5 Hz because 1920x1080 AprilTag detection is CPU-bound in this Python/OpenCV implementation.

## Implemented Requirements

- RGB + `CameraInfo` only estimator interface.
- Dynamic K, D, width, height, frame_id from `CameraInfo`.
- Configurable input topics and frame override.
- `tag36h11` socket IDs 100-103 and bulb IDs 200-203.
- Single `bundles.yaml` source for PnP geometry, simulation generation, and tests.
- Joint multi-Tag PnP with `solvePnPRansac`, `solvePnPRefineLM`, and reprojection RMSE gate.
- Single Tag degraded mode with `SOLVEPNP_IPPE_SQUARE`; never precision valid.
- Required Pose, axis, visible count, inlier count, RMSE, validity, debug image, and diagnostics topics.
- Independent evaluator that uses `/gazebo/model_states`; estimator does not use this truth.
- Optional image impairment node for noise, blur, brightness/contrast, delay, drop, and JPEG degradation.
- Real D435i profile and migration notes.

## Not Fully Completed

- Full rendered Gazebo T1-T11 data collection was not completed in this run. The package contains the Gazebo world, models, camera plugin, evaluator, and launch files, and a 25 second headless smoke run started successfully. CSV results currently checked in are offline synthetic regressions, not rendered Gazebo evaluator CSVs.
- T7, T8, and T11 are scaffolded by the evaluator/regression framework but do not yet have moving Gazebo scenario controllers.
- `gz sdf -k` on the world file hung while resolving includes; local model SDF files were validated instead.
- Debug video capture was not produced; debug image topic is implemented.

## Reproducible Commands

```bash
# 1. Build
cd /home/zlhq/px4_fly_ws
catkin build uav_bulb_camera_pose
source devel/setup.bash

# 2. Regenerate textures and Gazebo scene from bundles.yaml
PYTHONPATH=$PWD/src/uav_bulb_camera_pose:$PYTHONPATH python3 src/uav_bulb_camera_pose/sim/scripts/generate_tag_textures.py --bundle-yaml src/uav_bulb_camera_pose/config/bundles.yaml --output-dir src/uav_bulb_camera_pose/sim/models/tag_textures
PYTHONPATH=$PWD/src/uav_bulb_camera_pose:$PYTHONPATH python3 src/uav_bulb_camera_pose/sim/scripts/generate_scene_from_yaml.py --bundle-yaml src/uav_bulb_camera_pose/config/bundles.yaml --output-root src/uav_bulb_camera_pose/sim --texture-dir src/uav_bulb_camera_pose/sim/models/tag_textures

# 3. Start Gazebo + perception
roslaunch uav_bulb_camera_pose sim_perception.launch gui:=false

# 4. Start evaluator in another terminal
roslaunch uav_bulb_camera_pose evaluator.launch scenario_name:=gazebo_manual

# 5. View debug and topics
rostopic echo -n1 /camera/camera/color/camera_info
rostopic echo -n1 /socket/pose_camera
rostopic echo -n1 /bulb/pose_camera
rqt_image_view /bulb_vision/debug_image

# 6. Run regression
PYTHONPATH=$PWD/src/uav_bulb_camera_pose:$PYTHONPATH python3 -m unittest discover -s src/uav_bulb_camera_pose/test -p 'test_*.py'
PYTHONPATH=$PWD/src/uav_bulb_camera_pose:$PYTHONPATH python3 src/uav_bulb_camera_pose/scripts/run_offline_regression.py --bundle-yaml src/uav_bulb_camera_pose/config/bundles.yaml --result-dir src/uav_bulb_camera_pose/results
```

## Real D435i Migration Notes

Only launch parameters and YAML profiles should change for a real camera. The PnP and detection core must stay unchanged. Start RGB from `realsense2_camera`, confirm the actual namespace with `rostopic list`, set `image_topic` and `camera_info_topic`, verify `CameraInfo`, and replay a rosbag first.

Depth is intentionally not used by the main estimator. If later enabled, only use aligned depth to color or an explicit depth-to-color transform.
