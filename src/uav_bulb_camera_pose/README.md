# UAV Bulb Camera Pose

ROS1 Melodic package for RGB-only AprilTag bundle pose estimation of a lamp socket and bulb in `camera_color_optical_frame`.

The estimator subscribes only to:

- `sensor_msgs/Image`
- `sensor_msgs/CameraInfo`

It does not subscribe to PX4, MAVROS, arm control, Gazebo truth, depth, model states, or TF for the visual estimate. Gazebo truth is used only by the separate `pose_evaluator.py`.

## Implemented Interface

Default input topics match a RealSense D435i-style RGB namespace:

```bash
/camera/camera/color/image_raw
/camera/camera/color/camera_info
```

Outputs:

```bash
/socket/pose_camera
/socket/axis_camera
/socket/precision_valid
/socket/degraded_pose_valid
/socket/reprojection_error_px
/socket/visible_tag_count
/socket/inlier_corner_count
/bulb/pose_camera
/bulb/axis_camera
/bulb/precision_valid
/bulb/degraded_pose_valid
/bulb/reprojection_error_px
/bulb/visible_tag_count
/bulb/inlier_corner_count
/bulb_vision/debug_image
/bulb_vision/diagnostics
```

`PoseStamped.header.frame_id` is taken from `CameraInfo.header.frame_id`, unless `camera_frame_override` is set. In simulation it is set to `camera_color_optical_frame`.

## Algorithm

For each RGB frame:

1. Detect OpenCV AprilTag `DICT_APRILTAG_36h11`.
2. Split detections by IDs from `config/bundles.yaml`.
3. Transform all visible Tag corners from Tag frame to socket/bulb frame using `T_object_tag`.
4. Run `cv2.solvePnPRansac` over the merged visible corners.
5. Refine inliers with `cv2.solvePnPRefineLM`.
6. Compute reprojection RMSE, visible Tag count, inlier corner count, positive depth, and timestamp consistency.
7. Publish `precision_valid` only when all gates pass.

Single visible Tag uses `SOLVEPNP_IPPE_SQUARE` for a degraded pose and always publishes `precision_valid=false`.

## Build

```bash
cd /home/zlhq/px4_fly_ws
catkin build uav_bulb_camera_pose
source devel/setup.bash
```

## Generate Simulation Assets

Assets are already generated in `sim/models`, but they can be reproduced from the single YAML geometry source:

```bash
cd /home/zlhq/px4_fly_ws
PYTHONPATH=$PWD/src/uav_bulb_camera_pose:$PYTHONPATH \
python3 src/uav_bulb_camera_pose/sim/scripts/generate_tag_textures.py \
  --bundle-yaml src/uav_bulb_camera_pose/config/bundles.yaml \
  --output-dir src/uav_bulb_camera_pose/sim/models/tag_textures

PYTHONPATH=$PWD/src/uav_bulb_camera_pose:$PYTHONPATH \
python3 src/uav_bulb_camera_pose/sim/scripts/generate_scene_from_yaml.py \
  --bundle-yaml src/uav_bulb_camera_pose/config/bundles.yaml \
  --output-root src/uav_bulb_camera_pose/sim \
  --texture-dir src/uav_bulb_camera_pose/sim/models/tag_textures
```

## Run Gazebo Simulation

```bash
cd /home/zlhq/px4_fly_ws
source devel/setup.bash
roslaunch uav_bulb_camera_pose sim_perception.launch gui:=false
```

For a desktop GUI run:

```bash
roslaunch uav_bulb_camera_pose sim_perception.launch gui:=true
```

Start the independent evaluator in another terminal:

```bash
cd /home/zlhq/px4_fly_ws
source devel/setup.bash
roslaunch uav_bulb_camera_pose evaluator.launch scenario_name:=gazebo_manual
```

Inspect outputs:

```bash
rostopic echo -n1 /camera/camera/color/camera_info
rostopic echo -n1 /socket/precision_valid
rostopic echo -n1 /socket/pose_camera
rqt_image_view /bulb_vision/debug_image
```

## Offline Regression

The current checked-in CSV results were generated with:

```bash
cd /home/zlhq/px4_fly_ws
PYTHONPATH=$PWD/src/uav_bulb_camera_pose:$PYTHONPATH \
python3 src/uav_bulb_camera_pose/scripts/run_offline_regression.py \
  --bundle-yaml src/uav_bulb_camera_pose/config/bundles.yaml \
  --result-dir src/uav_bulb_camera_pose/results
```

Run unit tests:

```bash
cd /home/zlhq/px4_fly_ws
PYTHONPATH=$PWD/src/uav_bulb_camera_pose:$PYTHONPATH \
python3 -m unittest discover -s src/uav_bulb_camera_pose/test -p 'test_*.py'
```

## Real D435i Migration

1. Start `realsense2_camera` with RGB enabled.
2. Confirm actual topic names with `rostopic list`.
3. Update only `image_topic` and `camera_info_topic` in `config/real_d435i_profile.yaml` or via launch args/rosparams.
4. Confirm `CameraInfo.width`, `CameraInfo.height`, `K`, `D`, and image dimensions match.
5. Confirm output `PoseStamped.header.frame_id` is the color optical frame.
6. Record a rosbag and replay into this node before using live output.
7. Leave `use_depth_validation=false` unless using `aligned_depth_to_color/image_raw`.

Real D435i risks not fully modeled by Gazebo: auto exposure, reflections, motion blur, rolling shutter, USB bandwidth, lighting, and unreliable depth on glass/metal. This package outputs camera-frame perception only; hand-eye calibration and robot control belong to later phases.
