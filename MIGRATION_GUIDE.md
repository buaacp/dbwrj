# 带机械臂无人机仿真迁移整理

本文根据当前工作区 `/home/zlhq/px4_fly_ws` 的 README、launch 文件和源码整理，用于把带机械臂无人机仿真、相机识别、控制算法和代码迁移到另一台设备。

## 1. 当前工作区内容

当前仓库是一个 Catkin 工作区，核心自有包在 `src/` 下：

| 路径 | 作用 | 迁移要求 |
| --- | --- | --- |
| `src/arm_control` | 机械臂控制、MPC 控制、实物/仿真控制 launch、无重力球 world | 必须迁移 |
| `src/camera` | RealSense D435i 图像获取、立方体识别、目标三维定位、滤波、RViz 配置、自定义 msg | 必须迁移 |
| `src/test_fly` | PX4/MAVROS 飞行测试、目标位置转发、任务状态控制、飞行日志 | 必须迁移 |
| `autostart` | 一键启动脚本 | 建议迁移 |
| `README.md` | 原始启动流程记录 | 必须迁移 |

不建议迁移的生成目录：

| 路径 | 说明 |
| --- | --- |
| `build/` | Catkin 编译产物，到新设备后重新编译 |
| `devel/` | Catkin 环境产物，到新设备后重新生成 |
| `logs/` | 旧编译/运行日志，可选备份 |
| `src/*/__pycache__`、`*.pyc` | Python 缓存，不需要迁移 |

## 2. 当前仓库之外还必须补齐的内容

README 中的关键命令依赖以下 ROS 包，但当前 `src/` 目录没有这些包：

| 包/组件 | README 中的使用方式 | 作用 | 迁移要求 |
| --- | --- | --- | --- |
| `le_arm` | `roslaunch le_arm iris_arm.launch`、`roslaunch le_arm le_arm_gazebo.launch` | 带机械臂无人机模型、机械臂控制器、Gazebo 仿真模型 | 必须从原设备找到并迁移 |
| `px4` | `roslaunch px4 indoor1.launch` | PX4 SITL/MAVROS 室内飞行仿真启动 | 必须从原设备找到并迁移或重新安装 |
| PX4-Autopilot / Firmware | 通常被 `px4` launch 间接依赖 | 飞控 SITL、机型参数、Gazebo 插件 | 必须保持版本一致 |
| Gazebo 模型路径 | `model://sun`、无人机/机械臂模型 | Gazebo 加载模型和插件 | 必须同步 `GAZEBO_MODEL_PATH`、`GAZEBO_PLUGIN_PATH` |

迁移前请在原设备执行：

```bash
rospack find le_arm
rospack find px4
echo $GAZEBO_MODEL_PATH
echo $GAZEBO_PLUGIN_PATH
echo $PX4_HOME_LAT
echo $PX4_HOME_LON
echo $PX4_HOME_ALT
```

把输出路径中的源码、模型、world、urdf/xacro/sdf、meshes、PX4 参数和启动文件一并备份。

## 3. 系统环境建议

当前代码风格和缓存显示项目可能运行在 ROS 1 + Python 3.6/3.8 环境。新设备建议尽量保持与原设备一致：

| 项目 | 建议 |
| --- | --- |
| 操作系统 | Ubuntu 18.04 + ROS Melodic，或 Ubuntu 20.04 + ROS Noetic。优先选择原设备相同版本 |
| ROS | ROS 1，包含 `rospy`、`roscpp`、`std_msgs`、`sensor_msgs`、`geometry_msgs`、`tf`、`tf2`、`tf2_ros`、`tf2_geometry_msgs`、`visualization_msgs` |
| 构建工具 | `catkin_tools` 或 `catkin_make`，当前工作区存在 `.catkin_tools` |
| 仿真 | Gazebo、`gazebo_ros`、PX4 SITL、MAVROS |
| 相机 | Intel RealSense D435i、`realsense2_camera`、`cv_bridge` |
| Python 依赖 | `numpy`、`scipy`、`opencv-python` 或系统 OpenCV、`casadi`、`pyserial`、`sympy` |

ROS 依赖包示例：

```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-mavros ros-$ROS_DISTRO-mavros-extras
sudo apt install ros-$ROS_DISTRO-gazebo-ros ros-$ROS_DISTRO-cv-bridge
sudo apt install ros-$ROS_DISTRO-realsense2-camera ros-$ROS_DISTRO-tf2-ros ros-$ROS_DISTRO-tf2-geometry-msgs
```

MAVROS 还需要安装 geographiclib 数据集：

```bash
sudo /opt/ros/$ROS_DISTRO/lib/mavros/install_geographiclib_datasets.sh
```

Python 依赖示例：

```bash
pip3 install numpy scipy casadi pyserial sympy opencv-python
```

如果使用系统 ROS 的 OpenCV/cv_bridge，优先使用 apt 安装，避免 pip OpenCV 与 ROS Python ABI 冲突。

## 4. 迁移文件清单

建议从原设备打包以下内容：

```bash
cd /home/zlhq
tar --exclude='px4_fly_ws/build' \
    --exclude='px4_fly_ws/devel' \
    --exclude='px4_fly_ws/logs' \
    --exclude='*.pyc' \
    --exclude='__pycache__' \
    -czf px4_fly_ws_src_backup.tar.gz px4_fly_ws
```

还需要单独打包：

1. `rospack find le_arm` 对应目录。
2. `rospack find px4` 对应目录。
3. PX4-Autopilot/Firmware 目录。
4. 自定义 Gazebo 模型目录和插件目录。
5. 原设备 `.bashrc` 中与 ROS、PX4、Gazebo 相关的环境变量。
6. RealSense 相关 udev 规则和相机标定/手眼标定参数。

## 5. 新设备部署步骤

1. 安装与原设备一致的 Ubuntu、ROS、Gazebo、MAVROS、RealSense 驱动和 Python 依赖。
2. 解压 `px4_fly_ws` 到新设备，例如 `/home/<user>/px4_fly_ws`。
3. 把 `le_arm`、`px4`、PX4-Autopilot/Firmware 放到与原设备一致的位置，或放入同一个 Catkin 工作区的 `src/` 中。
4. 恢复 `.bashrc` 环境变量，至少包含：

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source /home/<user>/px4_fly_ws/devel/setup.bash
export GAZEBO_MODEL_PATH=<原设备模型路径>:$GAZEBO_MODEL_PATH
export GAZEBO_PLUGIN_PATH=<原设备插件路径>:$GAZEBO_PLUGIN_PATH
```

5. 编译工作区：

```bash
cd /home/<user>/px4_fly_ws
catkin clean -y
catkin build
source devel/setup.bash
```

如果没有安装 `catkin_tools`，可用：

```bash
catkin_make
source devel/setup.bash
```

6. 确认 ROS 能找到关键包：

```bash
rospack find arm_control
rospack find camera
rospack find test_fly
rospack find le_arm
rospack find px4
rospack find realsense2_camera
```

## 6. README 中的启动流程整理

### 6.1 完整仿真流程

```bash
roslaunch le_arm iris_arm.launch
# 等 Gazebo、PX4、MAVROS、模型完全起来
roslaunch test_fly test_control.launch
roslaunch arm_control control_arm_sim.launch
```

任务开始指令：

```bash
rostopic pub /mission_state std_msgs/Int32 "data: 1"
```

### 6.2 实物流程，目标点由 Gazebo 虚拟点提供

```bash
roslaunch arm_control real_test_condition.launch
roslaunch arm_control control_arm_real.launch
```

注意：`control_arm_real.py` 会尝试打开 `/dev/ttyUSB0`，失败后尝试 `/dev/ttyUSB1`。新设备需要确认舵机串口权限和设备名。

### 6.3 单独机械臂仿真

```bash
roslaunch le_arm le_arm_gazebo.launch
roslaunch test_fly get_ball_pos.launch
roslaunch arm_control control_arm_single.launch
```

等价脚本：

```bash
bash autostart/single_arm.bash
```

### 6.4 加入视觉定位后的机械臂抓取

```bash
roslaunch camera cube_test.launch
roslaunch arm_control control_arm_real.launch
```

等价脚本：

```bash
bash autostart/vision_arm.bash
```

### 6.5 单独控制飞机

```bash
roslaunch px4 indoor1.launch
roslaunch test_fly test_control.launch
```

### 6.6 单双目相机视觉识别

```bash
roslaunch camera detect_test.launch
```

### 6.7 双目相机识别与定位

```bash
roslaunch camera cube_test.launch
```

### 6.8 手眼标定

```bash
roslaunch camera cube_test.launch
roslaunch arm_control arm_vision_biaoding.launch
```

常用检查：

```bash
rostopic echo /joint_states
rostopic echo /cube_marker
rostopic echo /filtered_cube_position
rostopic echo /object_detection/object_position_tripod
rostopic echo /weightless_ball/pose
rostopic echo /mission_state
```

## 7. 关键话题与节点关系

| 来源 | 话题/服务 | 去向 | 作用 |
| --- | --- | --- | --- |
| Gazebo | `/gazebo/model_states` | `test_fly/get_local_pose.py` | 读取无人机和无重力球状态 |
| `test_fly/get_local_pose.py` | `/weightless_ball/pose` | `arm_control` | 仿真目标点 |
| `test_fly/get_local_pose.py` | `iris_0/mavros/vision_pose/pose` | MAVROS/PX4 | 外部视觉位姿 |
| `camera/cube_position_D435i.py` | `/cube_marker` | `camera/position_filter.py` | 相机识别出的目标 marker |
| `camera/position_filter.py` | `/object_detection/object_position_tripod` | `arm_control_real.py` | 视觉目标点 |
| `arm_control` | `/le_arm_controller/command` | `le_arm` 控制器 | 机械臂关节命令 |
| 用户/`mission_control.py` | `/mission_state` | `test_fly` | 任务状态切换 |
| MAVROS | `/iris_0/mavros/cmd/arming`、`/iris_0/mavros/set_mode` | `test_fly` | 解锁和模式切换 |

## 8. 迁移后验收顺序

按从底层到完整流程的顺序检查：

1. `catkin build` 无报错，`source devel/setup.bash` 后 `rospack find` 能找到所有包。
2. `roslaunch camera cube_test.launch` 能启动 D435i，并有 `/camera/color/image_raw`、`/camera/aligned_depth_to_color/image_raw`、`/camera/color/camera_info`。
3. `roslaunch le_arm le_arm_gazebo.launch` 能打开 Gazebo，`rostopic echo /joint_states` 有机械臂关节状态。
4. `roslaunch test_fly get_ball_pos.launch` 后，`/weightless_ball/pose` 有数据。
5. `roslaunch arm_control control_arm_single.launch` 后，`/le_arm_controller/command` 有控制命令。
6. `roslaunch le_arm iris_arm.launch` 后，确认 `/iris_0/mavros/state`、`/iris_0/mavros/local_position/pose` 正常。
7. 发布 `/mission_state` 后，飞行控制流程能切换任务状态。
8. 最后再跑 README 中的完整仿真流程。

## 9. 容易遗漏的问题

1. 当前仓库不包含 `le_arm` 和 `px4`，只复制本仓库会导致 README 中多个 launch 失败。
2. `build/` 和 `devel/` 不要直接搬到新设备使用，应重新编译。
3. RealSense D435i 需要新设备有 USB 权限、udev 规则和匹配的 `realsense2_camera`。
4. `/dev/ttyUSB0`、`/dev/ttyUSB1` 在新设备可能变化，实物机械臂控制前要检查串口。
5. `GAZEBO_MODEL_PATH` 和 `GAZEBO_PLUGIN_PATH` 不一致会导致模型、控制器或 PX4 Gazebo 插件加载失败。
6. Python 依赖中 `casadi`、`scipy`、`pyserial`、`cv2` 是算法和舵机控制常见缺失项。
7. ROS 命名空间固定使用 `iris_0`、`mavros`、`/le_arm_controller/command`，如果新设备 launch 改了命名空间，控制链路会断。
8. 相机目标识别依赖颜色阈值和深度图，换光照/相机后可能需要重新调 HSV 阈值和标定参数。

## 10. 建议最终备份结构

```text
transfer_backup/
├── px4_fly_ws/                 # 当前工作区源码
├── le_arm/                     # 带机械臂无人机模型包
├── px4/                        # ROS px4 启动包
├── PX4-Autopilot/              # PX4 SITL/Firmware
├── gazebo_models/              # 自定义 Gazebo 模型
├── gazebo_plugins/             # 自定义 Gazebo 插件
├── env/
│   ├── bashrc_ros_px4_gazebo.txt
│   └── package_versions.txt
└── calibration/
    ├── realsense/
    └── hand_eye/
```

建议在原设备导出版本信息：

```bash
lsb_release -a
rosversion -d
gazebo --version
python3 --version
pip3 freeze > package_versions.txt
dpkg -l | grep -E 'ros-|gazebo|mavros|realsense' > apt_ros_gazebo_versions.txt
```
