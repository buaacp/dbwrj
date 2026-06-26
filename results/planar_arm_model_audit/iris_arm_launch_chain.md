# iris_arm Launch Chain

- canonical launch: `/home/zlhq/catkin_ws/src/le_arm/launch/iris_arm.launch`
- robot_description xacro: `/home/zlhq/catkin_ws/src/le_arm/urdf/iris_arm_base.xacro`
- includes: `component_snippets.xacro`, `iris.xacro`, `le_arm.urdf.xacro`
- arm xacro includes: `le_arm.transmission.xacro`, `le_arm_gripper.urdf.xacro`
- controller YAML:
  - `/home/zlhq/catkin_ws/src/le_arm/controller/le_arm_controller.yaml`
  - `/home/zlhq/catkin_ws/src/le_arm/controller/wrist_roll_controller.yaml`
  - `/home/zlhq/catkin_ws/src/le_arm/controller/gripper_controller.yaml`

## Static Findings

- shoulder_pan_joint type with lock_shoulder_pan=true: `fixed`
- shoulder_pan_joint transmission: ``
- /le_arm_controller/command joints: `['shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint']`
- /wrist_roll_controller/command joint: `wrist_roll_joint`
- gripper controller joint: `['left_knuckle_joint']`
