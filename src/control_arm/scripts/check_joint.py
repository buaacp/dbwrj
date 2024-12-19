#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander

robot = RobotCommander()
group = MoveGroupCommander("arm")
joint_names = group.get_joints()  # 获取关节名称
print(joint_names)
