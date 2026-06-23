#!/usr/bin/env python3
import argparse
import csv
import math
import os
import re
import sys

import numpy as np
import rospy
from gazebo_msgs.msg import ModelStates

from uav_bulb_camera_pose.bundle_geometry import load_bundles, matrix_to_pose, pose_to_matrix, rot_to_quat


def pose_msg_to_matrix(pose):
    p = pose.position
    q = pose.orientation
    return pose_to_matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])


def rotation_error_deg(A, B):
    R = A[:3, :3].T.dot(B[:3, :3])
    v = (np.trace(R) - 1.0) / 2.0
    return math.degrees(math.acos(max(min(float(v), 1.0), -1.0)))


def mesh_pose_from_dae(path):
    text = open(path, 'r').read()
    m = re.search(r'<float_array id="positions-array" count="12">([^<]+)</float_array>', text)
    if not m:
        raise RuntimeError('positions array not found in %s' % path)
    vals = np.array([float(x) for x in m.group(1).split()], dtype=np.float64).reshape(4, 3)
    center = np.mean(vals, axis=0)
    x_axis = vals[1] - vals[0]
    y_axis = vals[3] - vals[0]
    x_axis = x_axis / np.linalg.norm(x_axis)
    y_axis = y_axis / np.linalg.norm(y_axis)
    z_axis = np.cross(x_axis, y_axis)
    z_axis = z_axis / np.linalg.norm(z_axis)
    y_axis = np.cross(z_axis, x_axis)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.column_stack([x_axis, y_axis, z_axis])
    T[:3, 3] = center
    return T


def model_pose(states, name):
    if name not in states.name:
        return None
    return pose_msg_to_matrix(states.pose[states.name.index(name)])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bundle-yaml', required=True)
    parser.add_argument('--model-root', required=True)
    parser.add_argument('--output-csv', required=True)
    parser.add_argument('--output-md', required=True)
    parser.add_argument('--wait-gazebo', action='store_true')
    args = parser.parse_args()

    bundles = load_bundles(args.bundle_yaml)
    states = None
    if args.wait_gazebo:
        rospy.init_node('check_bundle_geometry_consistency', anonymous=True)
        states = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=10.0)

    model_names = {'socket': 'socket_tag_ring', 'bulb': 'bulb_tag_ring'}
    rows = []
    for bundle_name, bundle in sorted(bundles.items()):
        model_name = model_names[bundle_name]
        T_W_bundle = model_pose(states, model_name) if states is not None else np.eye(4)
        for tag in bundle.tags:
            dae = os.path.join(args.model_root, model_name, 'meshes', 'tag_%03d.dae' % tag.id)
            T_bundle_tag_visual = mesh_pose_from_dae(dae)
            T_bundle_tag_yaml = tag.T_object_tag
            T_W_tag_visual = T_W_bundle.dot(T_bundle_tag_visual)
            T_W_tag_yaml = T_W_bundle.dot(T_bundle_tag_yaml)
            dT = np.linalg.inv(T_bundle_tag_yaml).dot(T_bundle_tag_visual)
            trans_err = float(np.linalg.norm(dT[:3, 3]))
            rot_err = rotation_error_deg(T_bundle_tag_yaml, T_bundle_tag_visual)
            p_vis, q_vis = matrix_to_pose(T_W_tag_visual)
            p_yaml, q_yaml = matrix_to_pose(T_bundle_tag_yaml)
            rows.append({
                'bundle': bundle_name,
                'tag_id': tag.id,
                'gazebo_model': model_name,
                'gazebo_visual_pose_world_xyz_xyzw': '%.9f %.9f %.9f %.9f %.9f %.9f %.9f' % (
                    p_vis[0], p_vis[1], p_vis[2], q_vis[0], q_vis[1], q_vis[2], q_vis[3]),
                'bundle_yaml_pose_xyz_xyzw': '%.9f %.9f %.9f %.9f %.9f %.9f %.9f' % (
                    p_yaml[0], p_yaml[1], p_yaml[2],
                    tag.T_object_tag[0, 0] * 0.0 + matrix_to_pose(tag.T_object_tag)[1][0],
                    matrix_to_pose(tag.T_object_tag)[1][1],
                    matrix_to_pose(tag.T_object_tag)[1][2],
                    matrix_to_pose(tag.T_object_tag)[1][3]),
                'translation_error_m': trans_err,
                'rotation_error_deg': rot_err,
                'tag_front_normal_bundle_xyz': '%.9f %.9f %.9f' % tuple(T_bundle_tag_visual[:3, :3].dot(np.array([0.0, 0.0, 1.0]))),
                'corner_order': 'OpenCV aruco corners are top-left, top-right, bottom-right, bottom-left in the rendered tag image; YAML tag local points are [-x,-y], [+x,-y], [+x,+y], [-x,+y] on z=0 before T_object_tag.',
            })

    os.makedirs(os.path.dirname(args.output_csv), exist_ok=True)
    with open(args.output_csv, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    with open(args.output_md, 'w') as f:
        f.write('# Bundle Geometry Consistency\n\n')
        f.write('Tag front normal is local +Z. Object/PnP corner order is `[-x,-y]`, `[+x,-y]`, `[+x,+y]`, `[-x,+y]` in Tag frame.\n\n')
        f.write('| bundle | tag_id | translation_error_m | rotation_error_deg | front_normal_bundle |\n')
        f.write('|---|---:|---:|---:|---|\n')
        for r in rows:
            f.write('| {bundle} | {tag_id} | {translation_error_m:.9g} | {rotation_error_deg:.9g} | `{tag_front_normal_bundle_xyz}` |\n'.format(**r))
    print(args.output_csv)
    print(args.output_md)


if __name__ == '__main__':
    main()
