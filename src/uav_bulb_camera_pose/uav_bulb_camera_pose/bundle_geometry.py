import math

import numpy as np
import yaml


def normalize_quat(q):
    q = np.asarray(q, dtype=np.float64)
    n = np.linalg.norm(q)
    if n <= 0.0:
        raise ValueError('zero quaternion')
    return q / n


def quat_to_rot(q):
    x, y, z, w = normalize_quat(q)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ], dtype=np.float64)


def rot_to_quat(R):
    R = np.asarray(R, dtype=np.float64)
    tr = float(np.trace(R))
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        return normalize_quat([
            (R[2, 1] - R[1, 2]) / s,
            (R[0, 2] - R[2, 0]) / s,
            (R[1, 0] - R[0, 1]) / s,
            0.25 * s,
        ])
    i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
    if i == 0:
        s = math.sqrt(max(1.0 + R[0, 0] - R[1, 1] - R[2, 2], 0.0)) * 2.0
        q = [0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s]
    elif i == 1:
        s = math.sqrt(max(1.0 + R[1, 1] - R[0, 0] - R[2, 2], 0.0)) * 2.0
        q = [(R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s]
    else:
        s = math.sqrt(max(1.0 + R[2, 2] - R[0, 0] - R[1, 1], 0.0)) * 2.0
        q = [(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s]
    return normalize_quat(q)


def slerp(q0, q1, alpha):
    q0 = normalize_quat(q0)
    q1 = normalize_quat(q1)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    dot = max(min(dot, 1.0), -1.0)
    if dot > 0.9995:
        return normalize_quat(q0 + alpha * (q1 - q0))
    theta_0 = math.acos(dot)
    theta = theta_0 * alpha
    sin_theta = math.sin(theta)
    sin_theta_0 = math.sin(theta_0)
    s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    return normalize_quat(s0 * q0 + s1 * q1)


def pose_to_matrix(t, q):
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = quat_to_rot(q)
    T[:3, 3] = np.asarray(t, dtype=np.float64)
    return T


def matrix_to_pose(T):
    return np.asarray(T[:3, 3], dtype=np.float64), rot_to_quat(T[:3, :3])


class TagSpec(object):
    def __init__(self, tag_id, family, size_m, T_object_tag):
        self.id = int(tag_id)
        self.family = str(family)
        self.size_m = float(size_m)
        self.T_object_tag = np.asarray(T_object_tag, dtype=np.float64)

    @property
    def tag_corners(self):
        s = self.size_m / 2.0
        return np.array([
            [-s, -s, 0.0],
            [s, -s, 0.0],
            [s, s, 0.0],
            [-s, s, 0.0],
        ], dtype=np.float64)

    @property
    def object_corners(self):
        corners_h = np.hstack([self.tag_corners, np.ones((4, 1), dtype=np.float64)])
        return (self.T_object_tag.dot(corners_h.T)).T[:, :3]

    @property
    def ippe_square_object_corners(self):
        c = self.object_corners
        return np.array([c[3], c[2], c[1], c[0]], dtype=np.float64)


class BundleSpec(object):
    def __init__(self, name, frame_id, family, tags):
        self.name = name
        self.frame_id = frame_id
        self.family = family
        self.tags = tags
        self.tags_by_id = {t.id: t for t in tags}

    def object_points_for_detection(self, tag_id):
        return self.tags_by_id[int(tag_id)].object_corners


def load_bundles(path):
    with open(path, 'r') as f:
        data = yaml.safe_load(f)
    bundles = {}
    for name, entry in data['bundles'].items():
        family = entry.get('family', 'tag36h11')
        tags = []
        for t in entry.get('tags', []):
            tx, ty, tz, qx, qy, qz, qw = t['T_object_tag']
            tags.append(TagSpec(
                t['id'],
                t.get('family', family),
                t['size_m'],
                pose_to_matrix([tx, ty, tz], [qx, qy, qz, qw]),
            ))
        bundles[name] = BundleSpec(name, entry.get('frame_id', name + '_frame'), family, tags)
    return bundles
