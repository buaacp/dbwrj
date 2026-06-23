import math

import cv2
import numpy as np

from .bundle_geometry import rot_to_quat


APRILTAG_DICTS = {
    'tag36h11': 'DICT_APRILTAG_36h11',
    'DICT_APRILTAG_36h11': 'DICT_APRILTAG_36h11',
}


class Detection(object):
    def __init__(self, tag_id, corners, margin=0.0, hamming=0):
        self.id = int(tag_id)
        self.corners = np.asarray(corners, dtype=np.float64).reshape(4, 2)
        self.decision_margin = float(margin)
        self.hamming = int(hamming)


class PoseResult(object):
    def __init__(self, bundle_name):
        self.bundle_name = bundle_name
        self.valid_pose = False
        self.precision_valid = False
        self.degraded_pose_valid = False
        self.visible_tag_count = 0
        self.inlier_corner_count = 0
        self.reprojection_rmse_px = float('inf')
        self.rvec = None
        self.tvec = None
        self.rotation_matrix = None
        self.quaternion_xyzw = None
        self.axis = None
        self.inlier_indices = []
        self.message = ''


class AprilTagBundlePoseEstimator(object):
    def __init__(self, bundles, params=None):
        self.bundles = bundles
        self.params = params or {}
        self.dictionary = self._make_dictionary(self._bundle_family())
        detector_params = cv2.aruco.DetectorParameters()
        if hasattr(detector_params, 'cornerRefinementMethod'):
            detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, detector_params)

    def _bundle_family(self):
        for bundle in self.bundles.values():
            return bundle.family
        return 'tag36h11'

    @staticmethod
    def _make_dictionary(family):
        attr = APRILTAG_DICTS.get(family, family)
        if not hasattr(cv2, 'aruco') or not hasattr(cv2.aruco, attr):
            raise RuntimeError('OpenCV aruco AprilTag dictionary is unavailable: %s' % family)
        return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, attr))

    def detect(self, image_bgr_or_gray):
        if image_bgr_or_gray.ndim == 3:
            gray = cv2.cvtColor(image_bgr_or_gray, cv2.COLOR_BGR2GRAY)
        else:
            gray = image_bgr_or_gray
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None:
            return []
        detections = []
        for c, tag_id in zip(corners, ids.flatten()):
            detections.append(Detection(int(tag_id), np.asarray(c, dtype=np.float64).reshape(4, 2)))
        return detections

    def estimate_all(self, detections, K, D):
        results = {}
        for name, bundle in self.bundles.items():
            visible = [d for d in detections if d.id in bundle.tags_by_id]
            results[name] = self.estimate_bundle(bundle, visible, K, D)
        return results

    def estimate_bundle(self, bundle, detections, K, D):
        result = PoseResult(bundle.name)
        result.visible_tag_count = len(detections)
        if not detections:
            result.message = 'no visible tags'
            return result

        object_points = []
        image_points = []
        for det in detections:
            object_points.append(bundle.object_points_for_detection(det.id))
            image_points.append(det.corners)
        object_points = np.vstack(object_points).astype(np.float64)
        image_points = np.vstack(image_points).astype(np.float64)

        if len(detections) == 1:
            return self._estimate_single_tag_degraded(bundle, detections[0], K, D, result)

        ransac_error = float(self.params.get('ransac_reprojection_error_px', 4.0))
        iterations = int(self.params.get('ransac_iterations', 100))
        ok, rvec, tvec, inliers = cv2.solvePnPRansac(
            object_points,
            image_points,
            np.asarray(K, dtype=np.float64),
            np.asarray(D, dtype=np.float64),
            iterationsCount=iterations,
            reprojectionError=ransac_error,
            confidence=0.99,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok or rvec is None or tvec is None:
            result.message = 'solvePnPRansac failed'
            return result

        if inliers is None:
            inlier_idx = np.arange(len(object_points), dtype=np.int32)
        else:
            inlier_idx = inliers.reshape(-1).astype(np.int32)
        if len(inlier_idx) >= 4 and hasattr(cv2, 'solvePnPRefineLM'):
            rvec, tvec = cv2.solvePnPRefineLM(
                object_points[inlier_idx],
                image_points[inlier_idx],
                np.asarray(K, dtype=np.float64),
                np.asarray(D, dtype=np.float64),
                rvec,
                tvec,
            )

        self._fill_pose_metrics(result, object_points, image_points, K, D, rvec, tvec, inlier_idx)
        min_tags = int(self.params.get('min_precision_visible_tags', 2))
        min_corners = int(self.params.get('min_inlier_corners', 8))
        max_rmse = float(self.params.get('max_reprojection_rmse_px', 2.5))
        min_z = float(self.params.get('min_positive_depth_m', 0.10))
        max_z = float(self.params.get('max_positive_depth_m', 5.0))
        z = float(result.tvec[2])
        result.precision_valid = (
            result.visible_tag_count >= min_tags
            and result.inlier_corner_count >= min_corners
            and result.reprojection_rmse_px < max_rmse
            and min_z < z < max_z
        )
        result.degraded_pose_valid = False
        return result

    def _estimate_single_tag_degraded(self, bundle, detection, K, D, result):
        tag = bundle.tags_by_id[detection.id]
        image_points = np.array([detection.corners[3], detection.corners[2], detection.corners[1], detection.corners[0]], dtype=np.float64)
        s = tag.size_m / 2.0
        object_points = np.array([
            [-s, s, 0.0],
            [s, s, 0.0],
            [s, -s, 0.0],
            [-s, -s, 0.0],
        ], dtype=np.float64)
        ok, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            np.asarray(K, dtype=np.float64),
            np.asarray(D, dtype=np.float64),
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not ok:
            result.message = 'single-tag IPPE_SQUARE failed'
            return result
        R_C_Tag, _ = cv2.Rodrigues(rvec)
        T_C_Tag = np.eye(4, dtype=np.float64)
        T_C_Tag[:3, :3] = R_C_Tag
        T_C_Tag[:3, 3] = np.asarray(tvec, dtype=np.float64).reshape(3)
        T_C_Object = T_C_Tag.dot(np.linalg.inv(tag.T_object_tag))
        R_C_Object = T_C_Object[:3, :3]
        rvec, _ = cv2.Rodrigues(R_C_Object)
        tvec = T_C_Object[:3, 3].reshape(3, 1)
        normal_order_obj = tag.object_corners
        normal_order_img = detection.corners
        self._fill_pose_metrics(result, normal_order_obj, normal_order_img, K, D, rvec, tvec, np.arange(4, dtype=np.int32))
        result.precision_valid = False
        result.degraded_pose_valid = True
        result.message = 'single-tag degraded pose'
        return result

    def _fill_pose_metrics(self, result, object_points, image_points, K, D, rvec, tvec, inlier_idx):
        result.valid_pose = True
        result.rvec = np.asarray(rvec, dtype=np.float64).reshape(3, 1)
        result.tvec = np.asarray(tvec, dtype=np.float64).reshape(3)
        result.inlier_indices = [int(i) for i in inlier_idx]
        result.inlier_corner_count = len(result.inlier_indices)
        R, _ = cv2.Rodrigues(result.rvec)
        result.rotation_matrix = R
        result.quaternion_xyzw = rot_to_quat(R)
        axis = R.dot(np.array([0.0, 0.0, 1.0], dtype=np.float64))
        n = np.linalg.norm(axis)
        result.axis = axis / n if n > 0.0 else axis
        if len(result.inlier_indices) > 0:
            proj, _ = cv2.projectPoints(
                object_points[result.inlier_indices],
                result.rvec,
                result.tvec.reshape(3, 1),
                np.asarray(K, dtype=np.float64),
                np.asarray(D, dtype=np.float64),
            )
            err = proj.reshape(-1, 2) - image_points[result.inlier_indices]
            result.reprojection_rmse_px = math.sqrt(float(np.mean(np.sum(err * err, axis=1))))
        result.message = 'ok'


def draw_debug(image, detections, results):
    out = image.copy()
    for det in detections:
        pts = det.corners.astype(np.int32)
        cv2.polylines(out, [pts], True, (0, 255, 0), 2)
        cv2.putText(out, str(det.id), tuple(pts[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    y = 24
    for name, res in sorted(results.items()):
        txt = '%s tags=%d inliers=%d rmse=%.2f precision=%s degraded=%s' % (
            name, res.visible_tag_count, res.inlier_corner_count, res.reprojection_rmse_px,
            res.precision_valid, res.degraded_pose_valid)
        cv2.putText(out, txt, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)
        y += 24
    return out
