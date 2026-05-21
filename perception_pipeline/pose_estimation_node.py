#!/usr/bin/env python3
import os
import sys
import threading
from typing import Dict, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import trimesh
from scipy.spatial.transform import Rotation


class PoseEstimationNode(Node):
    def __init__(self):
        super().__init__('pose_estimation')

        self.declare_parameter('mesh_dir', 'data/tless/models_cad')
        self.declare_parameter(
            'foundationpose_path',
            os.path.expanduser('~/ros2_ws/src/FoundationPose'),
        )

        self._mesh_dir = self.get_parameter('mesh_dir').get_parameter_value().string_value
        fp_path = os.path.expanduser(
            self.get_parameter('foundationpose_path').get_parameter_value().string_value
        )

        self._bridge = CvBridge()
        self._lock = threading.Lock()

        self._meshes: Dict[int, trimesh.Trimesh] = {}
        self._estimators: Dict[int, object] = {}
        self._publishers: Dict[int, rclpy.publisher.Publisher] = {}

        self._latest_rgb: Optional[np.ndarray] = None
        self._latest_depth: Optional[np.ndarray] = None
        self._latest_K: Optional[np.ndarray] = None

        self._fp_ready = self._load_foundationpose(fp_path)

        self.create_subscription(Image, '/camera/rgb/image_raw', self._rgb_cb, 10)
        self.create_subscription(Image, '/camera/depth/image_raw', self._depth_cb, 10)
        self.create_subscription(CameraInfo, '/camera/camera_info', self._info_cb, 10)
        self.create_subscription(Detection2DArray, '/detections', self._detections_cb, 10)

        self.get_logger().info(
            f'PoseEstimationNode started (FoundationPose ready: {self._fp_ready})'
        )

    # ── FoundationPose bootstrap ──────────────────────────────────────────────

    def _load_foundationpose(self, fp_path: str) -> bool:
        if not os.path.isdir(fp_path):
            self.get_logger().warning(f'FoundationPose directory not found: {fp_path}')
            return False
        try:
            if fp_path not in sys.path:
                sys.path.insert(0, fp_path)
            from estimater import FoundationPose
            from learning.training.predict_score import ScorePredictor
            from learning.training.predict_pose_refine import PoseRefinePredictor
            import nvdiffrast.torch as dr

            self._FoundationPose = FoundationPose
            self._ScorePredictor = ScorePredictor
            self._PoseRefinePredictor = PoseRefinePredictor
            self._dr = dr
            return True
        except Exception as e:
            self.get_logger().warning(
                f'FoundationPose import failed ({e}). '
                'Node will run but pose estimation is disabled.'
            )
            return False

    # ── Sensor callbacks ──────────────────────────────────────────────────────

    def _rgb_cb(self, msg: Image) -> None:
        try:
            rgb = self._bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            with self._lock:
                self._latest_rgb = rgb
        except Exception as e:
            self.get_logger().error(f'RGB conversion error: {e}')

    def _depth_cb(self, msg: Image) -> None:
        try:
            raw = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            # T-LESS depth is in mm; convert to meters
            depth_m = raw.astype(np.float32) / 1000.0
            with self._lock:
                self._latest_depth = depth_m
        except Exception as e:
            self.get_logger().error(f'Depth conversion error: {e}')

    def _info_cb(self, msg: CameraInfo) -> None:
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        with self._lock:
            self._latest_K = K

    # ── Detection callback ────────────────────────────────────────────────────

    def _detections_cb(self, msg: Detection2DArray) -> None:
        if not self._fp_ready:
            return

        with self._lock:
            rgb = self._latest_rgb
            depth = self._latest_depth
            K = self._latest_K

        if rgb is None or depth is None or K is None:
            return

        for det in msg.detections:
            if not det.results:
                continue
            obj_id = int(det.results[0].hypothesis.class_id)

            cx = det.bbox.center.position.x
            cy = det.bbox.center.position.y
            w = det.bbox.size_x
            h = det.bbox.size_y
            x1 = max(0, int(cx - w / 2))
            y1 = max(0, int(cy - h / 2))
            x2 = min(rgb.shape[1], int(cx + w / 2))
            y2 = min(rgb.shape[0], int(cy + h / 2))

            ob_mask = np.zeros(rgb.shape[:2], dtype=bool)
            ob_mask[y1:y2, x1:x2] = True

            mesh = self._get_mesh(obj_id)
            if mesh is None:
                continue

            try:
                if obj_id not in self._estimators:
                    est = self._make_estimator(obj_id, mesh)
                    pose_mat = est.register(
                        K=K, rgb=rgb, depth=depth, ob_mask=ob_mask, ob_id=obj_id
                    )
                else:
                    est = self._estimators[obj_id]
                    pose_mat = est.track_one(rgb=rgb, depth=depth, K=K, iteration=2)
            except Exception as e:
                self.get_logger().error(
                    f'Pose estimation failed for obj {obj_id}: {e}'
                )
                self._estimators.pop(obj_id, None)
                continue

            self._get_publisher(obj_id).publish(
                self._mat_to_pose_stamped(pose_mat, msg.header)
            )

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _get_mesh(self, obj_id: int) -> Optional[trimesh.Trimesh]:
        if obj_id in self._meshes:
            return self._meshes[obj_id]
        path = os.path.join(self._mesh_dir, f'obj_{obj_id:06d}.ply')
        if not os.path.isfile(path):
            self.get_logger().error(f'Mesh not found: {path}')
            return None
        mesh = trimesh.load(path)
        mesh.vertices = mesh.vertices / 1000.0  # mm → meters
        self._meshes[obj_id] = mesh
        return mesh

    def _make_estimator(self, obj_id: int, mesh: trimesh.Trimesh):
        scorer = self._ScorePredictor()
        refiner = self._PoseRefinePredictor()
        glctx = self._dr.RasterizeCudaContext()
        est = self._FoundationPose(
            model_pts=np.array(mesh.vertices, dtype=np.float32),
            model_normals=np.array(mesh.vertex_normals, dtype=np.float32),
            mesh=mesh,
            scorer=scorer,
            refiner=refiner,
            glctx=glctx,
        )
        self._estimators[obj_id] = est
        return est

    def _get_publisher(self, obj_id: int):
        if obj_id not in self._publishers:
            topic = f'/pose_estimation/obj_{obj_id:06d}'
            self._publishers[obj_id] = self.create_publisher(PoseStamped, topic, 10)
        return self._publishers[obj_id]

    def _mat_to_pose_stamped(self, mat: np.ndarray, header) -> PoseStamped:
        ps = PoseStamped()
        ps.header = header
        ps.pose.position.x = float(mat[0, 3])
        ps.pose.position.y = float(mat[1, 3])
        ps.pose.position.z = float(mat[2, 3])
        q = Rotation.from_matrix(mat[:3, :3]).as_quat()  # [x, y, z, w]
        ps.pose.orientation.x = float(q[0])
        ps.pose.orientation.y = float(q[1])
        ps.pose.orientation.z = float(q[2])
        ps.pose.orientation.w = float(q[3])
        return ps


def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
