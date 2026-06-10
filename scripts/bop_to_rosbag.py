import rclpy # ros depedencies 
from rclpy.serialization import serialize_message #ros messages conversion
import rosbag2_py # for writing ROS 2 bags
from sensor_msgs.msg import Image, CameraInfo # image camera info
from geometry_msgs.msg import PoseStamped # for ground truth poses
from cv_bridge import CvBridge # for converting OpenCV images to ROS messages
from scipy.spatial.transform import Rotation # for converting rotation matrices to quaternions
import cv2, json, numpy as np, os # for file handling and data processing
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

# importing cv bridge
bridge = CvBridge()

# target object
TARGET_OBJ_ID = 25

# Define paths
base = os.path.expanduser('~/ros2_ws/src/perception_pipeline/data')
scene_path = f"{base}/tless/test_primesense/000001"
output_path = f"{base}/bags/tless_scene1"

# Load camera parameters and ground truth poses
with open(f"{scene_path}/scene_camera.json") as f:
    cameras = json.load(f)

# Load ground truth bbox detections
with open(f"{scene_path}/scene_gt_info.json") as f:
    bboxes = json.load(f)

# Load ground truth poses
with open(f"{scene_path}/scene_gt.json") as f:
    ground_truth = json.load(f)

# setup writer
writer = rosbag2_py.SequentialWriter()

storage_options = rosbag2_py.StorageOptions(uri=output_path, storage_id='sqlite3')
converter_options = rosbag2_py.ConverterOptions('', '')
writer.open(storage_options, converter_options)

# register topics
writer.create_topic(rosbag2_py.TopicMetadata(
    name='/camera/rgb/image_raw', type='sensor_msgs/msg/Image', serialization_format='cdr'))
writer.create_topic(rosbag2_py.TopicMetadata(
    name='/camera/depth/image_raw', type='sensor_msgs/msg/Image', serialization_format='cdr'))
writer.create_topic(rosbag2_py.TopicMetadata(
    name='/camera/camera_info', type='sensor_msgs/msg/CameraInfo', serialization_format='cdr'))
writer.create_topic(rosbag2_py.TopicMetadata(
    name='/gt_detections', type='vision_msgs/msg/Detection2DArray', serialization_format='cdr'))
writer.create_topic(rosbag2_py.TopicMetadata(
    name='/gt_instance_mask', type='sensor_msgs/msg/Image', serialization_format='cdr'))
writer.create_topic(rosbag2_py.TopicMetadata(
    name='/gt_pose',
    type='geometry_msgs/msg/PoseStamped',
    serialization_format='cdr'))
writer.create_topic(rosbag2_py.TopicMetadata(
    name='/camera_pose',
    type='geometry_msgs/msg/PoseStamped',
    serialization_format='cdr'))

# Process each image and write to the bag
for img_id_str, cam_data in cameras.items():
    img_id = int(img_id_str)
    t_ns = int(img_id * 0.033 * 1e9)  # nanoseconds

    # RGB
    rgb = cv2.imread(f"{scene_path}/rgb/{img_id:06d}.png")
    rgb_msg = bridge.cv2_to_imgmsg(rgb, encoding="bgr8")
    rgb_msg.header.stamp.sec = int(img_id * 0.033)
    rgb_msg.header.stamp.nanosec = int((img_id * 0.033 % 1) * 1e9)
    writer.write('/camera/rgb/image_raw', serialize_message(rgb_msg), t_ns)

    # Depth
    depth = cv2.imread(f"{scene_path}/depth/{img_id:06d}.png", cv2.IMREAD_UNCHANGED)
    depth_scale = cam_data["depth_scale"]
    depth_meters = (depth * depth_scale / 1000).astype(np.float32)
    depth_msg = bridge.cv2_to_imgmsg(depth_meters, encoding="32FC1")
    depth_msg.header.stamp = rgb_msg.header.stamp
    writer.write('/camera/depth/image_raw', serialize_message(depth_msg), t_ns)

    # Camera Info
    info = CameraInfo()
    info.header.stamp = rgb_msg.header.stamp
    info.k = cam_data["cam_K"]
    writer.write('/camera/camera_info', serialize_message(info), t_ns)

    gts_this_img = ground_truth.get(img_id_str, [])
    boxes_this_img = bboxes.get(img_id_str, [])

    # Find the entry for TARGET_OBJ_ID in this frame
    target_gt = None
    target_box = None
    target_inst_idx = None
    for inst_idx, (box_info, gt) in enumerate(zip(boxes_this_img, gts_this_img)):
        if gt["obj_id"] == TARGET_OBJ_ID:
            target_gt = gt
            target_box = box_info
            target_inst_idx = inst_idx
            break

    if target_gt is None:
        # object not visible in this frame, skip pose/mask/detection
        continue

    R = np.array(target_gt["cam_R_m2c"]).reshape(3, 3)
    t_vec = np.array(target_gt["cam_t_m2c"]) / 1000.0  # mm -> m

    # ── Ground truth object pose (object in camera frame) ───────────────────
    pose = PoseStamped()
    pose.header.stamp = rgb_msg.header.stamp
    pose.header.frame_id = "camera_link"
    pose.pose.position.x = float(t_vec[0])
    pose.pose.position.y = float(t_vec[1])
    pose.pose.position.z = float(t_vec[2])
    quat = Rotation.from_matrix(R).as_quat()
    pose.pose.orientation.x = float(quat[0])
    pose.pose.orientation.y = float(quat[1])
    pose.pose.orientation.z = float(quat[2])
    pose.pose.orientation.w = float(quat[3])
    writer.write('/gt_pose', serialize_message(pose), t_ns)

    # ── Camera pose in world frame (invert the object->camera transform) ────
    # Forward:  X_cam = R * X_world + t   (object in camera frame)
    # Inverse:  X_world = R^T * X_cam - R^T * t  (camera in world frame)
    R_inv = R.T
    t_inv = -R_inv @ t_vec
    cam_pose = PoseStamped()
    cam_pose.header.stamp = rgb_msg.header.stamp
    cam_pose.header.frame_id = "world"
    cam_pose.pose.position.x = float(t_inv[0])
    cam_pose.pose.position.y = float(t_inv[1])
    cam_pose.pose.position.z = float(t_inv[2])
    quat_inv = Rotation.from_matrix(R_inv).as_quat()
    cam_pose.pose.orientation.x = float(quat_inv[0])
    cam_pose.pose.orientation.y = float(quat_inv[1])
    cam_pose.pose.orientation.z = float(quat_inv[2])
    cam_pose.pose.orientation.w = float(quat_inv[3])
    writer.write('/camera_pose', serialize_message(cam_pose), t_ns)

    # ── Detection (single object only) ──────────────────────────────────────
    det_arr = Detection2DArray()
    det_arr.header.stamp = rgb_msg.header.stamp
    det_arr.header.frame_id = "camera_link"

    x, y, w, h = target_box["bbox_visib"]
    d2 = Detection2D()
    d2.header = det_arr.header
    d2.bbox.center.position.x = float(x + w / 2.0)
    d2.bbox.center.position.y = float(y + h / 2.0)
    d2.bbox.size_x = float(w)
    d2.bbox.size_y = float(h)
    hyp = ObjectHypothesisWithPose()
    hyp.hypothesis.class_id = str(TARGET_OBJ_ID)
    hyp.hypothesis.score = 1.0
    d2.results.append(hyp)
    det_arr.detections.append(d2)
    writer.write('/gt_detections', serialize_message(det_arr), t_ns)

    # ── Instance mask (visible portion only) ────────────────────────────────
    # mask_visib = only the visible pixels (occlusion-aware) — correct for ICP
    mask_file = f"{scene_path}/mask_visib/{img_id:06d}_{target_inst_idx:06d}.png"
    m = cv2.imread(mask_file, cv2.IMREAD_GRAYSCALE)
    if m is not None:
        instance_mask = np.zeros(m.shape, dtype=np.uint16)
        instance_mask[m > 0] = TARGET_OBJ_ID  # label pixels with the object id
        mask_msg = bridge.cv2_to_imgmsg(instance_mask, encoding="mono16")
        mask_msg.header.stamp = rgb_msg.header.stamp
        mask_msg.header.frame_id = "camera_link"
        writer.write('/gt_instance_mask', serialize_message(mask_msg), t_ns)

del writer
print(f"Bag saved to {output_path}")