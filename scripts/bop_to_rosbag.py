import rclpy # ros depedencies 
from rclpy.serialization import serialize_message #ros messages conversion
import rosbag2_py # for writing ROS 2 bags
from sensor_msgs.msg import Image, CameraInfo # image camera info
from geometry_msgs.msg import PoseStamped # for ground truth poses
from cv_bridge import CvBridge # for converting OpenCV images to ROS messages
from scipy.spatial.transform import Rotation # for converting rotation matrices to quaternions
import cv2, json, numpy as np, os # for file handling and data processing
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

# improting cv bridge
bridge = CvBridge() 

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

# Load ground truth poses (if available)
with open(f"{scene_path}/scene_gt.json") as f:
    ground_truth = json.load(f)

# setup writer
writer = rosbag2_py.SequentialWriter()

# configure storage options (using SQLite3 backend)
storage_options = rosbag2_py.StorageOptions(uri=output_path, storage_id='sqlite3')


# configure converter options (using default, no compression)
converter_options = rosbag2_py.ConverterOptions('', '')

# open the bag for writing
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


# register a topic per object (up to 30 in T-LESS)
for obj_id in range(1, 31):
    writer.create_topic(rosbag2_py.TopicMetadata(
        name=f'/gt_pose/obj_{obj_id:06d}',
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
    info.k = cam_data["cam_K"]  # lowercase k in ROS 2
    writer.write('/camera/camera_info', serialize_message(info), t_ns)

    gts_this_img = ground_truth.get(img_id_str, [])

    # Instance segmentation mask (one label image, unique value per instance)
    # mask_visib files are named {img_id:06d}_{instance_idx:06d}.png, ordered like scene_gt
    instance_mask = None
    for inst_idx in range(len(gts_this_img)):
        mask_file = f"{scene_path}/mask_visib/{img_id:06d}_{inst_idx:06d}.png"
        m = cv2.imread(mask_file, cv2.IMREAD_GRAYSCALE)
        if m is None:
            continue
        if instance_mask is None:
            instance_mask = np.zeros(m.shape, dtype=np.uint16)
        # label = instance index + 1 (0 = background)
        instance_mask[m > 0] = inst_idx + 1

    if instance_mask is not None:
        mask_msg = bridge.cv2_to_imgmsg(instance_mask, encoding="mono16")
        mask_msg.header.stamp = rgb_msg.header.stamp
        mask_msg.header.frame_id = "camera_link"
        writer.write('/gt_instance_mask', serialize_message(mask_msg), t_ns)

    # Ground Truth Poses
    for gt in gts_this_img:
        R = np.array(gt["cam_R_m2c"]).reshape(3, 3)
        t_vec = np.array(gt["cam_t_m2c"]) / 1000.0

        pose = PoseStamped()
        pose.header.stamp = rgb_msg.header.stamp
        pose.header.frame_id = "camera_link"
        pose.pose.position.x = float(t_vec[0]) # converting to float for ROS message compatibility
        pose.pose.position.y = float(t_vec[1])
        pose.pose.position.z = float(t_vec[2])

        quat = Rotation.from_matrix(R).as_quat()
        pose.pose.orientation.x = float(quat[0])
        pose.pose.orientation.y = float(quat[1])
        pose.pose.orientation.z = float(quat[2])
        pose.pose.orientation.w = float(quat[3])

        writer.write(f"/gt_pose/obj_{gt['obj_id']:06d}", serialize_message(pose), t_ns)

    # Bboxes
    det_arr = Detection2DArray()
    det_arr.header.stamp = rgb_msg.header.stamp
    det_arr.header.frame_id = "camera_link"

    boxes_this_img = bboxes.get(img_id_str, [])

    # zip pairs box[i] with pose[i] — same object, same order
    for box_info, gt in zip(boxes_this_img, gts_this_img):
        x, y, w, h = box_info["bbox_visib"]   # [x, y, width, height]

        d2 = Detection2D()
        d2.header = det_arr.header
        d2.bbox.center.position.x = float(x + w / 2.0)   # center, not corner
        d2.bbox.center.position.y = float(y + h / 2.0)
        d2.bbox.size_x = float(w)
        d2.bbox.size_y = float(h)

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = str(gt["obj_id"])  # real T-LESS object id
        hyp.hypothesis.score = 1.0                    # ground truth = full confidence
        d2.results.append(hyp)

        det_arr.detections.append(d2)

    writer.write('/gt_detections', serialize_message(det_arr), t_ns)

del writer
print(f"Bag saved to {output_path}")