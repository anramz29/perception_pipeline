#!/usr/bin/env python3
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO


# YOLOv11 Object Detection Node
class YOLOv11DetectorNode(Node):
    def __init__(self):
        super().__init__('yolov11_detector')

        # Path to the YOLOv11 model weights (e.g., 'yolov11n.pt' or a custom path)
        self.declare_parameter('model_path', '')

        # Confidence threshold for detections (0.0 to 1.0)
        self.declare_parameter('confidence_threshold', 0.25)


        # Get parameters
        model_path = self.get_parameter('model_path').get_parameter_value().string_value 
        self.conf_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value

        # need to always validate
        if not model_path:
            raise RuntimeError("Parameter 'model_path' must be set to a valid model path.")

        # Loead Yolov11 
        self.model = YOLO(model_path)
        self.bridge = CvBridge()

        # Publishers for detections and debug images, message buffer size of 10
        self.det_pub = self.create_publisher(Detection2DArray, '/detections', 10)
        self.debug_pub = self.create_publisher(Image, '/detections/debug_image', 10)

        # Subscribe to the raw RGB image topic
        self.create_subscription(Image, '/camera/rgb/image_raw', self._image_cb, 10)

        # Log startup info, including the model path
        self.get_logger().info(f'YOLOv11Detector node started, model: {model_path}')

    def _image_cb(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        results = self.model(frame, conf=self.conf_threshold, verbose=False)

        det_array = Detection2DArray()
        det_array.header = msg.header

        boxes = results[0].boxes if results and results[0].boxes is not None else []
        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()

            det = Detection2D()
            det.header = msg.header
            det.bbox.center.position.x = (x1 + x2) / 2.0
            det.bbox.center.position.y = (y1 + y2) / 2.0
            det.bbox.size_x = float(x2 - x1)
            det.bbox.size_y = float(y2 - y1)

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(int(box.cls[0]))
            hyp.hypothesis.score = float(box.conf[0])
            det.results.append(hyp)

            det_array.detections.append(det)

        self.det_pub.publish(det_array)
        self._publish_debug(frame, boxes, msg.header)

    def _publish_debug(self, frame, boxes, header) -> None:
        if self.debug_pub.get_subscription_count() == 0:
            return

        vis = frame.copy()
        for box in boxes:
            x1, y1, x2, y2 = (int(v) for v in box.xyxy[0].tolist())
            conf = float(box.conf[0])
            cls_id = int(box.cls[0])

            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f'cls:{cls_id} {int(conf * 100)}%'
            (tw, th), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            text_y = max(y1 - 5, th + 5)
            cv2.rectangle(vis, (x1, text_y - th - 5), (x1 + tw, text_y + baseline - 5),
                          (0, 255, 0), cv2.FILLED)
            cv2.putText(vis, label, (x1, text_y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        debug_msg = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
        debug_msg.header = header
        self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YOLOv11DetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
