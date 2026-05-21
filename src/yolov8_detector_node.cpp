#include "perception_pipeline/yolov8_detector.hpp"

#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

#include <algorithm>
#include <stdexcept>

namespace perception_pipeline
{

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
YOLOv8Detector::YOLOv8Detector(const rclcpp::NodeOptions & options)
: Node("yolov8_detector", options)
{
  // Declare / read parameters
  model_path_     = declare_parameter<std::string>("model_path", "");
  conf_threshold_ = declare_parameter<float>("confidence_threshold", 0.25f);
  nms_threshold_  = declare_parameter<float>("nms_threshold", 0.45f);

  if (model_path_.empty()) {
    throw std::runtime_error("Parameter 'model_path' must be set to a valid ONNX file path.");
  }

  // Load ONNX model via OpenCV DNN
  net_ = cv::dnn::readNetFromONNX(model_path_);
  net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
  net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
  RCLCPP_INFO(get_logger(), "Loaded YOLOv8 model: %s", model_path_.c_str());

  // Publishers
  det_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>("/detections", 10);
  debug_pub_ = create_publisher<sensor_msgs::msg::Image>("/detections/debug_image", 10);

  // Subscriber
  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "/camera/rgb/image_raw", 10,
    std::bind(&YOLOv8Detector::imageCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "YOLOv8Detector node started.");
}

// ---------------------------------------------------------------------------
// Image callback
// ---------------------------------------------------------------------------
void YOLOv8Detector::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  cv::Mat frame;
  try {
    frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  float scale;
  int pad_x, pad_y;
  cv::Mat blob_input = preprocess(frame, scale, pad_x, pad_y);

  net_.setInput(blob_input);

  // YOLOv8 ONNX output: [1, 4+num_classes, num_anchors]
  cv::Mat raw_output = net_.forward();  // shape (1, 84, 8400) for COCO

  auto detections = postprocess(raw_output, scale, pad_x, pad_y, frame.cols, frame.rows);

  publishDetections(detections, msg->header);
  publishDebugImage(frame, detections, msg->header);
}

// ---------------------------------------------------------------------------
// Preprocessing — letterbox resize to INPUT_W x INPUT_H
// Returns a 4-D blob ready for net_.setInput().
// scale, pad_x, pad_y are set so detections can be mapped back to original coords.
// ---------------------------------------------------------------------------
cv::Mat YOLOv8Detector::preprocess(
  const cv::Mat & frame,
  float & scale,
  int & pad_x, int & pad_y)
{
  int orig_w = frame.cols;
  int orig_h = frame.rows;

  // Uniform scale that fits the image into INPUT_W x INPUT_H
  scale = std::min(
    static_cast<float>(INPUT_W) / orig_w,
    static_cast<float>(INPUT_H) / orig_h);

  int new_w = static_cast<int>(orig_w * scale);
  int new_h = static_cast<int>(orig_h * scale);

  pad_x = (INPUT_W - new_w) / 2;
  pad_y = (INPUT_H - new_h) / 2;

  cv::Mat resized;
  cv::resize(frame, resized, cv::Size(new_w, new_h));

  // Place on grey canvas
  cv::Mat canvas(INPUT_H, INPUT_W, CV_8UC3, cv::Scalar(114, 114, 114));
  resized.copyTo(canvas(cv::Rect(pad_x, pad_y, new_w, new_h)));

  // BGR → RGB, [0,255] → [0,1], shape (1,3,H,W)
  cv::Mat blob;
  cv::dnn::blobFromImage(canvas, blob, 1.0 / 255.0,
    cv::Size(INPUT_W, INPUT_H),
    cv::Scalar(0, 0, 0),
    /*swapRB=*/true, /*crop=*/false, CV_32F);

  return blob;
}

// ---------------------------------------------------------------------------
// Post-processing
// YOLOv8 ONNX layout: output shape is (1, 4+nc, num_anchors)
//   rows 0-3  : cx, cy, w, h  (in 640×640 space)
//   rows 4..  : per-class scores (already sigmoid-activated)
// ---------------------------------------------------------------------------
std::vector<Detection> YOLOv8Detector::postprocess(
  const cv::Mat & output,
  float scale,
  int pad_x, int pad_y,
  int orig_w, int orig_h)
{
  // output dims: [1, rows, cols] where rows = 4+nc, cols = num_anchors
  // Reshape to (rows, num_anchors) for easier indexing
  const int rows = output.size[1];   // 4 + num_classes
  const int num_anchors = output.size[2];
  const int num_classes = rows - 4;

  // Wrap raw data (no copy)
  cv::Mat data(rows, num_anchors, CV_32F, const_cast<float *>(
    reinterpret_cast<const float *>(output.data)));

  std::vector<cv::Rect> boxes;
  std::vector<float>    scores;
  std::vector<int>      class_ids;

  for (int a = 0; a < num_anchors; ++a) {
    // Find the class with highest score among all classes for this anchor
    float best_score = -1.f;
    int   best_cls   = -1;
    for (int c = 0; c < num_classes; ++c) {
      float s = data.at<float>(4 + c, a);
      if (s > best_score) {
        best_score = s;
        best_cls   = c;
      }
    }

    if (best_score < conf_threshold_) {
      continue;
    }

    // cx, cy, w, h in 640×640 input space
    float cx = data.at<float>(0, a);
    float cy = data.at<float>(1, a);
    float w  = data.at<float>(2, a);
    float h  = data.at<float>(3, a);

    // Map back to original image coordinates
    float x0 = (cx - w / 2.f - pad_x) / scale;
    float y0 = (cy - h / 2.f - pad_y) / scale;
    float x1 = (cx + w / 2.f - pad_x) / scale;
    float y1 = (cy + h / 2.f - pad_y) / scale;

    // Clamp to image bounds
    x0 = std::clamp(x0, 0.f, static_cast<float>(orig_w - 1));
    y0 = std::clamp(y0, 0.f, static_cast<float>(orig_h - 1));
    x1 = std::clamp(x1, 0.f, static_cast<float>(orig_w - 1));
    y1 = std::clamp(y1, 0.f, static_cast<float>(orig_h - 1));

    int bx = static_cast<int>(x0);
    int by = static_cast<int>(y0);
    int bw = static_cast<int>(x1 - x0);
    int bh = static_cast<int>(y1 - y0);

    if (bw <= 0 || bh <= 0) {
      continue;
    }

    boxes.push_back(cv::Rect(bx, by, bw, bh));
    scores.push_back(best_score);
    class_ids.push_back(best_cls);
  }

  // NMS
  std::vector<int> nms_indices;
  cv::dnn::NMSBoxes(boxes, scores, conf_threshold_, nms_threshold_, nms_indices);

  std::vector<Detection> results;
  results.reserve(nms_indices.size());
  for (int idx : nms_indices) {
    results.push_back({class_ids[idx], scores[idx], boxes[idx]});
  }
  return results;
}

// ---------------------------------------------------------------------------
// Publish Detection2DArray
// ---------------------------------------------------------------------------
void YOLOv8Detector::publishDetections(
  const std::vector<Detection> & dets,
  const std_msgs::msg::Header & header)
{
  vision_msgs::msg::Detection2DArray msg;
  msg.header = header;

  for (const auto & d : dets) {
    vision_msgs::msg::Detection2D det;
    det.header = header;

    // Bounding box center + size
    det.bbox.center.position.x = d.bbox.x + d.bbox.width  / 2.0;
    det.bbox.center.position.y = d.bbox.y + d.bbox.height / 2.0;
    det.bbox.size_x = static_cast<double>(d.bbox.width);
    det.bbox.size_y = static_cast<double>(d.bbox.height);

    // Hypothesis
    vision_msgs::msg::ObjectHypothesisWithPose hyp;
    hyp.hypothesis.class_id = std::to_string(d.class_id);
    hyp.hypothesis.score    = static_cast<double>(d.confidence);
    det.results.push_back(hyp);

    msg.detections.push_back(det);
  }

  det_pub_->publish(msg);
}

// ---------------------------------------------------------------------------
// Publish debug image with drawn bounding boxes
// ---------------------------------------------------------------------------
void YOLOv8Detector::publishDebugImage(
  const cv::Mat & frame,
  const std::vector<Detection> & dets,
  const std_msgs::msg::Header & header)
{
  if (debug_pub_->get_subscription_count() == 0) {
    return;  // skip expensive drawing if nobody is listening
  }

  cv::Mat vis = frame.clone();

  for (const auto & d : dets) {
    cv::rectangle(vis, d.bbox, cv::Scalar(0, 255, 0), 2);

    std::string label = "cls:" + std::to_string(d.class_id) +
      " " + std::to_string(static_cast<int>(d.confidence * 100)) + "%";

    int baseline = 0;
    cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
    int text_y = std::max(d.bbox.y - 5, text_size.height + 5);

    cv::rectangle(
      vis,
      cv::Point(d.bbox.x, text_y - text_size.height - 5),
      cv::Point(d.bbox.x + text_size.width, text_y + baseline - 5),
      cv::Scalar(0, 255, 0), cv::FILLED);

    cv::putText(
      vis, label,
      cv::Point(d.bbox.x, text_y - 5),
      cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
  }

  auto out = cv_bridge::CvImage(header, "bgr8", vis).toImageMsg();
  debug_pub_->publish(*out);
}

}  // namespace perception_pipeline

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perception_pipeline::YOLOv8Detector>());
  rclcpp::shutdown();
  return 0;
}
