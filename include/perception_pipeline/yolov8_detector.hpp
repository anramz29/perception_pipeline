#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>

#include <string>
#include <vector>

namespace perception_pipeline
{

struct Detection
{
  int class_id;
  float confidence;
  cv::Rect bbox;  // in original image coordinates
};

class YOLOv8Detector : public rclcpp::Node
{
public:
  explicit YOLOv8Detector(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

  // Pre/post-processing helpers
  cv::Mat preprocess(const cv::Mat & frame, float & scale, int & pad_x, int & pad_y);
  std::vector<Detection> postprocess(
    const cv::Mat & output,
    float scale,
    int pad_x, int pad_y,
    int orig_w, int orig_h);

  // Publishing helpers
  void publishDetections(
    const std::vector<Detection> & dets,
    const std_msgs::msg::Header & header);
  void publishDebugImage(
    const cv::Mat & frame,
    const std::vector<Detection> & dets,
    const std_msgs::msg::Header & header);

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr det_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;

  // Parameters
  std::string model_path_;
  float conf_threshold_;
  float nms_threshold_;

  // Network
  cv::dnn::Net net_;
  static constexpr int INPUT_W = 640;
  static constexpr int INPUT_H = 640;
};

}  // namespace perception_pipeline
