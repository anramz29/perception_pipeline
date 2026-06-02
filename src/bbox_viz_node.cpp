// this node takes the  detections from rosbag and layers them ontop of the raw_image topic

#include <iostream>
#include <string>
#include <memory>
 
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// message_filters: lets us subscribe to multiple topics and get a single
// callback only when messages with matching (approximately equal) timestamps
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using sensor_msgs::msg::Image;
using vision_msgs::msg::Detection2DArray;

class BBoxVizNode : public rclcpp::Node {
public:
    BBoxVizNode() : Node("bbox_viz"){
        // intialize debug publisher
        debug_pub_ = create_publisher<Image>("/gt_detections_debug", 10);

        // these are message::Filter subscribers meaning they feed into the 
        // syncronizer directly
        rgb_sub_.subscribe(this, "/camera/rgb/image_raw");
        det_sub_.subscribe(this, "/gt_detections");

        // Queue size 10. When a matching RGB+det
        // is found, it calls callback with BOTH at once.
        sync_ = std::make_shared<Sync>(SyncPolicy(10), rgb_sub_, det_sub_);
        sync_->registerCallback(
            std::bind(&BBoxVizNode::callback, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(get_logger(), "bbox_viz started, publishing /gt_detections_debug");
    }
private:
    void callback(const Image::ConstSharedPtr& rgb_msg,
                const Detection2DArray::ConstSharedPtr& det_msg){

            // now try and except block to turn img msg to cv format
            cv_bridge::CvImageConstPtr cv_ptr;
            try {
                // share image data w/out copying
                cv_ptr = cv_bridge::toCvShare(rgb_msg, "bgr8");
            } catch (const cv_bridge::Exception& e) {
                RCLCPP_INFO(get_logger(), "cv_bridge: %s", e.what());
                return;
            }

            // clone the image as we will be drawing ontop of it 
            cv::Mat vis = cv_ptr->image.clone();

            // now draw each detection, remeber that 2D detection array gives us the center points
            // and the scale not custom message like ive done before
            for (const auto& det: det_msg->detections){
                double cx = det.bbox.center.position.x;
                double cy = det.bbox.center.position.y;
                double w  = det.bbox.size_x;
                double h  = det.bbox.size_y;

                // need now to obtain or x1, y1 ... and revert to an integer
                int x1 = static_cast<int>(cx - w/ 2.0);
                int y1 = static_cast<int>(cy - h / 2.0);
                int x2 = static_cast<int>(cx + w / 2.0);
                int y2 = static_cast<int>(cy + h / 2.0);

                // create the visualization rectangle
                cv::rectangle(vis, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);

                // Pull the class id (the T-LESS object id, e.g. "29") if present.
                std::string label = "obj";
                if (!det.results.empty())
                label = "obj " + det.results[0].hypothesis.class_id;

                // Draw the label just above the top-left corner.
                int baseline = 0;
                cv::Size ts = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX,
                                            0.5, 1, &baseline);
                int ty = std::max(y1 - 5, ts.height + 5);
                cv::rectangle(vis,
                            cv::Point(x1, ty - ts.height - 5),
                            cv::Point(x1 + ts.width, ty + baseline - 5),
                            cv::Scalar(0, 255, 0), cv::FILLED);
                cv::putText(vis, label, cv::Point(x1, ty - 5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        cv::Scalar(0, 0, 0), 1);
                }
            // republish the annontated image
            auto out = cv_bridge::CvImage(rgb_msg->header, "bgr8", vis).toImageMsg();
            // point and derefernce
            debug_pub_->publish(*out);
        }
    // message_filters plumbing types. ApproximateTime over <Image, Detection2DArray>.
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    Image, Detection2DArray>;
    using Sync = message_filters::Synchronizer<SyncPolicy>;

    message_filters::Subscriber<Image> rgb_sub_;
    message_filters::Subscriber<Detection2DArray> det_sub_;
    std::shared_ptr<Sync> sync_;
    rclcpp::Publisher<Image>::SharedPtr debug_pub_;
};

int main(int argc, char** argv)
    {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BBoxVizNode>());
    rclcpp::shutdown();
    }