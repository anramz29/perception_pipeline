// this node takes the  detections from rosbag and layers them ontop of the raw_image topic

#include <iostream>
#include <string>
#include <memory>
 
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>


// message_filters: lets us subscribe to multiple topics and get a single
// callback only when messages with matching (approximately equal) timestamps
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using sensor_msgs::msg::Image;
using vision_msgs::msg::Detection2DArray;
using geometry_msgs::msg::PoseStamped;
using sensor_msgs::msg::CameraInfo;

class point_localization_node : public rclcpp::Node {
public:
    point_localization_node() : Node("point_localization_node"){
        // intialize debug publisher
        pose_pub_ = create_publisher<PoseStamped>("/estimated_pose", 10);

        // these are message::Filter subscribers meaning they feed into the 
        // syncronizer directly
        det_sub_.subscribe(this, "/gt_detections");
        depth_sub_.subscribe(this, "/camera/depth/image_raw");
        info_sub_.subscribe(this, "/camera/camera_info");

        // Queue size 10. When a matching RGB+det
        // is found, it calls callback with BOTH at once.
        sync_ = std::make_shared<Sync>(SyncPolicy(10), det_sub_, depth_sub_, info_sub_);
        sync_->registerCallback(
            std::bind(&point_localization_node::callback, this,
                      std::placeholders::_1, 
                      std::placeholders::_2, 
                      std::placeholders::_3));

        RCLCPP_INFO(get_logger(), "bbox_viz started, publishing /estimated_pose");

        
    }
private:
    void callback(const Detection2DArray::ConstSharedPtr& det_msg,
                    const Image::ConstSharedPtr& depth_msg,
                    const CameraInfo::ConstSharedPtr& info_msg)
        {
            // pull out the camera intrinsics
            double fx = info_msg->k[0];
            double cx = info_msg->k[2];
            double fy = info_msg->k[4];
            double cy = info_msg->k[5];
            
            // convert depth image to cv mat
            cv_bridge::CvImageConstPtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvShare(depth_msg, "32FC1");
            } catch (const cv_bridge::Exception& e) {
                RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what());
                return;
            }
            const cv::Mat& depth = cv_ptr->image;

            for (const auto& det : det_msg->detections){
                // get the center pixel
                int u = static_cast<int>(det.bbox.center.position.x);
                int v = static_cast<int>(det.bbox.center.position.y);

                // note: (row, col) = (v, u), y first!
                float Z = depth.at<float>(v, u);   

                // skip bad depth
                if (Z <= 0.0f || std::isnan(Z)) continue;  

                // actual back project algo
                double X = (u - cx) * Z / fx;
                double Y = (v - cy) * Z / fy;
                double Zp = Z;

                PoseStamped pose;
                pose.header.stamp = depth_msg->header.stamp;        // reuse stamp + frame_id (camera_link)
                pose.header.frame_id = "camera_link";
                pose.pose.position.x = X;
                pose.pose.position.y = Y;
                pose.pose.position.z = Zp;
                pose.pose.orientation.w = 1.0;          // identity rotation (no orientation yet)
            
                pose_pub_->publish(pose);
            }


        }
        // message_filters plumbing types. ApproximateTime over <Image, Detection2DArray>.
        using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        Detection2DArray, Image, CameraInfo>;
        using Sync = message_filters::Synchronizer<SyncPolicy>;
    
        message_filters::Subscriber<Detection2DArray> det_sub_;
        message_filters::Subscriber<Image> depth_sub_;
        message_filters::Subscriber<CameraInfo> info_sub_;
        std::shared_ptr<Sync> sync_;
        rclcpp::Publisher<PoseStamped>::SharedPtr pose_pub_;
};

int main(int argc, char** argv)
    {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<point_localization_node>());
    rclcpp::shutdown();
    }