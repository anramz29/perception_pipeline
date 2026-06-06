

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>


using sensor_msgs::msg::Image;
using vision_msgs::msg::Detection2DArray;
using geometry_msgs::msg::PoseStamped;
using sensor_msgs::msg::CameraInfo;

class icp_node : public rclcpp::Node {
public:
    icp_node() : Node("icp_node"){
        // get all our publishers
        // Todo figure out what we need to publish

        // get all the subscribers and sync
        pose_sub_.subscribe(this, "/estimated_pose");
        gt_mask_sub_.subscribe(this, "/gt_instance_mask");
        depth_sub_.subscribe(this, "/camera/depth/image_raw");
        info_sub_.subscribe(this, "/camera/camera_info");

        // Queue size 10. When a matching poses, masks. depth and camera infor
        // is found, it calls callback with BOTH at once.
        sync_ = std::make_shared<Sync>(SyncPolicy(10), pose_sub_, gt_mask_sub_, depth_sub_, info_sub_);
        sync_->registerCallback(
            std::bind(&icp_node::callback, this,
                      std::placeholders::_1, 
                      std::placeholders::_2, 
                      std::placeholders::_3,
                      std::placeholders::_4));

        // get the cad model
        declare_parameter("model_path", "");
        auto model_path = get_parameter("model_path").as_string();
        cad_model_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::io::loadPLYFile(model_path, *cad_model_);
    }
private:
    void callback(const PoseStamped::ConstSharedPtr& pose_msg,
                const Image::ConstSharedPtr& gt_mask_msg,
                const Image::ConstSharedPtr& depth_msg,
                const CameraInfo::ConstSharedPtr& camera_info_msg)
    {
        // Todo
    }

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        PoseStamped, Image, Image, CameraInfo>;
    using Sync = message_filters::Synchronizer<SyncPolicy>;

    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cad_model_;
    message_filters::Subscriber<PoseStamped> pose_sub_;
    message_filters::Subscriber<Image> gt_mask_sub_;
    message_filters::Subscriber<Image> depth_sub_;
    message_filters::Subscriber<CameraInfo> info_sub_;


};
