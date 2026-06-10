#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>

#include <Eigen/Geometry>
#include <optional>
#include <cmath>

using sensor_msgs::msg::Image;
using sensor_msgs::msg::CameraInfo;
using geometry_msgs::msg::PoseStamped;
using Cloud = pcl::PointCloud<pcl::PointXYZ>;

class IcpNode : public rclcpp::Node {
public:
    IcpNode() : Node("icp_node") {
        // get all our publishers
        ref_pose_pub_ = create_publisher<PoseStamped>("/refined_pose", 10);

        // get all the subscribers and sync
        pose_sub_.subscribe(this, "/estimated_pose");
        gt_mask_sub_.subscribe(this, "/gt_instance_mask");
        depth_sub_.subscribe(this, "/camera/depth/image_raw");
        info_sub_.subscribe(this, "/camera/camera_info");
        gt_pose_sub_.subscribe(this, "/gt_pose");

        // Queue size 10. When a matching pose, mask, depth, camera info,
        // and gt pose is found, it calls callback with ALL at once.
        sync_ = std::make_shared<Sync>(SyncPolicy(10),
            pose_sub_, gt_mask_sub_, depth_sub_, info_sub_, gt_pose_sub_);
        sync_->registerCallback(std::bind(&IcpNode::callback, this,
            std::placeholders::_1, std::placeholders::_2,
            std::placeholders::_3, std::placeholders::_4,
            std::placeholders::_5));

        // get the cad model
        declare_parameter("model_path", "");
        cad_model_ = std::make_shared<Cloud>();
        pcl::io::loadPLYFile(get_parameter("model_path").as_string(), *cad_model_);

        // T-LESS PLY models are in millimetres; scene cloud is in metres
        for (auto& pt : *cad_model_) { pt.x *= 0.001f; pt.y *= 0.001f; pt.z *= 0.001f; }
    }

private:
    // ── Type aliases ──────────────────────────────────────────────────────────
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        PoseStamped, Image, Image, CameraInfo, PoseStamped>;
    using Sync = message_filters::Synchronizer<SyncPolicy>;

    // ── Helpers ───────────────────────────────────────────────────────────────

    // Convert a ROS image into a cv::Mat with the given encoding.
    // Returns false on failure (and logs the error).
    bool toCvMat(const Image::ConstSharedPtr& msg,
                 const std::string& encoding,
                 const char* label,
                 cv_bridge::CvImageConstPtr& out)
    {
        try {
            out = cv_bridge::toCvShare(msg, encoding);
            return true;
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge %s: %s", label, e.what());
            return false;
        }
    }

    // Iterate over the rows and columns of the depth image.
    // Skip background pixels in mask, skip non-valid / no-return depth,
    // then back-project valid pixels to 3D using camera intrinsics.
    Cloud::Ptr buildSceneCloud(const cv::Mat& depth, const cv::Mat& mask,
                               double fx, double fy, double cx, double cy)
    {
        auto cloud = std::make_shared<Cloud>();
        for (int v = 0; v < depth.rows; ++v) {
            for (int u = 0; u < depth.cols; ++u) {
                if (mask.at<uint16_t>(v, u) == 0) continue;

                // read depth at this pixel (32FC1 -> float)
                float z = depth.at<float>(v, u);
                if (z <= 0.0f || std::isnan(z)) continue;

                // append our valid x, y, and z
                cloud->push_back({
                    static_cast<float>((u - cx) * z / fx),
                    static_cast<float>((v - cy) * z / fy),
                    z
                });
            }
        }
        return cloud;
    }

    // Move CAD model to the initial guess, run ICP against the scene cloud.
    // Returns the final absolute transform (ICP delta * initial guess),
    // or nullopt if ICP does not converge.
    std::optional<Eigen::Matrix4f> runIcp(const Cloud::Ptr& scene,
                                          const Eigen::Matrix4f& init_T)
    {
        // move CAD model to the initial guess
        auto init_cad = std::make_shared<Cloud>();
        pcl::transformPointCloud(*cad_model_, *init_cad, init_T);

        // run ICP
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(init_cad);
        icp.setInputTarget(scene);
        icp.setMaxCorrespondenceDistance(0.05);
        icp.setMaximumIterations(50);

        // run alignment
        Cloud aligned;
        icp.align(aligned);

        // check for convergence
        if (!icp.hasConverged()) return std::nullopt;

        // build final transformation from ICP delta * initial guess
        return icp.getFinalTransformation() * init_T;
    }

    // Extract translation and rotation from the transform, pack into PoseStamped.
    PoseStamped toPoseStamped(const Eigen::Matrix4f& T,
                              const std_msgs::msg::Header& header)
    {
        Eigen::Vector3f    t(T.block<3,1>(0,3));
        Eigen::Quaternionf q(T.block<3,3>(0,0));

        PoseStamped out;
        out.header             = header;
        out.pose.position.x    = t.x();
        out.pose.position.y    = t.y();
        out.pose.position.z    = t.z();
        out.pose.orientation.x = q.x();
        out.pose.orientation.y = q.y();
        out.pose.orientation.z = q.z();
        out.pose.orientation.w = q.w();
        return out;
    }

    // Compare estimated pose against ground truth.
    // Logs Euclidean position error (metres) and rotation error (degrees).
    void logErrors(const Eigen::Matrix4f& T, const PoseStamped& gt)
    {
        Eigen::Vector3f    t(T.block<3,1>(0,3));
        Eigen::Quaternionf q(T.block<3,3>(0,0));

        double dx = t.x() - gt.pose.position.x;
        double dy = t.y() - gt.pose.position.y;
        double dz = t.z() - gt.pose.position.z;
        double pos_err = std::sqrt(dx*dx + dy*dy + dz*dz);

        Eigen::Quaternionf q_gt(
            gt.pose.orientation.w,
            gt.pose.orientation.x,
            gt.pose.orientation.y,
            gt.pose.orientation.z);
        double dot         = std::abs(q.dot(q_gt));
        double rot_err_deg = 2.0 * std::acos(std::min(dot, 1.0)) * 180.0 / M_PI;

        RCLCPP_INFO(get_logger(), "pos_err=%.4fm  rot_err=%.2fdeg", pos_err, rot_err_deg);
    }

    // ── Main callback ─────────────────────────────────────────────────────────

    void callback(const PoseStamped::ConstSharedPtr& pose_msg,
                  const Image::ConstSharedPtr&        gt_mask_msg,
                  const Image::ConstSharedPtr&        depth_msg,
                  const CameraInfo::ConstSharedPtr&   info_msg,
                  const PoseStamped::ConstSharedPtr&  gt_pose_msg)
    {
        cv_bridge::CvImageConstPtr depth_ptr, mask_ptr;
        if (!toCvMat(depth_msg,   "32FC1",  "depth", depth_ptr)) return;
        if (!toCvMat(gt_mask_msg, "mono16", "mask",  mask_ptr))  return;

        // pull out the camera intrinsics
        double fx = info_msg->k[0], cx = info_msg->k[2];
        double fy = info_msg->k[4], cy = info_msg->k[5];

        auto scene = buildSceneCloud(depth_ptr->image, mask_ptr->image, fx, fy, cx, cy);
        if (scene->empty()) return;

        // build initial transform from point_localization_node's position guess
        Eigen::Matrix4f init_T = Eigen::Matrix4f::Identity();
        init_T(0,3) = pose_msg->pose.position.x;
        init_T(1,3) = pose_msg->pose.position.y;
        init_T(2,3) = pose_msg->pose.position.z;

        auto result = runIcp(scene, init_T);
        if (!result) return;

        auto refined = toPoseStamped(*result, depth_msg->header);
        refined.header.frame_id = "camera_link";
        ref_pose_pub_->publish(refined);
        logErrors(*result, *gt_pose_msg);
    }

    // ── Members ───────────────────────────────────────────────────────────────

    rclcpp::Publisher<PoseStamped>::SharedPtr ref_pose_pub_;
    std::shared_ptr<Sync>  sync_;
    std::shared_ptr<Cloud> cad_model_;

    message_filters::Subscriber<PoseStamped> pose_sub_;
    message_filters::Subscriber<Image>       gt_mask_sub_;
    message_filters::Subscriber<Image>       depth_sub_;
    message_filters::Subscriber<CameraInfo>  info_sub_;
    message_filters::Subscriber<PoseStamped> gt_pose_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IcpNode>());
    rclcpp::shutdown();
}
