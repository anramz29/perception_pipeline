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
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Geometry>
#include <optional>
#include <cmath>
#include <chrono>

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
        // we use auto here to avoid typing out the long pcl::PointCloud<pcl::PointXYZ>::Ptr
        
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

    // downsamples the point cloud for faster processing for course feature matching
    Cloud::Ptr downSample(const Cloud::Ptr& input){
        auto filtered = std::make_shared<Cloud>();
        pcl::VoxelGrid<pcl::PointXYZ> vg;

        // pass the input
        vg.setInputCloud(input);

        // takes three seprate float args
        vg.setLeafSize(0.01f, 0.01f, 0.01f);

        // make sure to derfrence
        vg.filter(*filtered);

        return filtered;
    }

    std::optional<Eigen::Matrix4f> multiStartIcp(
        const Cloud::Ptr& scene, // PTR is short hand for shared ptr
        const Cloud::Ptr& scene_ds,
        const Cloud::Ptr& cad_ds,
        const Eigen::Vector3f& translation)
    {
        //1. tracking variables

        // tracks the lowest fitness score seen so far
        float best_score = std::numeric_limits<float>::max();

        // stores the transform of best score
        Eigen::Matrix4f best_T = Eigen::Matrix4f::Identity();

        // guard against return best_T if zero starts converging
        bool any_converged = false;

        // 2. now we loop over 12 rotations that are evenly spaced around Z 
        // 30 deg steps
        for (int flip = 0; flip < 2; flip++){
            for (int i = 0; i < 12; i++){
                // divide a full circule into 12 equal steps 30 deg apart
                float angle = 2.0f * M_PI * i/12;

                // create a identity matrix to fill
                Eigen::Matrix4f candidate = Eigen::Matrix4f::Identity();

                // rotation around z
                Eigen::Matrix3f Rz = Eigen::AngleAxisf(angle,
                    Eigen::Vector3f::UnitZ()).toRotationMatrix();

                // add this so that we can flip the icp allowing stopping cosistent
                // 180 deg errors
                Eigen::Matrix3f Rx = Eigen::AngleAxisf(flip * M_PI,
                    Eigen::Vector3f::UnitX()).toRotationMatrix();

                // add the rotation and translation compenets to the candidate
                candidate.block<3,3>(0,0) = Rx * Rz;
                candidate.block<3,1>(0,3) = translation;

                // transform downsampled Cad to candidate pose
                auto cad_candidate = std::make_shared<Cloud>();
                pcl::transformPointCloud(*cad_ds, *cad_candidate, candidate);
                
                // fast ICP on downsampled clouds — just enough to score this rotation
                pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
                icp.setInputSource(cad_candidate);
                icp.setInputTarget(scene_ds);
                icp.setMaxCorrespondenceDistance(0.02);
                icp.setMaximumIterations(15);

                Cloud aligned;
                icp.align(aligned);

                if (!icp.hasConverged()) continue;

                float score = icp.getFitnessScore();
                if (score < best_score){
                    best_score = score;
                    best_T = icp.getFinalTransformation() * candidate;
                    any_converged = true;
                }   
            }
        }
        if (!any_converged) return std::nullopt;
        return runIcp(scene, best_T);
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
        icp.setMaximumIterations(30);

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

        // 1. build our scene cloud
        auto scene = buildSceneCloud(depth_ptr->image, mask_ptr->image, fx, fy, cx, cy);
        if (scene->empty()) return;

        // 2. downsample both cad and real point cloud
        auto scene_ds = downSample(scene);
        auto cad_ds   = downSample(cad_model_);

        // 3. create our translation vector
        Eigen::Vector3f translation(
            pose_msg->pose.position.x,
            pose_msg->pose.position.y,
            pose_msg->pose.position.z
        );

        // 4. multi-start ICP: try 24 rotations, pick best, then final full-res ICP
        auto t0 = std::chrono::steady_clock::now();
        auto result = multiStartIcp(scene, scene_ds, cad_ds, translation);
        auto t1 = std::chrono::steady_clock::now();
        double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
            "multiStartIcp took %.1f ms", ms);

        // 5. debug statement
        if (!result) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "multi-start ICP failed to converge");
            return;
        }


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
