// C++ YOLO detector node using ONNX Runtime.
//
// Replaces detector_node.py. Key differences from the Python version:
//   - Inference runs via ONNX Runtime C++ API instead of ultralytics (Python).
//     ultralytics wraps PyTorch which has significant Python/GIL overhead per call.
//     ONNX Runtime runs the exported graph directly in C++ with no interpreter overhead.
//   - Preprocessing (letterbox, BGR->RGB, HWC->CHW, normalize) is done with raw
//     OpenCV C++ calls and a memcpy rather than numpy array operations.
//   - NMS is implemented here explicitly. ultralytics does NMS internally in Python;
//     the ONNX export does NOT include NMS, so we must do it ourselves.
//   - Memory management is explicit (RAII via std::unique_ptr, Ort::Value on stack).
//     Python relies on the GC; here allocations are predictable and deterministic.
//   - cv_bridge::toCvShare gives a zero-copy view of the ROS image buffer (no memcpy
//     of the raw frame). The Python version uses imgmsg_to_cv2 which always copies.

// yolov11 

// this is in progress at the moment, but the general structure is there. 

#include <algorithm>
#include <cstring>
#include <string>
#include <type_traits>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>

// YOLO11/YOLOv8 expects a fixed 640x640 input regardless of source resolution.
static constexpr int NET_W = 640;
static constexpr int NET_H = 640;


// this is the plain strucutre used for decoding dections in original corrds
struct Det { float x1, y1, x2, y2, score; int cls; };


static cv::Mat letterbox(const cv::Mat& img, float& scale, int& pad_x, int& pad_y)
{
    // obtain the scaling factor information to resize
    float r = std::min(float(NET_W) / img.cols, float(NET_H) / img.rows);

    // Compute the new width and height after scaling.
    int nw = std::lround(img.cols * r);
    int nh = std::lround(img.rows * r);

    // How many pixels of gray padding on each side.
    pad_x = (NET_W - nw) / 2; 
    pad_y = (NET_H - nh) / 2; 
    scale = r;

    // Letterbox by resizing the image and copying it into a gray canvas.
    cv::Mat out(NET_H, NET_W, CV_8UC3, cv::Scalar(114, 114, 114)); //grey canvas as yolo was trained on it
    cv::Mat resized; // paste initalize out rise params
    cv::resize(img, resized, cv::Size(nw, nh), 0, 0, cv::INTER_LINEAR);
    resized.copyTo(out(cv::Rect(pad_x, pad_y, nw, nh))); //paste image in the center
    return out;
}

// det structure
// Intersection-over-Union between two detections.
// Used by NMS to decide whether two boxes overlap enough to be duplicates.
static float iou(const Det& a, const Det& b)
{
    float ix1 = std::max(a.x1, b.x1), iy1 = std::max(a.y1, b.y1);
    float ix2 = std::min(a.x2, b.x2), iy2 = std::min(a.y2, b.y2);
    // intersection area
    float inter = std::max(0.f, ix2 - ix1) * std::max(0.f, iy2 - iy1);
    // compute the union is area a + b - union
    return inter / ((a.x2-a.x1)*(a.y2-a.y1) + (b.x2-b.x1)*(b.y2-b.y1) - inter + 1e-6f);
}

// Greedy Non-Maximum Suppression.
//
// YOLO produces thousands of overlapping candidate boxes (8400 for 640x640 input).
// NMS keeps only the highest-scoring box in each cluster of overlapping boxes.
//
// Why is this here and not in the ONNX model?
// ultralytics' Python export with format='onnx' omits NMS by default so the graph
// stays portable. The Python ultralytics wrapper ran NMS on the raw tensor output
// automatically. Since we're calling ONNX Runtime directly, we do it ourselves.
//
// Takes dets by value intentionally — we sort in-place.
static std::vector<Det> nms(std::vector<Det> dets, float thresh)
{
    // Sort highest confidence first so we always keep the best box.
    std::sort(dets.begin(), dets.end(), [](const Det& a, const Det& b){
        return a.score > b.score;
    });

    // define our dead array
    std::vector<bool> dead(dets.size(), false);

    // our out "passing detections"
    std::vector<Det> out;
    for (size_t i = 0; i < dets.size(); ++i) {
        // if survived add to the out
        if (dead[i]) continue;
        out.push_back(dets[i]);
        for (size_t j = i + 1; j < dets.size(); ++j)
            // important to define the threshold, iou is intersect over
            // overlap, so it's basically filtering about to get even better 
            // results
            if (!dead[j] && iou(dets[i], dets[j]) > thresh) dead[j] = true;
    }
    return out;
}


class DetectorNode : public rclcpp::Node
{
public:
    // Constructor
    DetectorNode() : Node("detector"),
                     env_(ORT_LOGGING_LEVEL_WARNING, "detector")
    {
        
        declare_parameter("model_path", "");
        declare_parameter("confidence_threshold", 0.25);
        declare_parameter("nms_threshold", 0.45);  // no equivalent in Python

        // load our model and params
        auto model_path = get_parameter("model_path").as_string();
        conf_thresh_ = static_cast<float>(get_parameter("confidence_threshold").as_double());
        nms_thresh_  = static_cast<float>(get_parameter("nms_threshold").as_double());

        // logic for setting the model path
        if (model_path.empty())
            throw std::runtime_error("model_path must be set");

        // ONNX Runtime session — equivalent to YOLO(model_path) in Python.
        Ort::SessionOptions opts;
        opts.SetIntraOpNumThreads(1); // YOLOv8 is single-threaded internally belive it or not
        opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED); // optimize for speed
        session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), opts); // intialize the session

        // ONNX models address inputs/outputs by string name, not index.
        // We read the names once at startup and reuse them every callback.
        // AllocatedStringPtr is freed when it goes out of scope, so we copy to std::string.
        Ort::AllocatorWithDefaultOptions alloc;
        input_name_  = std::string(session_->GetInputNameAllocated(0, alloc).get());
        output_name_ = std::string(session_->GetOutputNameAllocated(0, alloc).get());

        // Same topic names and QoS depth as the Python node.
        det_pub_   = create_publisher<vision_msgs::msg::Detection2DArray>("/detections", 10);
        debug_pub_ = create_publisher<sensor_msgs::msg::Image>("/detections/debug_image", 10);
        sub_       = create_subscription<sensor_msgs::msg::Image>(
            "/camera/rgb/image_raw", 10,
            std::bind(&DetectorNode::image_cb, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Detector started (ONNX Runtime): %s", model_path.c_str());
    }

// used internally
private:

    // Callback
    void image_cb(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // ── Convert ROS image to OpenCV Mat ───────────────────────────────────
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what());
            return;
        }
        // this allows us a non copy buffer this is why cpp is so much faster than python
        const cv::Mat& frame = cv_ptr->image;

        // ── Preprocessing ────────────────────────────────────────────────────
        // ultralytics did all of this internally in Python. Because we're calling
        // ONNX Runtime directly, we must replicate it ourselves.

        // passed by refrence
        float scale; int pad_x, pad_y;

        // create our letter box
        cv::Mat lb = letterbox(frame, scale, pad_x, pad_y);

        // YOLO was trained on RGB; OpenCV stores images as BGR by default.
        cv::Mat rgb; cv::cvtColor(lb, rgb, cv::COLOR_BGR2RGB);

        // Normalize pixels from [0,255] uint8 to [0.0,1.0] float32.
        // convertTo with alpha=1/255 does this in one pass over the data.
        rgb.convertTo(rgb, CV_32F, 1.0 / 255.0);

        // ONNX Runtime expects a flat float array in CHW order (channels first).
        // OpenCV stores images in HWC order (channels last, interleaved).
        // cv::split separates the three channels into individual planes, then
        // memcpy lays them out sequentially: RRRR...GGGG...BBBB...
        // In Python: numpy did this with img.transpose(2,0,1) then np.ascontiguousarray.
        // We want to splut this due to NN processing the conv letter channel goes against all the layers
        // we are given 3 640 * 640 blob matrix -> tensor
        std::vector<float> blob(3 * NET_H * NET_W);
        std::vector<cv::Mat> ch(3);

        // takes the interweived HWC image and splits it into 3 seprate matrix
        cv::split(rgb, ch);

        // Copy each channel's data into the correct offset in the flat blob.
        for (int c = 0; c < 3; ++c)
            std::memcpy(blob.data() + c * NET_H * NET_W, // destination offesting for each channel
                        ch[c].ptr<float>(),  // the source
                        NET_H * NET_W * sizeof(float)); // number of bites need to allocate

        // ── Inference ────────────────────────────────────────────────────────
        // Wrap the flat blob in an Ort::Value without copying — the tensor just
        // points at blob.data(). blob must stay alive for the duration of Run().
        std::array<int64_t, 4> shape{1, 3, NET_H, NET_W};
        auto mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

        // create our 4d tensor
        Ort::Value in = Ort::Value::CreateTensor<float>(
            mem, blob.data(), blob.size(), shape.data(), shape.size());

        const char* in_names[]  = {input_name_.c_str()}; // what input and output names to use
        const char* out_names[] = {output_name_.c_str()};
        auto outputs = session_->Run(
            Ort::RunOptions{nullptr},  // default run options
            in_names,                  // input names array
            &in,                       // pointer to input tensor
            1,                         // number of inputs (just 1)
            out_names,                 // output names array
            1                          // number of outputs (just 1)
        );

        // ── Decode output ────────────────────────────────────────────────────
        // YOLO11/YOLOv8 ONNX output shape: [1, 4+num_classes, num_anchors]
        //   e.g. [1, 84, 8400] for COCO (80 classes, 640x640 input → 8400 candidate boxes)
        //
        // Layout is row-major (C order): to access anchor i, row r:
        //   data[r * num_anchors + i]
        //
        // Rows 0-3: cx, cy, w, h in 640x640 pixel space (not normalized 0-1).
        // Rows 4+:  class probabilities, already sigmoid-activated by the export.
        //
        // In Python: ultralytics unpacked this tensor automatically inside predict().
        auto out_shape   = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
        int num_anchors  = static_cast<int>(out_shape[2]);
        int num_classes  = static_cast<int>(out_shape[1]) - 4;
        const float* d   = outputs[0].GetTensorData<float>();

        std::vector<Det> dets;
        for (int i = 0; i < num_anchors; ++i) {
            // Find the highest-scoring class for this anchor.
            float max_s = 0.f; int cls = 0;
            for (int c = 0; c < num_classes; ++c) {
                float s = d[(4 + c) * num_anchors + i];
                if (s > max_s) { max_s = s; cls = c; }
            }
            // Skip below-threshold candidates before doing the more expensive
            // coordinate math and NMS.
            if (max_s < conf_thresh_) continue;

            float cx = d[0 * num_anchors + i];
            float cy = d[1 * num_anchors + i];
            float w  = d[2 * num_anchors + i];
            float h  = d[3 * num_anchors + i];

            // Reverse the letterbox transform: subtract padding, divide by scale.
            // Clamp to the original image bounds to handle edge detections cleanly.
            auto unpad = [&](float v, float pad, int dim) {
                return std::min(std::max((v - pad) / scale, 0.f), float(dim));
            };
            dets.push_back(Det{
                unpad(cx - w/2, pad_x, frame.cols),
                unpad(cy - h/2, pad_y, frame.rows),
                unpad(cx + w/2, pad_x, frame.cols),
                unpad(cy + h/2, pad_y, frame.rows),
                max_s, cls
            });
        }

        // Remove duplicate overlapping boxes.
        dets = nms(dets, nms_thresh_);

        // ── Publish detections ───────────────────────────────────────────────
        // Same message structure as the Python node so pose_estimation_node.py
        // can subscribe without changes.


        vision_msgs::msg::Detection2DArray arr;
        arr.header = msg->header;
        for (const auto& det : dets) {
            vision_msgs::msg::Detection2D d2;
            d2.header = msg->header;
            d2.bbox.center.position.x = (det.x1 + det.x2) / 2.0;
            d2.bbox.center.position.y = (det.y1 + det.y2) / 2.0;
            d2.bbox.size_x = det.x2 - det.x1;
            d2.bbox.size_y = det.y2 - det.y1;

            vision_msgs::msg::ObjectHypothesisWithPose hyp;
            hyp.hypothesis.class_id = std::to_string(det.cls);
            hyp.hypothesis.score    = det.score;
            d2.results.push_back(hyp);
            arr.detections.push_back(d2);
        }
        det_pub_->publish(arr);
        publish_debug(frame, dets, msg->header);
    }

    void publish_debug(const cv::Mat& frame, const std::vector<Det>& dets,
                       const std_msgs::msg::Header& header)
    {
        // Skip drawing if nobody is listening — same optimization as Python node.
        if (debug_pub_->get_subscription_count() == 0) return;

        cv::Mat vis = frame.clone();
        for (const auto& d : dets) {
            cv::rectangle(vis, cv::Point(d.x1, d.y1), cv::Point(d.x2, d.y2),
                          cv::Scalar(0, 255, 0), 2);
            std::string lbl = "cls:" + std::to_string(d.cls) +
                              " " + std::to_string(int(d.score * 100)) + "%";
            int base;
            cv::Size ts = cv::getTextSize(lbl, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &base);
            int ty = std::max(int(d.y1) - 5, ts.height + 5);
            cv::rectangle(vis, cv::Point(d.x1, ty - ts.height - 5),
                          cv::Point(d.x1 + ts.width, ty + base - 5),
                          cv::Scalar(0, 255, 0), cv::FILLED);
            cv::putText(vis, lbl, cv::Point(d.x1, ty - 5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
        }
        debug_pub_->publish(*cv_bridge::CvImage(header, "bgr8", vis).toImageMsg());
    }

    // Ort::Env must be declared before session_ — it must outlive the session.
    // In Python this was hidden inside the YOLO() object.
    Ort::Env env_;
    std::unique_ptr<Ort::Session> session_;
    std::string input_name_, output_name_;
    float conf_thresh_, nms_thresh_;

    rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr det_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr             debug_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr          sub_;
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    // make_shared allocates the node and its members once on the heap.
    // Python's rclpy.spin() did the same thing but with reference-counted Python objects.
    rclcpp::spin(std::make_shared<DetectorNode>());
    rclcpp::shutdown();
}
