#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <vector>

/**
 * image_human_processor — SPECIALISED human detector (depth-based).
 *
 * Unlike image_processor (which reports the mean depth of three coarse bands),
 * this node looks for PERSON-SHAPED blobs:
 *   1. Compute the depth gradient — a standing human is a FLAT depth plateau,
 *      while the floor is a steep depth ramp. Thresholding the gradient removes
 *      the floor/walls and leaves only "near + flat" regions (the human body).
 *   2. Connected components on that mask.
 *   3. Convert each blob's pixel size to REAL-WORLD metres using the D435
 *      intrinsics (world_w = px_w * depth / fx). Keep only blobs with
 *      person-like proportions (tall & narrow).
 *   4. Publish the closest valid human + its distance/angle.
 *
 * Outputs (all relative to the node's namespace, e.g. /robot_0/...):
 *   human_detected  std_msgs/Bool     — a person-shaped blob is in view
 *   human_close     std_msgs/Bool     — person within `close_distance` (feeds
 *                                       the {human_close} BT blackboard flag)
 *   human_distance  std_msgs/Float32  — distance to the closest human (m)
 *   human_angle     std_msgs/Float32  — angular offset (deg, + = LEFT)
 *   human_info      std_msgs/String   — debug string
 */

class HumanDetector : public rclcpp::Node
{
public:
    HumanDetector() : Node("image_human_processor")
    {
        this->declare_parameter("enable_gui", false);
        this->declare_parameter("max_range", 8.0);       // ignore depth beyond this (m) — camera far clip is 10 m
        this->declare_parameter("grad_threshold", 0.15); // depth gradient m/pixel (floor ramp is >> this)
        // Anthropometric human dimensions (loose — the 64×48 resolution inflates
        // blob bounding boxes via morphological close + floor merging, so we
        // keep the bounds wide; the fallback with its own validation catches
        // the close-range case.
        this->declare_parameter("human_min_width",   0.12);  // looser: at range the body fragments, keep thin parts
        // max width 1.60: the actor's close-range blob can reach 1.25-1.5 m
        // (spread legs + slight floor merge).  Walls are still rejected by the
        // side-context surface check, so this does NOT re-add wall false-positives.
        this->declare_parameter("human_max_width",   1.60);
        this->declare_parameter("human_min_height",  0.40);
        this->declare_parameter("human_max_height",  2.30);
        // Wall guard: a blob wider‑than‑tall is NOT a person.  0.35 keeps the
        // walking actor's close-range spread (legs + slight floor merge) while
        // rejecting flat floor strips.  Walls are still rejected by the
        // side-context surface check, so loosening here is safe.
        this->declare_parameter("human_min_aspect",   0.35);
        this->declare_parameter("human_min_world_area", 0.04);  // lets range fragments (0.13x0.5 m) through
        this->declare_parameter("close_distance", 1.5);  // human_close threshold (m)
        this->declare_parameter("close_frame_frac", 0.25); // fallback: person filling the frame
        this->declare_parameter("min_area", 25);           // min blob area (px)
        this->declare_parameter("close_kernel", 7);        // morphological close kernel (px) — bridges body fragments
        // Wall/surface guard: if a blob's LEFT and RIGHT sides continue at the
        // SAME depth (within this tolerance), it's a flat surface (wall), not
        // an isolated object like a person.
        this->declare_parameter("surface_depth_tol", 0.3);
        // Foreground isolation: keep only pixels within this many meters of the
        // NEAREST object, so a person never merges with the wall behind them.
        this->declare_parameter("foreground_band", 0.6);   // MAX foreground band (m); the actual band scales with distance
        // Wall report: only when a wall truly FILLS the view (≈90%+ of the
        // frame). Lower values made floor slivers / partial walls trigger
        // "wall is ahead" constantly.
        this->declare_parameter("wall_frame_frac", 0.90);
        // Temporal hold: once a human is detected, keep reporting it for this
        // many seconds so human_close doesn't flicker when the 64×48 depth
        // blob fragments frame‑to‑frame.
        this->declare_parameter("detection_hold_s", 1.0);
        // Camera HFOV in RADIANS — 87 deg as declared in leo_description's
        // macros.xacro. The sim's camera_info K matrix is inconsistent for the
        // 64x48 image (cx=320.5 > width), so we derive fx/fy from the FOV and
        // the ACTUAL image size instead — same approach as depth_to_scan.py.
        this->declare_parameter("hfov", 1.51844);

        enable_gui_ = this->get_parameter("enable_gui").as_bool();
        max_range_ = this->get_parameter("max_range").as_double();
        grad_thresh_ = this->get_parameter("grad_threshold").as_double();
        min_w_ = this->get_parameter("human_min_width").as_double();
        max_w_ = this->get_parameter("human_max_width").as_double();
        min_h_ = this->get_parameter("human_min_height").as_double();
        max_h_ = this->get_parameter("human_max_height").as_double();
        min_aspect_ = this->get_parameter("human_min_aspect").as_double();
        min_world_area_ = this->get_parameter("human_min_world_area").as_double();
        close_dist_ = this->get_parameter("close_distance").as_double();
        close_frame_frac_ = this->get_parameter("close_frame_frac").as_double();
        min_area_ = this->get_parameter("min_area").as_int();
        close_kernel_ = this->get_parameter("close_kernel").as_int();
        surface_tol_ = this->get_parameter("surface_depth_tol").as_double();
        hold_duration_ = this->get_parameter("detection_hold_s").as_double();
        foreground_band_ = this->get_parameter("foreground_band").as_double();
        wall_frame_frac_ = this->get_parameter("wall_frame_frac").as_double();
        hfov_ = this->get_parameter("hfov").as_double();

        detected_pub_ = this->create_publisher<std_msgs::msg::Bool>("human_detected", 10);
        close_pub_ = this->create_publisher<std_msgs::msg::Bool>("human_close", 10);
        distance_pub_ = this->create_publisher<std_msgs::msg::Float32>("human_distance", 10);
        angle_pub_ = this->create_publisher<std_msgs::msg::Float32>("human_angle", 10);
        info_pub_ = this->create_publisher<std_msgs::msg::String>("human_info", 10);
        wall_pub_ = this->create_publisher<std_msgs::msg::Bool>("wall_in_view", 10);

        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "depth_camera/depth_image", rclcpp::SensorDataQoS(),
            std::bind(&HumanDetector::depth_cb, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "HumanDetector started (GUI %s)",
                    enable_gui_ ? "ENABLED" : "DISABLED");
    }

private:
    void depth_cb(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat depth;
        try {
            auto depth_ptr = cv_bridge::toCvShare(msg);
            depth = depth_ptr->image;
            if (depth_ptr->encoding == "16UC1") {
                depth.convertTo(depth, CV_32FC1, 0.001);  // mm -> m
            }
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        if (depth.empty() || depth.channels() != 1) return;

        int w = depth.cols, h = depth.rows;

        // Derive the true focal length from the camera FOV and the ACTUAL image
        // size (the camera_info K matrix is inconsistent for the 64x48 image).
        // Same convention as depth_to_scan.py: 87 deg horizontal FOV.
        double vfov = 2.0 * std::atan(std::tan(hfov_ / 2.0) * h / w);
        double fx = (w / 2.0) / std::tan(hfov_ / 2.0);
        double fy = (h / 2.0) / std::tan(vfov / 2.0);
        double cx = w / 2.0;
        double cy = h / 2.0;
        int min_area = static_cast<int>(std::max(10.0, min_area_ * w * h / (640.0 * 480.0)));

        // ---- one-time log: confirm the ACTUAL depth resolution.  If this
        // prints 64x48, the running sim is still using an old robot model —
        // rebuild leo_description + fully restart Gazebo.
        if (!logged_size_) {
            RCLCPP_INFO(this->get_logger(),
                        "depth image %dx%d, fx=%.1f fy=%.1f, max_range %.1f m",
                        w, h, fx, fy, max_range_);
            logged_size_ = true;
        }

        // ---- 1) gradient of depth: human = flat plateau, floor = steep ramp ----
        cv::Mat gx, gy;
        cv::Sobel(depth, gx, CV_32F, 1, 0, 3);
        cv::Sobel(depth, gy, CV_32F, 0, 1, 3);
        cv::Mat grad = cv::abs(gx) + cv::abs(gy);

        // near + flat + valid
        cv::Mat mask = (depth > 0.2) & (depth < max_range_) & (grad < grad_thresh_);

        // fill small holes inside the body (noise, gaps between limbs)
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE,
                         cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(close_kernel_, close_kernel_)));
        mask.convertTo(mask, CV_8U);

        // ---- 2) connected components ----
        cv::Mat labels, stats, centroids;
        int n = cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8);

        bool detected = false;
        bool is_close = false;
        float best_dist = 1e9f;
        float best_angle = 0.0f;
        float best_w = 0.0f, best_h = 0.0f;
        cv::Rect best_box;
        bool via_fallback = false;

        // Track the LARGEST near+flat blob too — used by the wall report and
        // the debug "rejected" line.
        int largest_area = 0;
        int largest_i = -1;
        double largest_md = 0.0;
        int largest_bw = 0, largest_bh = 0;

        for (int i = 1; i < n; ++i) {   // skip label 0 (background)
            int area = stats.at<int>(i, cv::CC_STAT_AREA);
            if (area < min_area) continue;   // too small = noise

            cv::Mat blob_mask = (labels == i);

            // ---- 3a) this blob's OWN nearest depth ----
            // (NOT the global min — the floor right in front of the robot is
            // usually nearer than a human several metres away, and a global
            // band would drop the human entirely.)
            double blob_min_d = 0.0;
            cv::minMaxLoc(depth, &blob_min_d, nullptr, nullptr, nullptr, blob_mask);
            if (blob_min_d <= 0.2 || blob_min_d > max_range_) continue;

            // ---- 3b) near slice: only THIS blob's pixels within band of its
            // own nearest pixel.  The band SCALES with distance: tight up close
            // (less floor/wall merging -> the blob stays person-shaped instead
            // of going 'too squat'/'too wide'), wider at range (keeps the noisy
            // or angled body together).  The body is ~0.3 m deep at any range.
            double band = std::clamp(0.2 + 0.08 * blob_min_d, 0.25, foreground_band_);
            cv::Mat near_mask;
            cv::bitwise_and(blob_mask,
                            (depth > 0.2) & (depth < blob_min_d + band),
                            near_mask);
            std::vector<cv::Point> pts;
            cv::findNonZero(near_mask, pts);
            if (static_cast<int>(pts.size()) < min_area) continue;

            cv::Rect box = cv::boundingRect(pts);
            int b_cx = box.x + box.width / 2;
            double mean_d = cv::mean(depth, near_mask)[0];
            if (mean_d <= 0.2 || mean_d > max_range_) continue;

            double world_w = box.width * mean_d / fx;
            double world_h = box.height * mean_d / fy;

            if (static_cast<int>(pts.size()) > largest_area) {
                largest_area = static_cast<int>(pts.size());
                largest_i = i;
                largest_md = mean_d;
                largest_bw = box.width;
                largest_bh = box.height;
            }

            // ---- 4) person-like?  Anthropometric filter (adult dimensions) ----
            std::string why = "";
            if (world_w < min_w_) why = "too narrow";
            else if (world_w > max_w_) why = "too wide";
            else if (world_h < min_h_) why = "too short";
            else if (world_h > max_h_) why = "too tall";
            else if (world_h / world_w < min_aspect_) why = "too squat";
            else if (world_w * world_h < min_world_area_) why = "area too small";
            if (!why.empty()) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "rejected blob: %.2f x %.2f m @ %.2f m (%s)",
                    world_w, world_h, mean_d, why.c_str());
                continue;
            }

            // ---- 4b) wall/surface guard: a flat surface continues at the SAME
            // depth on BOTH sides of the blob; a person has BACKGROUND behind it.
            // This kills the "HUMAN when looking at a wall" false positive.
            bool surface_ok = true;
            {
                int mid_row = box.y + box.height / 2;
                int span = std::max(1, box.height / 4);
                int margin = std::max(4, box.width / 20);
                int both = 0;
                for (int r = mid_row - span; r <= mid_row + span; ++r) {
                    if (r < 0 || r >= h) continue;
                    int cl = box.x - margin;
                    int cr = box.x + box.width + margin;
                    if (cl < 0 || cr >= w) continue;   // frame edge: can't tell
                    double dl = depth.at<float>(r, cl);
                    double dr = depth.at<float>(r, cr);
                    bool lsame = (dl > 0.2 && std::abs(dl - mean_d) < surface_tol_);
                    bool rsame = (dr > 0.2 && std::abs(dr - mean_d) < surface_tol_);
                    if (lsame && rsame) both++;
                }
                if (both > span) surface_ok = false;   // wall fills both sides
            }
            if (!surface_ok) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "rejected blob: %.2f x %.2f m @ %.2f m (flat surface/wall)",
                    world_w, world_h, mean_d);
                continue;
            }

            // ---- 5) pick the CLOSEST human ----
            if (mean_d < best_dist) {
                best_dist = static_cast<float>(mean_d);
                // + = LEFT (image x smaller than principal point => left of camera)
                best_angle = static_cast<float>(-(b_cx - cx) / fx * 180.0 / M_PI);
                best_w = static_cast<float>(world_w);
                best_h = static_cast<float>(world_h);
                best_box = box;
                detected = true;
                is_close = (best_dist < close_dist_);
            }
        }

        // ---- debug: show why the biggest blob was rejected (helps tuning) ----
        if (!detected && largest_i > 0) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "biggest blob %.2f x %.2f m @ %.2f m (%.0f%% frame) - rejected by size filter",
                largest_bw * largest_md / fx, largest_bh * largest_md / fy,
                largest_md, 100.0 * largest_area / (w * h));
        }

        // ---- WALL report: a big near+flat slab that is NOT a person ----
        // Only when it truly FILLS the view (~90%+ of the frame).
        bool wall_in_view = false;
        if (!detected && largest_i > 0) {
            double ww = largest_bw * largest_md / fx;
            double frame_frac = static_cast<double>(largest_area) / (w * h);
            if (ww > max_w_ && frame_frac > wall_frame_frac_) {
                wall_in_view = true;
            }
        }

        // ---- publish ----

        // --- temporal hysteresis: hold detection so human_close doesn't ---
        // --- flicker when the 64×48 blob fragments frame‑to‑frame.      ---
        double now_s = this->now().seconds();
        if (detected) {
            last_det_ = now_s;
            last_dist_ = best_dist;  last_angle_ = best_angle;
            last_w_ = best_w;        last_h_ = best_h;
            last_close_ = is_close;  last_box_ = best_box;
        }
        if (!detected && last_det_ > 0 && (now_s - last_det_) < hold_duration_) {
            detected = true;
            is_close = last_close_;
            best_dist = last_dist_;  best_angle = last_angle_;
            best_w = last_w_;        best_h = last_h_;
            best_box = last_box_;
            via_fallback = false;
        }

        // ---- publish ----
        std_msgs::msg::Bool b;
        b.data = detected;
        detected_pub_->publish(b);
        b.data = is_close;
        close_pub_->publish(b);
        b.data = wall_in_view;
        wall_pub_->publish(b);

        std_msgs::msg::Float32 f;
        f.data = best_dist;
        distance_pub_->publish(f);
        f.data = best_angle;
        angle_pub_->publish(f);

        std_msgs::msg::String s;
        std::string state;
        if (detected) {
            state = "HUMAN";
            s.data = state;
            // Human: keep throttled updates (distance changes are useful)
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "HUMAN at %.2f m, %.1f deg (%s), %.2f x %.2f m%s%s",
                best_dist, best_angle,
                best_angle > 5.0 ? "left" : (best_angle < -5.0 ? "right" : "front"),
                best_w, best_h, is_close ? "  [CLOSE]" : "",
                via_fallback ? "  (frame-fill fallback)" : "");
        } else if (wall_in_view) {
            state = "WALL";
            s.data = state;
            // Wall / clear: log only on STATE CHANGES so the console isn't
            // spammed every frame.
            if (state != last_state_) {
                RCLCPP_INFO(this->get_logger(), "wall is ahead");
            }
        } else {
            state = "CLEAR";
            s.data = "NO_HUMAN";
            if (state != last_state_) {
                RCLCPP_INFO(this->get_logger(), "view clear");
            }
        }
        last_state_ = state;
        info_pub_->publish(s);

        // ---- optional GUI ----
        if (enable_gui_) {
            cv::Mat vis;
            cv::normalize(depth, vis, 0, 255, cv::NORM_MINMAX);
            vis.convertTo(vis, CV_8U);
            cv::applyColorMap(vis, vis, cv::COLORMAP_JET);
            if (detected) {
                cv::rectangle(vis, best_box, cv::Scalar(0, 255, 0), 2);
                cv::putText(vis, cv::format("%.2fm", best_dist),
                            cv::Point(best_box.x, std::max(best_box.y - 5, 0)),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
            }
            cv::imshow("HumanDetector", vis);
            cv::waitKey(1);
        }
    }

    bool enable_gui_;
    double max_range_, grad_thresh_;
    double min_w_, max_w_, min_h_, max_h_;
    double min_aspect_, min_world_area_;
    double close_dist_;
    double close_frame_frac_;
    double foreground_band_;
    double wall_frame_frac_;
    double hfov_;
    double hold_duration_;
    int min_area_;
    int close_kernel_;
    double surface_tol_;
    bool logged_size_ = false;

    // detection hold state
    double last_det_ = 0.0;
    float last_dist_ = 1e9f, last_angle_ = 0.0f, last_w_ = 0.0f, last_h_ = 0.0f;
    bool last_close_ = false;
    cv::Rect last_box_;

    // last published state (HUMAN / WALL / CLEAR) — for edge-triggered logs
    std::string last_state_ = "CLEAR";

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr detected_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr close_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr info_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr wall_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HumanDetector>());
    rclcpp::shutdown();
    return 0;
}
