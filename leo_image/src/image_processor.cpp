#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <string>
#include <vector>
#include <algorithm>

using std::placeholders::_1;
using std::placeholders::_2;

class RGBDZoneDetector : public rclcpp::Node
{
public:
    RGBDZoneDetector() : Node("image_processor")
    {
        // Declare parameter to control GUI — disable by default for headless safety
        this->declare_parameter("enable_gui", false);
        enable_gui_ = this->get_parameter("enable_gui").as_bool();

        zones_pub_ = this->create_publisher<std_msgs::msg::String>("detected_zones", 10);

        rgb_sub_.subscribe(this, "depth_camera/image");
        depth_sub_.subscribe(this, "depth_camera/depth_image");

        typedef message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncPolicy;

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), rgb_sub_, depth_sub_);
        sync_->registerCallback(std::bind(&RGBDZoneDetector::callback, this, _1, _2));

        RCLCPP_INFO(this->get_logger(), "GUI display %s", enable_gui_ ? "ENABLED" : "DISABLED");
    }

private:
    void callback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
                  const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg)
    {
        cv::Mat color, depth;
        try {
            // color = cv_bridge::toCvShare(rgb_msg, "bgr8")->image;
            auto depth_ptr = cv_bridge::toCvShare(depth_msg);
            depth = depth_ptr->image;

            if (depth_ptr->encoding == "16UC1") {
                depth.convertTo(depth, CV_32FC1, 0.001);  // mm → meters
            }
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        if (depth.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty depth image!");
            return;
        }

        // --- Obstacle detection ---
        float dist_left, dist_front, dist_right;
        compute_obstacle_distances(depth, dist_left, dist_front, dist_right);

        // RCLCPP_INFO(this->get_logger(),
        //     "Obstacle distances | Left: %.2f m | Front: %.2f m | Right: %.2f m",
        //     dist_left, dist_front, dist_right);

        std::string obstacle_zone = determine_obstacle_zone(dist_left, dist_front, dist_right);

        // // --- RED/BLUE detection ---
        // cv::Mat hsv;
        // cv::cvtColor(color, hsv, cv::COLOR_BGR2HSV);

        // float red_offset=0, red_dist=0;
        // float blue_offset=0, blue_dist=0;

        // bool red_detected = detect_color(hsv, depth, "RED", red_offset, red_dist);
        // bool blue_detected = detect_color(hsv, depth, "BLUE", blue_offset, blue_dist);

        // --- Publish single priority zone ---
        std::string final_zone;

        // if (red_detected) {
        //     final_zone = "RED," + std::to_string(red_offset) + "," + std::to_string(red_dist);
        //     RCLCPP_INFO(this->get_logger(), "ZONE: RED | offset=%.2f | dist=%.2f", red_offset, red_dist);
        // } else if (blue_detected) {
        //     final_zone = "BLUE," + std::to_string(blue_offset) + "," + std::to_string(blue_dist);
        //     RCLCPP_INFO(this->get_logger(), "ZONE: BLUE | offset=%.2f | dist=%.2f", blue_offset, blue_dist);
        // } 
        if (!obstacle_zone.empty()) {
            final_zone = obstacle_zone;
            RCLCPP_INFO(this->get_logger(), "ZONE: %s", obstacle_zone.c_str());
        } else {
            final_zone = "CLEAR";
            RCLCPP_INFO(this->get_logger(), "ZONE: CLEAR");
        }

        std_msgs::msg::String msg;
        msg.data = final_zone;
        zones_pub_->publish(msg);

        // --- Visualization (only when GUI is enabled) ---
        if (enable_gui_) {
            cv::Mat depth_vis;
            cv::normalize(depth, depth_vis, 0, 255, cv::NORM_MINMAX);
            depth_vis.convertTo(depth_vis, CV_8U);
            cv::applyColorMap(depth_vis, depth_vis, cv::COLORMAP_JET);
            cv::imshow("Depth", depth_vis);
            // cv::imshow("Color", color);
            cv::waitKey(1);
        }
    }

    void compute_obstacle_distances(const cv::Mat& depth,
                                    float& dist_left, float& dist_front, float& dist_right)
    {
        int w = depth.cols;
        int h = depth.rows;
        int zone_width = w / 3;

        cv::Rect left_zone(0,0,zone_width,h);
        cv::Rect front_zone(zone_width,0,zone_width,h);
        cv::Rect right_zone(2*zone_width,0,zone_width,h);

        cv::Mat d_left = depth(left_zone);
        cv::Mat d_front = depth(front_zone);
        cv::Mat d_right = depth(right_zone);

        cv::Mat mask_left = (d_left>0.05) & (d_left<10.0);
        cv::Mat mask_front = (d_front>0.05) & (d_front<10.0);
        cv::Mat mask_right = (d_right>0.05) & (d_right<10.0);

        dist_left = (cv::countNonZero(mask_left)>0) ? static_cast<float>(cv::mean(d_left, mask_left)[0]) : -0.1f;
        dist_front = (cv::countNonZero(mask_front)>0) ? static_cast<float>(cv::mean(d_front, mask_front)[0]) : -0.1f;
        dist_right = (cv::countNonZero(mask_right)>0) ? static_cast<float>(cv::mean(d_right, mask_right)[0]) : -0.1f;
    }

    std::string determine_obstacle_zone(float dist_left, float dist_front, float dist_right)
    {
        float threshold = 0.8f;
        if (dist_front>0 && dist_front<threshold) return "CORNER";
        else if (dist_left>0 && dist_left<threshold) return "LEFT";
        else if (dist_right>0 && dist_right<threshold) return "RIGHT";
        return "";
    }

    // bool detect_color(const cv::Mat& hsv, const cv::Mat& depth,
    //                   const std::string& color_name, float& offset_x, float& distance)
    // {
    //     cv::Mat mask;
    //     if (color_name=="RED") {
    //         cv::Mat mask1, mask2;
    //         cv::inRange(hsv, cv::Scalar(0,100,100), cv::Scalar(10,255,255), mask1);
    //         cv::inRange(hsv, cv::Scalar(160,100,100), cv::Scalar(179,255,255), mask2);
    //         cv::bitwise_or(mask1, mask2, mask);
    //     } else if (color_name=="BLUE") {
    //         cv::inRange(hsv, cv::Scalar(100,150,50), cv::Scalar(130,255,255), mask);
    //     } else return false;

    //     cv::erode(mask, mask, cv::Mat(), cv::Point(-1,-1), 2);
    //     cv::dilate(mask, mask, cv::Mat(), cv::Point(-1,-1), 2);

    //     std::vector<std::vector<cv::Point>> contours;
    //     cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    //     double max_area = 0;
    //     std::vector<cv::Point> max_contour;

    //     for (auto& c : contours) {
    //         double area = cv::contourArea(c);
    //         if (area > max_area) {
    //             max_area = area;
    //             max_contour = c;
    //         }
    //     }

    //     if (max_area < 400) return false;

    //     cv::Moments m = cv::moments(max_contour);
    //     if (m.m00==0) return false;

    //     int cx = int(m.m10/m.m00);
    //     int cy = int(m.m01/m.m00);

    //     cv::Mat mask_region = cv::Mat::zeros(mask.size(), CV_8U);
    //     cv::drawContours(mask_region, std::vector<std::vector<cv::Point>>{max_contour}, -1, 255, cv::FILLED);
    //     cv::Scalar mean_depth = cv::mean(depth, mask_region);
    //     distance = static_cast<float>(mean_depth[0]);

    //     if (std::isnan(distance) || distance<0.05 || distance>10.0) return false;

    //     offset_x = (cx - mask.cols/2.0f)/(mask.cols/2.0f);

    //     return true;
    // }

    bool enable_gui_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr zones_pub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    std::shared_ptr<message_filters::Synchronizer<
        message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::Image, sensor_msgs::msg::Image>>> sync_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RGBDZoneDetector>());
    rclcpp::shutdown();
    return 0;
}
