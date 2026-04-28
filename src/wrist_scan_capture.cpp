#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/empty.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <filesystem>
#include <fstream>
#include <chrono>
#include <ctime>

class WristScanCapture : public rclcpp::Node
{
public:
    WristScanCapture() : Node("wrist_scan_capture")
    {
        // Create timestamped session directory
        auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        char buf[32];
        std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", std::localtime(&t));
        session_dir_ = std::string(getenv("HOME")) + "/scan_data/scan_" + buf;
        std::filesystem::create_directories(session_dir_);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        color_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_rect_raw", 10,
            [this](sensor_msgs::msg::Image::SharedPtr msg) { latest_color_ = msg; });

        depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/depth/image_rect_raw", 10,
            [this](sensor_msgs::msg::Image::SharedPtr msg) { latest_depth_ = msg; });

        trigger_sub_ = create_subscription<std_msgs::msg::Empty>(
            "/wrist_scan/capture", 10,
            [this](std_msgs::msg::Empty::SharedPtr) { capture(); });

        RCLCPP_INFO(get_logger(), "WristScanCapture ready. Saving to: %s", session_dir_.c_str());
    }

private:
    void capture()
    {
        if (!latest_color_ || !latest_depth_) {
            RCLCPP_WARN(get_logger(), "No images yet, skipping capture %d", frame_count_);
            return;
        }

        char prefix[16];
        std::snprintf(prefix, sizeof(prefix), "%03d", frame_count_);
        std::string base = session_dir_ + "/" + prefix;

        // Save color
        try {
            auto cv_color = cv_bridge::toCvCopy(latest_color_, "bgr8");
            cv::imwrite(base + "_color.png", cv_color->image);
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "Color conversion failed: %s", e.what());
            return;
        }

        // Save depth (16-bit, millimeters)
        try {
            auto cv_depth = cv_bridge::toCvCopy(latest_depth_, "16UC1");
            cv::imwrite(base + "_depth.png", cv_depth->image);
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "Depth conversion failed: %s", e.what());
            return;
        }

        // Save transforms
        std::ofstream f(base + "_transforms.json");
        f << "{\n";
        const std::vector<std::pair<std::string,std::string>> frames = {
            {"link_left_arm_6", "camera_link"},
            {"ee_right",        "object_link"}
        };
        for (size_t i = 0; i < frames.size(); ++i) {
            const auto& [frame, label] = frames[i];
            try {
                auto tf = tf_buffer_->lookupTransform("base", frame, tf2::TimePointZero);
                const auto& tr = tf.transform.translation;
                const auto& ro = tf.transform.rotation;
                f << "  \"" << label << "\": {\n"
                  << "    \"translation\": [" << tr.x << ", " << tr.y << ", " << tr.z << "],\n"
                  << "    \"rotation_xyzw\": [" << ro.x << ", " << ro.y << ", " << ro.z << ", " << ro.w << "]\n"
                  << "  }" << (i + 1 < frames.size() ? "," : "") << "\n";
            } catch (const tf2::TransformException& e) {
                RCLCPP_WARN(get_logger(), "TF lookup failed for %s: %s", frame.c_str(), e.what());
                f << "  \"" << label << "\": null" << (i + 1 < frames.size() ? "," : "") << "\n";
            }
        }
        f << "}\n";

        RCLCPP_INFO(get_logger(), "Captured frame %03d", frame_count_);
        frame_count_++;
    }

    std::string session_dir_;
    int frame_count_ = 0;
    sensor_msgs::msg::Image::SharedPtr latest_color_, latest_depth_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_, depth_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr trigger_sub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WristScanCapture>());
    rclcpp::shutdown();
    return 0;
}
