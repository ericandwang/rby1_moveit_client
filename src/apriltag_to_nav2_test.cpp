#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

using namespace std::chrono_literals;

class TagGoalNav2Node : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav2 = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    TagGoalNav2Node()
    : Node("tag_goal_nav2_node"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
        // Setup Nav2 action client
        nav2_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose_apriltag", 10);

        // Wait for Nav2 action server
        while (!nav2_client_->wait_for_action_server(2s)) {
            RCLCPP_INFO(this->get_logger(), "Waiting for Nav2 action server...");
        }

        // Timer to periodically send goal
        timer_ = this->create_wall_timer(
            3s, std::bind(&TagGoalNav2Node::send_tag_goal, this));
    }

private:
    void send_tag_goal()
    {
        // Lookup transform from tag36h11:3 to world
        geometry_msgs::msg::TransformStamped transformStamped;
        try {
            transformStamped = tf_buffer_.lookupTransform(
                "map",         // target frame
                "tag36h11:3",    // source frame
                tf2::TimePointZero); // latest
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
            return;
        }

        // Convert to PoseStamped
        geometry_msgs::msg::PoseStamped goal_pose;
        goal_pose.header.stamp = this->now();
        goal_pose.header.frame_id = "map";
        goal_pose.pose.position.x = transformStamped.transform.translation.x;
        goal_pose.pose.position.y = transformStamped.transform.translation.y;
        goal_pose.pose.position.z = 0.0; // For ground robots, z is usually 0

        // Extract yaw from the transform's rotation
        double roll, pitch, yaw;
        tf2::Quaternion tf_quat;
        tf2::fromMsg(transformStamped.transform.rotation, tf_quat);
        tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);

        // Create new quaternion with only yaw (roll=0, pitch=0)
        tf2::Quaternion flat_quat;
        flat_quat.setRPY(0, 0, yaw);
        goal_pose.pose.orientation = tf2::toMsg(flat_quat);

        // Add offsets
        double offset_x = -0.2; // -20 cm
        goal_pose.pose.position.x += offset_x * std::cos(yaw);
        goal_pose.pose.position.y += offset_x * std::sin(yaw);
        double offset_y = 0.2;
        goal_pose.pose.position.x += -offset_y * std::sin(yaw);
        goal_pose.pose.position.y +=  offset_y * std::cos(yaw);

        // Only yaw is used for Nav2, but we copy the full quaternion
        //goal_pose.pose.orientation = transformStamped.transform.rotation;

        pose_pub_->publish(goal_pose);

        // Prepare Nav2 goal
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = goal_pose;

        // Send goal
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            [this](std::shared_ptr<GoalHandleNav2> goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by Nav2");
                } else {
                    RCLCPP_INFO(this->get_logger(), "Goal accepted by Nav2");
                }
            };
        send_goal_options.result_callback =
            [this](const GoalHandleNav2::WrappedResult & result) {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Goal failed or was aborted");
                }
            };

        nav2_client_->async_send_goal(goal_msg, send_goal_options);
        RCLCPP_INFO(this->get_logger(), "Sent Nav2 goal at (%.2f, %.2f)", 
                    goal_pose.pose.position.x, goal_pose.pose.position.y);
    }

    rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TagGoalNav2Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
