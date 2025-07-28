#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include<tf2_ros/transform_listener.h>
#include<tf2_ros/buffer.h>

#define M_PI 3.14159265358979323846
#define D2R 0.017453288888888

int main(int argc, char * argv[])
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create a ROS 2 node
    auto const node = std::make_shared<rclcpp::Node>(
        "simple_head",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    auto const logger = rclcpp::get_logger("simple_head");

    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    rclcpp::sleep_for(std::chrono::milliseconds(1000));

    geometry_msgs::msg::TransformStamped base_to_hand;
    geometry_msgs::msg::TransformStamped base_to_head;

    bool got_hand_tf = false;
    bool got_head_tf = false;

    rclcpp::Rate rate(10.0);  // 10 Hz retry frequency
    RCLCPP_INFO(logger, "Waiting for both transforms to become available...");

    while (rclcpp::ok() && (!got_hand_tf || !got_head_tf)) {
        try {
            base_to_hand = tf_buffer->lookupTransform(
                "base", "ee_finger_r1", tf2::TimePointZero, tf2::durationFromSec(0.5));
            got_hand_tf = true;
        } catch (const tf2::TransformException & ex) {
            RCLCPP_DEBUG(logger, "Waiting for base->ee_finger_r1: %s", ex.what());
        }

        try {
            base_to_head = tf_buffer->lookupTransform(
                "base", "link_head_1", tf2::TimePointZero, tf2::durationFromSec(0.5));
            got_head_tf = true;
        } catch (const tf2::TransformException & ex) {
            RCLCPP_DEBUG(logger, "Waiting for base->link_head_1: %s", ex.what());
        }

        if (!got_hand_tf || !got_head_tf) {
            rate.sleep();  // Wait a bit and try again
        }
    }

    /*

    try {
       base_to_hand =
            tf_buffer->lookupTransform(
                "base",
                "ee_right",
                tf2::TimePointZero,
                tf2::durationFromSec(3.0)
            );
        RCLCPP_INFO(node->get_logger(), "Transform base -> ee_right:");
        RCLCPP_INFO(node->get_logger(),
                    "Translation: [%.3f, %.3f, %.3f]",
                    base_to_hand.transform.translation.x,
                    base_to_hand.transform.translation.y,
                    base_to_hand.transform.translation.z);
        RCLCPP_INFO(node->get_logger(),
                    "Rotation (quaternion): [%.3f, %.3f, %.3f, %.3f]",
                    base_to_hand.transform.rotation.x,
                    base_to_hand.transform.rotation.y,
                    base_to_hand.transform.rotation.z,
                    base_to_hand.transform.rotation.w);
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(node->get_logger(), "Could not transform: %s", ex.what());
      }

    try {
        base_to_head =
            tf_buffer->lookupTransform(
                "base",
                "link_head_1",
                tf2::TimePointZero,
                tf2::durationFromSec(3.0)
            );
        RCLCPP_INFO(node->get_logger(), "Transform base -> link_head_1:");
        RCLCPP_INFO(node->get_logger(),
                    "Translation: [%.3f, %.3f, %.3f]",
                    base_to_head.transform.translation.x,
                    base_to_head.transform.translation.y,
                    base_to_head.transform.translation.z);
        RCLCPP_INFO(node->get_logger(),
                    "Rotation (quaternion): [%.3f, %.3f, %.3f, %.3f]",
                    base_to_head.transform.rotation.x,
                    base_to_head.transform.rotation.y,
                    base_to_head.transform.rotation.z,
                    base_to_head.transform.rotation.w);
        } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(node->get_logger(), "Could not transform: %s", ex.what());
        }

    */

    // Print the transforms before moving the head
    RCLCPP_INFO(logger, "Transform base -> ee_finger_r1:");
    RCLCPP_INFO(logger, "  Translation: [%.3f, %.3f, %.3f]",
                base_to_hand.transform.translation.x,
                base_to_hand.transform.translation.y,
                base_to_hand.transform.translation.z);
    RCLCPP_INFO(logger, "  Rotation (quaternion): [%.3f, %.3f, %.3f, %.3f]",
                base_to_hand.transform.rotation.x,
                base_to_hand.transform.rotation.y,
                base_to_hand.transform.rotation.z,
                base_to_hand.transform.rotation.w);

    RCLCPP_INFO(logger, "Transform base -> link_head_1:");
    RCLCPP_INFO(logger, "  Translation: [%.3f, %.3f, %.3f]",
                base_to_head.transform.translation.x,
                base_to_head.transform.translation.y,
                base_to_head.transform.translation.z);
    RCLCPP_INFO(logger, "  Rotation (quaternion): [%.3f, %.3f, %.3f, %.3f]",
                base_to_head.transform.rotation.x,
                base_to_head.transform.rotation.y,
                base_to_head.transform.rotation.z,
                base_to_head.transform.rotation.w);

    // Calculate desired tracking angle from head to hand
    double dx = base_to_hand.transform.translation.x - base_to_head.transform.translation.x;
    double dy = base_to_hand.transform.translation.y - base_to_head.transform.translation.y;
    double dz = base_to_hand.transform.translation.z - base_to_head.transform.translation.z;


    double yaw = std::atan2(dy, dx);
    double pitch = std::atan2(-dz, dx);

    // Create the MoveGroupInterface for your planning group
    using moveit::planning_interface::MoveGroupInterface;
    auto head = MoveGroupInterface(node, "rby1_head");

    // Set head to initial position
    std::vector<double> joint_positions(2);
    joint_positions[0] = yaw; //20 * D2R;
    joint_positions[1] = pitch; //10 * D2R;
    head.setJointValueTarget(joint_positions);
    moveit::planning_interface::MoveGroupInterface::Plan joint_plan;
    if(head.plan(joint_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        head.execute(joint_plan);
    }

    /** Throws "failed to get robot state"
    std::vector<double> joint_values = head.getCurrentJointValues();
    std::cout << "Current joint values: ";
    for (const auto &joint : joint_values) {
        std::cout << joint << " ";
    }
    std::cout << std::endl;
    **/

    //Shutdown
    rclcpp::shutdown();
    return 0;
}