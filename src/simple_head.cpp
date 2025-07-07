#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>

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

    // Create the MoveGroupInterface for your planning group
    using moveit::planning_interface::MoveGroupInterface;
    auto head = MoveGroupInterface(node, "rby1_head");

    // Set head to initial position
    std::vector<double> joint_positions(2);
    joint_positions[0] = 20 * D2R;
    joint_positions[1] = 10 * D2R;
    head.setJointValueTarget(joint_positions);
    moveit::planning_interface::MoveGroupInterface::Plan joint_plan;
    if(head.plan(joint_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        head.execute(joint_plan);
    }

    //Shutdown
    rclcpp::shutdown();
    return 0;
}