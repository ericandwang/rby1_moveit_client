#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

#define M_PI 3.14159265358979323846
#define D2R 0.017453288888888

int main(int argc, char * argv[])
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create a ROS 2 node
    auto const node = std::make_shared<rclcpp::Node>(
        "moveit_simple_client",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    auto const logger = rclcpp::get_logger("moveit_simple_client");

    // Create the MoveGroupInterface for your planning group
    using moveit::planning_interface::MoveGroupInterface;
    //auto dual_arm = MoveGroupInterface(node, "rby1_dualarm");
    auto left_arm = MoveGroupInterface(node, "rby1_left_arm");
    auto right_arm = MoveGroupInterface(node, "rby1_right_arm");
    std::cout << "End effector link left: " << left_arm.getEndEffectorLink() << std::endl;
    std::cout << "End effector link right: " << right_arm.getEndEffectorLink() << std::endl;
    //std::cout << "End effector: " << dual_arm.getEndEffectorLink() << std::endl;

    // Set frame references
    left_arm.setPoseReferenceFrame("link_head_2");
    right_arm.setPoseReferenceFrame("link_head_2");

    // Set arm to initial manipulation position
    std::vector<double> joint_positions(7);
    joint_positions[0] = -16 * D2R;
    joint_positions[1] = -50 * D2R;
    joint_positions[2] = 0 * D2R;
    joint_positions[3] = -70 * D2R;
    joint_positions[4] = 0 * D2R;
    joint_positions[5] = 0 * D2R;
    joint_positions[6] = 38 * D2R;
    right_arm.setJointValueTarget(joint_positions);
    moveit::planning_interface::MoveGroupInterface::Plan joint_plan;
    if(right_arm.plan(joint_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        right_arm.execute(joint_plan);
    }

    // Define the target poses
    geometry_msgs::msg::Pose target_pose_left;
    geometry_msgs::msg::Pose target_pose_right;
    // left arm
    target_pose_left.orientation.w = 1.0;
    target_pose_left.position.x = 0.294;
    target_pose_left.position.y = 0.219;
    target_pose_left.position.z = -0.375;

    // right arm
    //Eigen::AngleAxisd roll_angle(-M_PI/2, Eigen::Vector3d::UnitX());
    //Eigen::AngleAxisd pitch_angle(0, Eigen::Vector3d::UnitY());
    //Eigen::AngleAxisd yaw_angle(M_PI/2, Eigen::Vector3d::UnitZ());
  
    //Eigen::Quaterniond q = roll_angle * pitch_angle * yaw_angle;
    //q = q.normalized();
    
    Eigen::AngleAxisd roll(-M_PI/2, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch(0.2, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw(M_PI/2, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = yaw * pitch * roll;
    target_pose_right.orientation.w = q.w();
    target_pose_right.orientation.x = q.x();
    target_pose_right.orientation.y = q.y();
    target_pose_right.orientation.z = q.z();
    std::cout << "Orientation: "
          << "w=" << target_pose_right.orientation.w << ", "
          << "x=" << target_pose_right.orientation.x << ", "
          << "y=" << target_pose_right.orientation.y << ", "
          << "z=" << target_pose_right.orientation.z << std::endl;
    target_pose_right.position.x = 0.47;
    target_pose_right.position.y = -0.116;
    target_pose_right.position.z = -0.375;


    left_arm.setPoseTarget(target_pose_left,"ee_left");
    right_arm.setPoseTarget(target_pose_right,"ee_right");
    //dual_arm.setPoseTarget(target_pose_left,"link_left_arm_6");
    //dual_arm.setPoseTarget(target_pose_right,"link_right_arm_6");
    


    // Plan to the target poses
    //moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::planning_interface::MoveGroupInterface::Plan left_plan, right_plan;
    bool success_left = (left_arm.plan(left_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    bool success_right = (right_arm.plan(right_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    //bool success = (dual_arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    //bool success = (left_arm.plan(left_plan) && right_arm.plan(right_plan)) == moveit::core::MoveItErrorCode::SUCCESS;

    // Execute the plan if successful
    std::this_thread::sleep_for(std::chrono::seconds(2));
    if (success_left && success_right) {
        right_arm.execute(right_plan);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        left_arm.execute(left_plan);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        RCLCPP_INFO(logger, "Motion executed!");
    } else {
        RCLCPP_ERROR(logger, "Planning failed!");
    }


    // Execute the plan if successful
    /*
    std::this_thread::sleep_for(std::chrono::seconds(2));
    if (success) {
        dual_arm.execute(plan);
        RCLCPP_INFO(logger, "Motion executed!");
    } else {
        RCLCPP_ERROR(logger, "Planning failed!");
    }
    */

    //Shutdown
    rclcpp::shutdown();
    return 0;
}