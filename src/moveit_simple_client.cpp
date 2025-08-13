#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>

#define M_PI 3.14159265358979323846
#define D2R 0.017453288888888

std::atomic<bool> running{true};
rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
std::vector<double> latest_joint_positions_(24, 0.0);

void encoder_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    for (size_t i = 0; i < 24; ++i) {
        latest_joint_positions_[i] = msg->position[i];
    }
}

double get_encoder(const std::string& name) {
    // Use standard joint names from URDF
    if (name == "right_wheel") return latest_joint_positions_[0];
    else if (name == "left_wheel") return latest_joint_positions_[1];

    else if (name == "torso_0") return latest_joint_positions_[2];
    else if (name == "torso_1") return latest_joint_positions_[3];
    else if (name == "torso_2") return latest_joint_positions_[4];
    else if (name == "torso_3") return latest_joint_positions_[5];
    else if (name == "torso_4") return latest_joint_positions_[6];
    else if (name == "torso_5") return latest_joint_positions_[7];

    else if (name == "right_arm_0") return latest_joint_positions_[8];
    else if (name == "right_arm_1") return latest_joint_positions_[9];
    else if (name == "right_arm_2") return latest_joint_positions_[10];
    else if (name == "right_arm_3") return latest_joint_positions_[11];
    else if (name == "right_arm_4") return latest_joint_positions_[12];
    else if (name == "right_arm_5") return latest_joint_positions_[13];
    else if (name == "right_arm_6") return latest_joint_positions_[14];

    else if (name == "left_arm_0") return latest_joint_positions_[15];
    else if (name == "left_arm_1") return latest_joint_positions_[16];
    else if (name == "left_arm_2") return latest_joint_positions_[17];
    else if (name == "left_arm_3") return latest_joint_positions_[18];
    else if (name == "left_arm_4") return latest_joint_positions_[19];
    else if (name == "left_arm_5") return latest_joint_positions_[20];
    else if (name == "left_arm_6") return latest_joint_positions_[21];

    else if (name == "head_0") return latest_joint_positions_[22];
    else if (name == "head_1") return latest_joint_positions_[23];
    return 0.0;
}

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

    // script start
    auto joint_state_sub = node->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, encoder_callback);

    // Spin in a separate thread
    std::thread spin_thread([&](){
        while (running) {
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });

    // Offset from link_head_2 to camera_right
    const double dx = -0.01;
    const double dy = -0.060;
    const double dz = 0.015;

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
    //left_arm.setPoseReferenceFrame("camera_right");
    //right_arm.setPoseReferenceFrame("camera_right");

    // Set arm to initial manipulation position
    std::vector<double> joint_positions(7);
    joint_positions[0] = -16 * D2R;
    joint_positions[1] = -50 * D2R;
    joint_positions[2] = 0 * D2R;
    joint_positions[3] = -70 * D2R;
    joint_positions[4] = 0 * D2R;
    joint_positions[5] = 0 * D2R;
    joint_positions[6] = 38 * D2R;
    joint_positions[0] = -29 * D2R;
    joint_positions[1] = -45 * D2R;
    joint_positions[2] = 36 * D2R;
    joint_positions[3] = -96 * D2R;
    joint_positions[4] = 21 * D2R;
    joint_positions[5] = -53 * D2R;
    joint_positions[6] = 0 * D2R;
    joint_positions[0] = -0 * D2R;
    joint_positions[1] = -90 * D2R;
    joint_positions[2] = 0 * D2R;
    joint_positions[3] = -120 * D2R;
    joint_positions[4] = 0 * D2R;
    joint_positions[5] = -35 * D2R;
    joint_positions[6] = 0 * D2R;
    right_arm.setJointValueTarget(joint_positions);
    moveit::planning_interface::MoveGroupInterface::Plan joint_plan;
    if(right_arm.plan(joint_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        right_arm.execute(joint_plan);
    }

    std::cout << "right_arm_0: " << get_encoder("right_arm_0") << " rad" << std::endl;
    std::cout << "right_arm_1: " << get_encoder("right_arm_1") << " rad" << std::endl;
    std::cout << "right_arm_2: " << get_encoder("right_arm_2") << " rad" << std::endl;
    std::cout << "right_arm_3: " << get_encoder("right_arm_3") << " rad" << std::endl;
    std::cout << "right_arm_4: " << get_encoder("right_arm_4") << " rad" << std::endl;
    std::cout << "right_arm_5: " << get_encoder("right_arm_5") << " rad" << std::endl;
    std::cout << "right_arm_6: " << get_encoder("right_arm_6") << " rad" << std::endl;

    // Define the target poses
    //geometry_msgs::msg::Pose target_pose_left;
    geometry_msgs::msg::Pose target_pose_right;
    // left arm
    //target_pose_left.orientation.w = 1.0;
    //target_pose_left.position.x = 0.294;
    //target_pose_left.position.y = 0.219;
    //target_pose_left.position.z = -0.375;

    // // right arm orientation
    // Eigen::AngleAxisd roll(0, Eigen::Vector3d::UnitX());
    // Eigen::AngleAxisd pitch(0, Eigen::Vector3d::UnitY());
    // Eigen::AngleAxisd yaw(0, Eigen::Vector3d::UnitZ());
    // Eigen::Quaterniond q = yaw * pitch * roll;
    // target_pose_right.orientation.w = q.w();
    // target_pose_right.orientation.x = q.x();
    // target_pose_right.orientation.y = q.y();
    // target_pose_right.orientation.z = q.z();
    // // geometry_msgs::msg::Quaternion quat_msg;
    // // quat_msg.w = q.w();
    // // quat_msg.x = q.x();
    // // quat_msg.y = q.y();
    // // quat_msg.z = q.z();

    // // set fixed end effector orientation constraint
    // moveit_msgs::msg::OrientationConstraint ocm;
    // ocm.header.frame_id = "link_head_2"; // Reference frame (should match your pose goal)
    // ocm.link_name = "ee_right"; // End effector link
    // ocm.orientation = target_pose_right.orientation; // Desired orientation
    // ocm.absolute_x_axis_tolerance = 0.2; // Tolerance in radians (axes)
    // ocm.absolute_y_axis_tolerance = 0.2;
    // ocm.absolute_z_axis_tolerance = 0.2;
    // ocm.weight = 1.0;
   
    // // set soft box position constraint
    // moveit_msgs::msg::PositionConstraint pcm;
    // pcm.header.frame_id = "link_head_2";
    // pcm.link_name = "ee_right";
    // // Define a box region
    // shape_msgs::msg::SolidPrimitive box;
    // box.type = shape_msgs::msg::SolidPrimitive::BOX;
    // box.dimensions = {3, 3, 3}; // Size of the box (x, y, z in meters)
    // pcm.constraint_region.primitives.push_back(box);
    // // Set the pose (center) of the box
    // geometry_msgs::msg::Pose box_pose;
    // box_pose.position.x = 0.4;
    // box_pose.position.y = -0.3;
    // box_pose.position.z = -0.1; // Center of the constraint box
    // box_pose.orientation.w = 1.0; // No rotation
    // pcm.constraint_region.primitive_poses.push_back(box_pose);
    // // Optionally, set weight
    // pcm.weight = 0.5; // Lower weight means it's a softer constraint

    // // create path constraints, plan, and execute
    // moveit_msgs::msg::Constraints path_constraints;
    // path_constraints.orientation_constraints.push_back(ocm);
    // path_constraints.position_constraints.push_back(pcm);
    // // Set path constraints before planning
    // right_arm.setPathConstraints(path_constraints);
    // // Set your desired pose (target) and plan
    // right_arm.setPoseTarget(target_pose_right, "ee_right");
    // moveit::planning_interface::MoveGroupInterface::Plan right_plan;
    // right_arm.setPlanningTime(20.0);
    // bool success = (right_arm.plan(right_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    // if (success) {
    //     right_arm.execute(right_plan);
    // }

    // // first send to position 15 cm behind, 15 cm above, and 15 cm to the right of object
    // target_pose_right.position.x = 0.347 - dx;
    // target_pose_right.position.y = -0.035 - dy;
    // target_pose_right.position.z = -0.210 - dz;
    // //left_arm.setPoseTarget(target_pose_left,"ee_left");
    // right_arm.setPoseTarget(target_pose_right,"ee_right");
    // // Plan to the target poses
    // //moveit::planning_interface::MoveGroupInterface::Plan left_plan;
    // moveit::planning_interface::MoveGroupInterface::Plan right_plan;
    // //bool success_left = (left_arm.plan(left_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    // bool success = (right_arm.plan(right_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    // // Execute the plan if successful
    // std::this_thread::sleep_for(std::chrono::seconds(1));
    // if (success) {
    //     right_arm.execute(right_plan);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //     //left_arm.execute(left_plan);
    //     std::this_thread::sleep_for(std::chrono::seconds(1));
    //     RCLCPP_INFO(logger, "Motion executed!");
    // } else {
    //     RCLCPP_ERROR(logger, "Planning failed!");
    // }

    // Stop spinning
    running = false;
    spin_thread.join();

    //Shutdown
    rclcpp::shutdown();
    return 0;
}