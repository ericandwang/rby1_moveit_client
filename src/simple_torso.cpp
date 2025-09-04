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
        "simple_torso",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    auto const logger = rclcpp::get_logger("simple_torso");

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

    // Create the MoveGroupInterface for your planning group
    using moveit::planning_interface::MoveGroupInterface;
    auto torso = MoveGroupInterface(node, "rby1_torso");
    std::cout << "End effector link left: " << torso.getEndEffectorLink() << std::endl;

    // Set frame references
    torso.setPoseReferenceFrame("base");

    // Set torso to initial position
    std::vector<double> joint_positions(6);
    joint_positions[0] = 0 * D2R;
    joint_positions[1] = 10 * D2R;
    joint_positions[2] = -10 * D2R;
    joint_positions[3] = 5 * D2R;
    joint_positions[4] = 0 * D2R;
    joint_positions[5] = 0 * D2R;
    torso.setJointValueTarget(joint_positions);
    moveit::planning_interface::MoveGroupInterface::Plan joint_plan;
    if(torso.plan(joint_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        torso.execute(joint_plan);
    }

    // Define the target pose
    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.w = 1.0;
    target_pose.position.x = 0;
    target_pose.position.y = 0;
    target_pose.position.z = 1;
    //left_arm.setPoseTarget(target_pose_left,"ee_left");
    torso.setPoseTarget(target_pose,"link_torso_5");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    //bool success_left = (left_arm.plan(left_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    bool success = (torso.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    // Execute the plan if successful
    std::this_thread::sleep_for(std::chrono::seconds(1));
    if (success) {
        torso.execute(plan);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        //left_arm.execute(left_plan);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        RCLCPP_INFO(logger, "Motion executed!");
    } else {
        RCLCPP_ERROR(logger, "Planning failed!");
    }

    // Stop spinning
    running = false;
    spin_thread.join();

    //Shutdown
    rclcpp::shutdown();
    return 0;
}