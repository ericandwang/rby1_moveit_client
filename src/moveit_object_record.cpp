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
    //const double dx = -0.01;
    //const double dy = -0.060;
    //const double dz = 0.015;

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
    std::vector<double> joint_positions_head(2);
    joint_positions_head[0] = yaw; //20 * D2R;
    joint_positions_head[1] = pitch; //10 * D2R;
    head.setJointValueTarget(joint_positions_head);
    moveit::planning_interface::MoveGroupInterface::Plan joint_plan_head;
    if(head.plan(joint_plan_head) == moveit::core::MoveItErrorCode::SUCCESS) {
        head.execute(joint_plan_head);
    }

    joint_positions[6] = -150 * D2R;
    right_arm.setJointValueTarget(joint_positions);
    if(right_arm.plan(joint_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        right_arm.execute(joint_plan);
    }

    joint_positions[6] = 150 * D2R;
    right_arm.setJointValueTarget(joint_positions);
    if(right_arm.plan(joint_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        right_arm.execute(joint_plan);
    }

    // Stop spinning
    running = false;
    spin_thread.join();

    //Shutdown
    rclcpp::shutdown();
    return 0;
}