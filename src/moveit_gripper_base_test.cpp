#include <rclcpp/rclcpp.hpp>
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"
#include "rby1-sdk/upc/device.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <thread>
#include <atomic>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <cmath>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <chrono>

#define PROTOCOL_VERSION 2.0
#define BAUDRATE 2000000

#define ADDR_TORQUE_ENABLE 64
#define ADDR_PRESENT_POSITION 132
#define ADDR_GOAL_CURRENT 102
#define ADDR_GOAL_POSITION 116
#define ADDR_OPERATING_MODE 11

#define CURRENT_CONTROL_MODE 0
#define CURRENT_BASED_POSITION_CONTROL_MODE 5

#define MIN_INDEX 0
#define MAX_INDEX 1

#define DISTANCE_TO_WHEEL_ANGLE 10
#define SPEED_TO_WHEEL_W 1 //10
#define ANGLE_TO_WHEEL_ANGLE 2.65
#define W_TO_WHEEL_W 2.65

#define M_PI 3.14159265358979323846
#define D2R 0.017453288888888
#define R2D 57.29579143313326

using namespace rb;
const std::string kAll = ".*";
std::vector<Eigen::Matrix<double, 2, 1>> q_min_max_vector;
double object_position_x = 0.47;
double object_position_y = -0.116;
double object_position_z = -0.375;
std::atomic<bool> running{true};
std::atomic<bool> running2{true};
rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
std::vector<double> latest_joint_positions_(2, 0.0);

void apriltag_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    object_position_x =  msg->pose.position.z;
    object_position_y = -msg->pose.position.x;
    object_position_z = -msg->pose.position.y;
    std::cout << "AprilTag position: x = " << object_position_x
                               << ", y = " << object_position_y
                               << ", z = " << object_position_z << std::endl;
}

void TorqueEnable(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler, int id, int onoff) {
    packetHandler->write1ByteTxOnly(portHandler, id, ADDR_TORQUE_ENABLE, onoff);
    std::this_thread::sleep_for(std::chrono::microseconds(500));
}

std::optional<int> ReadTorqueEnable(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler, int id) {
    int8_t onoff = -1;
    uint8_t dxl_error = 0;
    int dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, (uint8_t*)&onoff, &dxl_error);

    if (dxl_comm_result == COMM_SUCCESS) {
        return onoff;
    } else {
        return {};
    }
}

std::optional<int> ReadOperationMode(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler, int id) {
    int8_t operation_mode = -1;
    uint8_t dxl_error = 0;
    int dxl_comm_result =
    packetHandler->read1ByteTxRx(portHandler, id, ADDR_OPERATING_MODE, (uint8_t*)&operation_mode, &dxl_error);

    if (dxl_comm_result == COMM_SUCCESS) {
        return operation_mode;
    } else {
        return {};
    }
}

std::optional<double> ReadEncoder(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler, int id) {
    int32_t position = 0;
    uint8_t dxl_error = 0;
    int dxl_comm_result =
    packetHandler->read4ByteTxRx(portHandler, id, ADDR_PRESENT_POSITION, (uint32_t*)&position, &dxl_error);

    if (dxl_comm_result == COMM_SUCCESS) {
        return (double)position / 4096. * 2. * 3.141592;  // unit [rad]
    } else {
        return {};
    }
}

void SendCurrent(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler, int id, double current) {
    //current unit is [A]
    int32_t current_value = (int)(current / 2.69 * 1000.);
    packetHandler->write2ByteTxOnly(portHandler, id, ADDR_GOAL_CURRENT, current_value);
}

void SendOperationMode(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler, int id, int operation_mode) {
    packetHandler->write1ByteTxOnly(portHandler, id, ADDR_OPERATING_MODE, operation_mode);
    std::this_thread::sleep_for(std::chrono::microseconds(500));
}

void SendGoalPosition(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler, int id, int goal_position) {
    packetHandler->write4ByteTxOnly(portHandler, id, ADDR_GOAL_POSITION, goal_position);
    std::this_thread::sleep_for(std::chrono::microseconds(500));
}

void initialization_for_gripper(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler, std::vector<int> activeIDs) {
    q_min_max_vector.push_back(Eigen::Matrix<double, 2, 1>::Zero());
    q_min_max_vector.push_back(Eigen::Matrix<double, 2, 1>::Zero());

    while (1) {
        static int cnt = 0;
        int is_init = true;

        if (activeIDs.size() != 2) {
        std::cout << "The number of Dynamixels for hand gripper does not match the configuration\n";
        return;
        }

        // total moving angle 540 deg

        for (auto const& id : activeIDs) {
            while (1) {

                std::optional<int> operation_mode = ReadOperationMode(portHandler, packetHandler, id);

                if (operation_mode.has_value()) {
                if (operation_mode.value() != CURRENT_CONTROL_MODE) {
                    TorqueEnable(portHandler, packetHandler, id, 0);
                    SendOperationMode(portHandler, packetHandler, id, CURRENT_CONTROL_MODE);
                    TorqueEnable(portHandler, packetHandler, id, 1);
                    std::cout << "try to change control mode, id : " << id << std::endl;
                } else {
                    break;
                }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            while (1) {

                std::optional<int> torque_enable = ReadTorqueEnable(portHandler, packetHandler, id);

                if (torque_enable.has_value()) {
                if (!torque_enable.value()) {
                    TorqueEnable(portHandler, packetHandler, id, 1);
                    std::cout << "try to enable torque, id : " << id << std::endl;
                } else {
                    break;
                }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            if (cnt % 2 == 0) {
                std::optional<double> q = ReadEncoder(portHandler, packetHandler, id);
                if (q.has_value()) {
                q_min_max_vector[id](cnt % 2) = q.value();
                }
                SendCurrent(portHandler, packetHandler, id, 0.5);
            } else {
                std::optional<double> q = ReadEncoder(portHandler, packetHandler, id);
                if (q.has_value()) {
                q_min_max_vector[id](cnt % 2) = q.value();
                }
                SendCurrent(portHandler, packetHandler, id, -0.5);
            }

            if ((double)(abs(q_min_max_vector[id](MAX_INDEX) - q_min_max_vector[id](MIN_INDEX))) * 180 / 3.141592 <
                540 * 0.9) {
                is_init = false;
            }
        }

        if (is_init) {
            for (auto const& id : activeIDs) {
                if (q_min_max_vector[id](MIN_INDEX) > q_min_max_vector[id](MAX_INDEX)) {
                double temp = q_min_max_vector[id](MIN_INDEX);
                q_min_max_vector[id](MIN_INDEX) = q_min_max_vector[id](MAX_INDEX);
                q_min_max_vector[id](MAX_INDEX) = temp;
                }

                SendCurrent(portHandler, packetHandler, id, 0.5);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(3000));
            break;
        }

        cnt++;

        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    }
    std::cout << "finish init\n";
}

void gripper_position_command(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler, std::vector<int> activeIDs, double right_goal, double left_goal) {
    for (auto const& id : activeIDs) {
        std::optional<int> operation_mode = ReadOperationMode(portHandler, packetHandler, id);
        if (operation_mode.has_value()) {
            if (operation_mode.value() != CURRENT_BASED_POSITION_CONTROL_MODE) {
                TorqueEnable(portHandler, packetHandler, id, 0);
                SendOperationMode(portHandler, packetHandler, id, CURRENT_BASED_POSITION_CONTROL_MODE);
                TorqueEnable(portHandler, packetHandler, id, 1);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            double goal_position = 0;
            if (id == 0) {
                goal_position = left_goal * q_min_max_vector[id](MAX_INDEX) +
                                    (1. - left_goal) * q_min_max_vector[id](MIN_INDEX);
            } else {
                goal_position = right_goal * q_min_max_vector[id](MAX_INDEX) +
                                    (1. - right_goal) * q_min_max_vector[id](MIN_INDEX);
            }
            SendGoalPosition(portHandler, packetHandler, id, (int)(goal_position * 4096. / 3.141592 / 2.));
        }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

double get_wheel_position(const std::string& wheel_name) {
    // Use standard joint names from URDF
    if (wheel_name == "right_wheel") return latest_joint_positions_[0];
    if (wheel_name == "left_wheel") return latest_joint_positions_[1];
    return 0.0;
}

void encoder_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    latest_joint_positions_[0] = msg->position[0];
    latest_joint_positions_[1] = msg->position[1];
}

void set_SE2_speed(double linear, double angular, const rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr& publisher,
    geometry_msgs::msg::TwistStamped& msg, const rclcpp::Node::SharedPtr& node){
    msg.header.stamp = node->now();
    msg.twist.linear.x = linear;
    msg.twist.angular.z = angular;
    publisher->publish(msg);
}


void move_linear(double distance, double speed, const rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr& publisher,
    geometry_msgs::msg::TwistStamped& msg, const rclcpp::Node::SharedPtr& node){
    // reading current position
    double start_pos_r = get_wheel_position("right_wheel");
    double start_pos_l = get_wheel_position("left_wheel");

    std::cout << "Current right wheel position before linear: " << get_wheel_position("right_wheel") << " rad" << std::endl;
    std::cout << "Current left wheel position before linear: " << get_wheel_position("left_wheel") << " rad" << std::endl;

    auto start = std::chrono::steady_clock::now();

    // reading encoder to measure distance and sending linear velocity command
    rclcpp::Rate rate(100); // 100 Hz
    while (std::abs(get_wheel_position("right_wheel") - start_pos_r) < distance*DISTANCE_TO_WHEEL_ANGLE ||
           std::abs(get_wheel_position("left_wheel") - start_pos_l) < distance*DISTANCE_TO_WHEEL_ANGLE) {
        set_SE2_speed(speed, 0.0, publisher, msg, node);
        rate.sleep();
    }

    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "Linear command loop execution time: " << elapsed_seconds.count() << " seconds" << std::endl;
    std::cout << "Wheel angle traversed: " << std::abs(get_wheel_position("right_wheel") - start_pos_r) << " rad" << std::endl;

    std::cout << "Current right wheel position after linear: " << get_wheel_position("right_wheel") << " rad" << std::endl;
    std::cout << "Current left wheel position after linear: " << get_wheel_position("left_wheel") << " rad" << std::endl;

    // sending stop command
    set_SE2_speed(0.0, 0.0, publisher, msg, node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    set_SE2_speed(0.0, 0.0, publisher, msg, node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    set_SE2_speed(0.0, 0.0, publisher, msg, node);
}

void move_angular(double angle, double w, const rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr& publisher,
    geometry_msgs::msg::TwistStamped& msg, const rclcpp::Node::SharedPtr& node){
    // reading current position
    double start_pos_r = get_wheel_position("right_wheel");
    double start_pos_l = get_wheel_position("left_wheel");

    std::cout << "Current right wheel position before angular: " << get_wheel_position("right_wheel") << " rad" << std::endl;
    std::cout << "Current left wheel position before angular: " << get_wheel_position("left_wheel") << " rad" << std::endl;

    auto start = std::chrono::steady_clock::now();

    // reading encoder to measure distance and sending angular velocity command
    rclcpp::Rate rate(100); // 100 Hz
    while (std::abs(get_wheel_position("right_wheel") - start_pos_r) < angle*ANGLE_TO_WHEEL_ANGLE ||
        std::abs(get_wheel_position("left_wheel") - start_pos_l) < angle*ANGLE_TO_WHEEL_ANGLE) {
        set_SE2_speed(0.0, w, publisher, msg, node);
        rate.sleep();
    }

    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "Angular command loop execution time: " << elapsed_seconds.count() << " seconds" << std::endl;
    std::cout << "Wheel angle traversed: " << std::abs(get_wheel_position("right_wheel") - start_pos_r) << " rad" << std::endl;

    std::cout << "Current right wheel position after angular: " << get_wheel_position("right_wheel") << " rad" << std::endl;
    std::cout << "Current left wheel position after angular: " << get_wheel_position("left_wheel") << " rad" << std::endl;

    // sending stop command
    set_SE2_speed(0.0, 0.0, publisher, msg, node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    set_SE2_speed(0.0, 0.0, publisher, msg, node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    set_SE2_speed(0.0, 0.0, publisher, msg, node);
}

int main(int argc, char * argv[])
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create a ROS 2 node
    auto const node = std::make_shared<rclcpp::Node>(
        "moveit_gripper_base_test",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    auto const logger = rclcpp::get_logger("moveit_gripper_base_test");

    auto subscription = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/apriltag/pose", 10, apriltag_callback);

    // Spin in a separate thread
    std::thread spin_thread([&](){
        while (running) {
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });

    // Wait for user input
    std::cout << "Press Enter to capture the latest AprilTag pose..." << std::endl;
    std::cin.get();

    // Stop spinning
    running = false;
    spin_thread.join();

    // Destroy subscription
    subscription.reset();

    // Creating joint state subscriber
    auto joint_state_sub = node->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, encoder_callback);

    // Spin in a separate thread
    std::thread spin_thread2([&](){
        while (running2) {
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });

    // initializing trajectory publisher
    auto publisher = node->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/rby1_base_controller/cmd_vel", 10);

    // initializing message
    auto msg = geometry_msgs::msg::TwistStamped();
    msg.header.stamp = node->now(); // Use current time
    msg.header.frame_id = "base";
    msg.twist.linear.x = 0.0;
    msg.twist.linear.y = 0.0;
    msg.twist.linear.z = 0.0;
    msg.twist.angular.x = 0.0;
    msg.twist.angular.y = 0.0;
    msg.twist.angular.z = 0.0;

    // script start
    auto robot = rb::Robot<y1_model::A>::Create("192.168.30.1:50051"); // RPC
    //auto robot = rb::Robot<y1_model::A>::Create("192.168.1.6:50051"); // Eric's Mujoco IP
    robot->Connect();
    if (!robot->IsPowerOn(kAll)){
        robot->PowerOn(kAll);
        robot->ServoOn(kAll);
        robot->EnableControlManager();
    }

    upc::InitializeDevice(upc::kGripperDeviceName);
    upc::InitializeDevice(upc::kMasterArmDeviceName);

    if (robot->IsPowerOn("48v")) {
        robot->SetToolFlangeOutputVoltage("right", 12);
        robot->SetToolFlangeOutputVoltage("left", 12);
        std::cout << "Attempting to 12V power on for gripper." << std::endl;

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    const char* devicename_gripper = "/dev/rby1_gripper";
    //const char* devicename_gripper = "/dev/ttyUSB0";

    dynamixel::PortHandler* portHandler_gripper = dynamixel::PortHandler::getPortHandler(devicename_gripper);
    dynamixel::PacketHandler* packetHandler_gripper = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (!portHandler_gripper->openPort()) {
        std::cerr << "Failed to open the port!" << std::endl;
        return 1;
    }

    if (!portHandler_gripper->setBaudRate(BAUDRATE)) {
        std::cerr << "Failed to change the baudrate!" << std::endl;
        return 1;
    }

    std::vector<int> activeIDs_gripper;

    for (int id = 0; id < 2; ++id) {
        uint8_t dxl_error = 0;
        int dxl_comm_result = packetHandler_gripper->ping(portHandler_gripper, id, &dxl_error);
        if (dxl_comm_result == COMM_SUCCESS) {
            std::cout << "Dynamixel ID " << id << " is active." << std::endl;
            activeIDs_gripper.push_back(id);            
        } else if (dxl_error != 0) {
            std::cout << "Servo error" << std::endl;
        }
        else {
            std::cerr << "Dynamixel ID " << id << " is not active." << std::endl;
        }
    }

    // gripper initialization
    initialization_for_gripper(portHandler_gripper, packetHandler_gripper, activeIDs_gripper);

    // send gripper command
    gripper_position_command(portHandler_gripper, packetHandler_gripper, activeIDs_gripper, 0.1, 0.1);

    // send arm command
    // Create the MoveGroupInterface for your planning group
    using moveit::planning_interface::MoveGroupInterface;
    //auto dual_arm = MoveGroupInterface(node, "rby1_dualarm");
    auto left_arm = MoveGroupInterface(node, "rby1_left_arm");
    auto right_arm = MoveGroupInterface(node, "rby1_right_arm");
    std::cout << "End effector link left: " << left_arm.getEndEffectorLink() << std::endl;
    std::cout << "End effector link right: " << right_arm.getEndEffectorLink() << std::endl;
    //std::cout << "End effector: " << dual_arm.getEndEffectorLink() << std::endl;

    // Set frame references
    //left_arm.setPoseReferenceFrame("link_head_2");
    //right_arm.setPoseReferenceFrame("link_head_2");
    left_arm.setPoseReferenceFrame("camera_right");
    right_arm.setPoseReferenceFrame("camera_right");

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
    joint_positions[5] = 53 * D2R;
    joint_positions[6] = 0 * D2R;
    right_arm.setJointValueTarget(joint_positions);
    moveit::planning_interface::MoveGroupInterface::Plan joint_plan;
    if(right_arm.plan(joint_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        right_arm.execute(joint_plan);
    }

    // Define the target poses
    //geometry_msgs::msg::Pose target_pose_left;
    geometry_msgs::msg::Pose target_pose_right;
    // left arm
    //target_pose_left.orientation.w = 1.0;
    //target_pose_left.position.x = 0.294;
    //target_pose_left.position.y = 0.219;
    //target_pose_left.position.z = -0.375;

    // right arm orientation
    Eigen::AngleAxisd roll(-M_PI/2, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch(0, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw(M_PI/2, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = yaw * pitch * roll;
    target_pose_right.orientation.w = q.w();
    target_pose_right.orientation.x = q.x();
    target_pose_right.orientation.y = q.y();
    target_pose_right.orientation.z = q.z();

    // set fixed end effector orientation constraint
    /*
    geometry_msgs::msg::Quaternion quat_msg;
    quat_msg.w = q.w();
    quat_msg.x = q.x();
    quat_msg.y = q.y();
    quat_msg.z = q.z();
    moveit_msgs::msg::OrientationConstraint ocm;
    ocm.link_name = "ee_right";
    ocm.header.frame_id = "camera_right";
    ocm.orientation = quat_msg;
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.1;
    ocm.weight = 1.0;
    moveit_msgs::msg::Constraints constraints;
    constraints.orientation_constraints.push_back(ocm);
    right_arm.setPathConstraints(constraints);
    */

    // first send to position 15 cm behind, 15 cm above, and 15 cm to the right of object
    target_pose_right.position.x = object_position_x - 0.15;
    target_pose_right.position.y = object_position_y - 0.15;
    target_pose_right.position.z = object_position_z + 0.15;
    //left_arm.setPoseTarget(target_pose_left,"ee_left");
    right_arm.setPoseTarget(target_pose_right,"ee_right");
    // Plan to the target poses
    //moveit::planning_interface::MoveGroupInterface::Plan left_plan;
    moveit::planning_interface::MoveGroupInterface::Plan right_plan;
    //bool success_left = (left_arm.plan(left_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    bool success = (right_arm.plan(right_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    // Execute the plan if successful
    std::this_thread::sleep_for(std::chrono::seconds(1));
    if (success) {
        right_arm.execute(right_plan);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        //left_arm.execute(left_plan);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        RCLCPP_INFO(logger, "Motion executed!");
    } else {
        RCLCPP_ERROR(logger, "Planning failed!");
    }

    // then move over object (send to position 15 cm behind and 15 cm above object)
    target_pose_right.position.x = object_position_x - 0.15;
    target_pose_right.position.y = object_position_y;
    target_pose_right.position.z = object_position_z + 0.15;
    right_arm.setPoseTarget(target_pose_right,"ee_right");
    success = (right_arm.plan(right_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    if (success) {
        right_arm.execute(right_plan);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::this_thread::sleep_for(std::chrono::seconds(1));
        RCLCPP_INFO(logger, "Motion executed!");
    } else {
        RCLCPP_ERROR(logger, "Planning failed!");
    }


    // then send to grasping position (7.3 cm offset between ee_right and the palm)
    target_pose_right.position.x = object_position_x - 0.073;
    target_pose_right.position.y = object_position_y;
    target_pose_right.position.z = object_position_z + 0.01;
    right_arm.setPoseTarget(target_pose_right,"ee_right");
    success = (right_arm.plan(right_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    if (success) {
        right_arm.execute(right_plan);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::this_thread::sleep_for(std::chrono::seconds(1));
        RCLCPP_INFO(logger, "Motion executed!");
    } else {
        RCLCPP_ERROR(logger, "Planning failed!");
    }

    // gripper close on object
    //std::cout << "Press Enter to close gripper..." << std::endl;
    //std::cin.get();  // Waits for the user to press Enter
    gripper_position_command(portHandler_gripper, packetHandler_gripper, activeIDs_gripper, 0.1, 0.6);

    // then move object up
    target_pose_right.position.x = object_position_x - 0.073;
    target_pose_right.position.y = object_position_y;
    target_pose_right.position.z = object_position_z + 0.15;
    right_arm.setPoseTarget(target_pose_right,"ee_right");
    success = (right_arm.plan(right_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    if (success) {
        right_arm.execute(right_plan);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::this_thread::sleep_for(std::chrono::seconds(1));
        RCLCPP_INFO(logger, "Motion executed!");
    } else {
        RCLCPP_ERROR(logger, "Planning failed!");
    }

    // move base back and turn 180 degrees and move forward
    move_linear(0.3, -0.2, publisher, msg, node);
    move_angular(M_PI/2, 0.3, publisher, msg, node);
    move_linear(0.3, 0.2, publisher, msg, node);

    //move to drop location 30 cm to the right and 15 cm above
    target_pose_right.position.x = object_position_x - 0.073;
    target_pose_right.position.y = object_position_y - 0.30;
    target_pose_right.position.z = object_position_z + 0.15;
    right_arm.setPoseTarget(target_pose_right,"ee_right");
    success = (right_arm.plan(right_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    if (success) {
        right_arm.execute(right_plan);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::this_thread::sleep_for(std::chrono::seconds(1));
        RCLCPP_INFO(logger, "Motion executed!");
    } else {
        RCLCPP_ERROR(logger, "Planning failed!");
    }

    //std::cout << "Press Enter to open gripper..." << std::endl;
    //std::cin.get();  // Waits for the user to press Enter
    gripper_position_command(portHandler_gripper, packetHandler_gripper, activeIDs_gripper, 0.1, 0.1);

    // Stop spinning
    running2 = false;
    spin_thread2.join();

    //Shutdown
    rclcpp::shutdown();
    return 0;
}