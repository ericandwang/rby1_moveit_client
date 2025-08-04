#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include<tf2_ros/transform_listener.h>
#include<tf2_ros/buffer.h>
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"
#include "rby1-sdk/upc/device.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

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

using namespace rb;
const std::string kAll = ".*";
std::vector<Eigen::Matrix<double, 2, 1>> q_min_max_vector;

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
    auto robot = rb::Robot<y1_model::A>::Create("192.168.30.1:50051");
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

    std::cout << "Press Enter to close gripper..." << std::endl;
    std::cin.get();  // Waits for the user to press Enter

    gripper_position_command(portHandler_gripper, packetHandler_gripper, activeIDs_gripper, 0.1, 0.6);

    std::cout << "Press Enter to bring hand in front of camera..." << std::endl;
    std::cin.get();  // Waits for the user to press Enter

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

    std::cout << "Press Enter to rotate wrist..." << std::endl;
    std::cin.get();  // Waits for the user to press Enter

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

    std::cout << "Press Enter to reset positions..." << std::endl;
    std::cin.get();  // Waits for the user to press Enter
    
    // reset head
    joint_positions_head[0] = 0;
    joint_positions_head[1] = 0;
    head.setJointValueTarget(joint_positions_head);
    if(head.plan(joint_plan_head) == moveit::core::MoveItErrorCode::SUCCESS) {
        head.execute(joint_plan_head);
    }

    // reset right arm
    joint_positions[0] = 0;
    joint_positions[1] = 0;
    joint_positions[2] = 0;
    joint_positions[3] = 0;
    joint_positions[4] = 0;
    joint_positions[5] = 0;
    joint_positions[6] = 0;
    right_arm.setJointValueTarget(joint_positions);
    if(right_arm.plan(joint_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        right_arm.execute(joint_plan);
    }

    std::cout << "Press Enter to open gripper..." << std::endl;
    std::cin.get();  // Waits for the user to press Enter

    gripper_position_command(portHandler_gripper, packetHandler_gripper, activeIDs_gripper, 0.1, 0.1);

    // Stop spinning
    running = false;
    spin_thread.join();

    //Shutdown
    rclcpp::shutdown();
    return 0;
}