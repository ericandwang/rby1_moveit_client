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
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"
#include "rby1-sdk/upc/device.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <thread>
#include <atomic>
#include <moveit_msgs/msg/orientation_constraint.hpp>

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

#define M_PI 3.14159265358979323846
#define D2R 0.017453288888888
#define R2D 57.29579143313326

using namespace rb;
using namespace std::chrono_literals;
using moveit::planning_interface::MoveGroupInterface;
const std::string kAll = ".*";
std::vector<Eigen::Matrix<double, 2, 1>> q_min_max_vector;
double object_position_x = 0.47;
double object_position_y = -0.116;
double object_position_z = -0.375;
std::atomic<bool> running0{true};
std::atomic<bool> running1{true};
std::atomic<bool> running2{true};
std::atomic<bool> running3{true};

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

struct GripperHandles {
    dynamixel::PortHandler* portHandler;
    dynamixel::PacketHandler* packetHandler;
    std::vector<int> activeIDs;
    bool ok = true;

    // helper method to command gripper position
    void command_position(double right_goal, double left_goal) const {
        gripper_position_command(portHandler, packetHandler, activeIDs, right_goal, left_goal);
    }
};

GripperHandles initialize_gripper() {
    GripperHandles handles;
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

    handles.portHandler = dynamixel::PortHandler::getPortHandler(devicename_gripper);
    handles.packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (!handles.portHandler->openPort()) {
        std::cerr << "Failed to open the port!" << std::endl;
        handles.ok = false;
        return handles;
    }

    if (!handles.portHandler->setBaudRate(BAUDRATE)) {
        std::cerr << "Failed to change the baudrate!" << std::endl;
        handles.ok = false;
        return handles;
    }

    for (int id = 0; id < 2; ++id) {
        uint8_t dxl_error = 0;
        int dxl_comm_result = handles.packetHandler->ping(handles.portHandler, id, &dxl_error);
        if (dxl_comm_result == COMM_SUCCESS) {
            std::cout << "Dynamixel ID " << id << " is active." << std::endl;
            handles.activeIDs.push_back(id);            
        } else if (dxl_error != 0) {
            std::cout << "Servo error" << std::endl;
        }
        else {
            std::cerr << "Dynamixel ID " << id << " is not active." << std::endl;
        }
    }

    // gripper initialization
    initialization_for_gripper(handles.portHandler, handles.packetHandler, handles.activeIDs);

    // send gripper command
    handles.command_position(0.1, 0.1);
    return handles;
}

void initialize_arm(const std::shared_ptr<rclcpp::Node>& node) {
    // Create the MoveGroupInterface for your planning group
    auto right_arm = MoveGroupInterface(node, "rby1_right_arm");
    std::cout << "End effector link right: " << right_arm.getEndEffectorLink() << std::endl;

    // Set frame references
    right_arm.setPoseReferenceFrame("link_head_2");
    //right_arm.setPoseReferenceFrame("camera_right");

    // Set arm to initial manipulation position
    std::vector<double> joint_positions(7);
    joint_positions[0] = -29 * D2R;
    joint_positions[1] = -77 * D2R; //-45 * D2R;
    joint_positions[2] = 22 * D2R; //36 * D2R;
    joint_positions[3] = -96 * D2R;
    joint_positions[4] = 21 * D2R;
    joint_positions[5] = 53 * D2R;
    joint_positions[6] = 0 * D2R;
    right_arm.setJointValueTarget(joint_positions);
    moveit::planning_interface::MoveGroupInterface::Plan joint_plan;
    if(right_arm.plan(joint_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        right_arm.execute(joint_plan);
    }
}

void grab_object(const std::shared_ptr<rclcpp::Node>& node, const rclcpp::Logger & logger, const GripperHandles& gripper) {
    // Offset from link_head_2 to camera_right
    const double dx = -0.01;
    const double dy = -0.060;
    const double dz = 0.015;

    // Create the MoveGroupInterface for your planning group
    auto right_arm = MoveGroupInterface(node, "rby1_right_arm");
    std::cout << "End effector link right: " << right_arm.getEndEffectorLink() << std::endl;

    // Set frame references
    right_arm.setPoseReferenceFrame("link_head_2");
    //right_arm.setPoseReferenceFrame("camera_right");

    // Set arm to initial manipulation position
    //std::vector<double> joint_positions(7);
    //joint_positions[0] = -29 * D2R;
    //joint_positions[1] = -45 * D2R;
    //joint_positions[2] = 36 * D2R;
    //joint_positions[3] = -96 * D2R;
    //joint_positions[4] = 21 * D2R;
    //joint_positions[5] = 53 * D2R;
    //joint_positions[6] = 0 * D2R;
    //right_arm.setJointValueTarget(joint_positions);
    //moveit::planning_interface::MoveGroupInterface::Plan joint_plan;
    //if(right_arm.plan(joint_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
    //    right_arm.execute(joint_plan);
    //}

    // Define the target poses
    geometry_msgs::msg::Pose target_pose_right;

    // right arm orientation
    Eigen::AngleAxisd roll(-M_PI/2, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch(0, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw(M_PI/2, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = yaw * pitch * roll;
    target_pose_right.orientation.w = q.w();
    target_pose_right.orientation.x = q.x();
    target_pose_right.orientation.y = q.y();
    target_pose_right.orientation.z = q.z();

    // first send to position 15 cm behind, 15 cm above, and 15 cm to the right of object
    target_pose_right.position.x = object_position_x + dx - 0.15;
    target_pose_right.position.y = object_position_y + dy - 0.15;
    target_pose_right.position.z = object_position_z + dz + 0.15;
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
    target_pose_right.position.x = object_position_x + dx - 0.15;
    target_pose_right.position.y = object_position_y + dy;
    target_pose_right.position.z = object_position_z + dz + 0.15;
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
    target_pose_right.position.x = object_position_x + dx - 0.073;
    target_pose_right.position.y = object_position_y + dy;
    target_pose_right.position.z = object_position_z + dz + 0.01;
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
    gripper_position_command(gripper.portHandler, gripper.packetHandler, gripper.activeIDs, 0.1, 0.6);

    // then move object up
    target_pose_right.position.x = object_position_x + dx - 0.073;
    target_pose_right.position.y = object_position_y + dy;
    target_pose_right.position.z = object_position_z + dz + 0.15;
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
}

class TagGoalNav2Node : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav2 = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    TagGoalNav2Node(double offset_x, double offset_y)
    : Node("tag_goal_nav2_node"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_),
      offset_x_(offset_x),
      offset_y_(offset_y),
      keepSendingGoal(true)
    {
        // Setup Nav2 action client
        nav2_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose_apriltag", 10);

        // Wait for Nav2 action server
        while (!nav2_client_->wait_for_action_server(2s)) {
            RCLCPP_INFO(this->get_logger(), "Waiting for Nav2 action server...");
        }

        // Timer to periodically send goal
        //timer_ = this->create_wall_timer(
        //    2s, std::bind(&TagGoalNav2Node::send_tag_goal, this));
        //Send goal multiple times
        for (int i = 0; i < 3; i++) {
            send_tag_goal();
            rclcpp::sleep_for(50ms);   // small delay between sends
        }
        //send_tag_goal();
    }

private:
    void send_tag_goal()
    {
        if (keepSendingGoal) {
            // Lookup transform from tag36h11:3 to world
            geometry_msgs::msg::TransformStamped transformStamped;
            try {
                transformStamped = tf_buffer_.lookupTransform(
                    "map",         // target frame
                    "tag36h11:3_map",    // source frame
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
            //double offset_x = -0.2; // -20 cm
            goal_pose.pose.position.x += offset_x_ * std::cos(yaw);
            goal_pose.pose.position.y += offset_x_ * std::sin(yaw);
            //double offset_y = 0.1;
            goal_pose.pose.position.x += -offset_y_ * std::sin(yaw);
            goal_pose.pose.position.y +=  offset_y_ * std::cos(yaw);

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
                        keepSendingGoal = false;
                        //timer_->cancel();
                    }
                };
            send_goal_options.result_callback =
                [this](const GoalHandleNav2::WrappedResult & result) {
                    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                        RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
                        running1 = false;
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Goal failed or was aborted");
                    }
                };

            nav2_client_->async_send_goal(goal_msg, send_goal_options);
            RCLCPP_INFO(this->get_logger(), "Sent Nav2 goal at (%.2f, %.2f)", 
                        goal_pose.pose.position.x, goal_pose.pose.position.y);
        }
    }

    rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    double offset_x_;
    double offset_y_;
    std::atomic<bool> keepSendingGoal{true};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // initialize gripper and arm
    auto gripper = initialize_gripper();
    if (!gripper.ok) {
        std::cerr << "[Error] Gripper initialization failed. Exiting." << std::endl;
        rclcpp::shutdown();
        return 1;
    }
    auto const node_grabber = std::make_shared<rclcpp::Node>(
        "grabber",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );
    auto const logger = rclcpp::get_logger("grabber");
    std::thread spin_thread2([&](){
        while (running2) {
            rclcpp::spin_some(node_grabber);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });
    initialize_arm(node_grabber);

    // create temporary local apriltag node
    auto const node_apriltag_local = std::make_shared<rclcpp::Node>(
        "apriltag_local",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );
    auto subscription0 = node_apriltag_local->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/apriltag/pose", 10, apriltag_callback);
    // Spin in a separate thread
    std::thread spin_thread0([&](){
        while (running0) {
            rclcpp::spin_some(node_apriltag_local);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });

    std::cout << "Press Enter to navigate to docking pose..." << std::endl;
    std::cin.get();
    
    // create apriltag nav2 docking node
    double offset_x1 = -1.5; double offset_y1 = 0.13;
    auto const node_apriltag_nav2_docking = std::make_shared<TagGoalNav2Node>(offset_x1, offset_y1);
    // Spin in a separate thread
    std::thread spin_thread1([&](){
        while (running1) {
            rclcpp::spin_some(node_apriltag_nav2_docking);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });

    std::cout << "Press Enter to navigate to AprilTag pose..." << std::endl;
    std::cin.get();
    running1 = false;
    spin_thread1.join();

    // create apriltag nav2 node
    double offset_x2 = -0.45; double offset_y2 = 0.13; // -0.23, 0.13
    auto const node_apriltag_nav2 = std::make_shared<TagGoalNav2Node>(offset_x2, offset_y2);
    // Spin in a separate thread
    std::thread spin_thread3([&](){
        while (running3) {
            rclcpp::spin_some(node_apriltag_nav2);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });

    std::cout << "When navigation complete, press Enter to capture the latest AprilTag pose and grasp..." << std::endl;
    std::cin.get();
    running3 = false;
    spin_thread3.join();
    running0 = false;
    spin_thread0.join();
    subscription0.reset();
   
    grab_object(node_grabber, logger, gripper);
    running2 = false;
    spin_thread2.join();
    rclcpp::shutdown();

    //std::cout << "Press Enter to close gripper..." << std::endl;
    //std::cin.get();  // Waits for the user to press Enter
    //gripper.command_position(0.1, 0.6);

    std::cout << "Press Enter to open gripper..." << std::endl;
    std::cin.get();  // Waits for the user to press Enter
    gripper.command_position(0.1, 0.1);

    std::cout << "Reached end of script." << std::endl;
    return 0;
}
