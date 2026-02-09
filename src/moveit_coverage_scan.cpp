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
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include <random>
#include <set>
#include <limits>
#include <cmath>
#include <fstream>
#include <mutex>
#include <array>
#include <sstream>

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
std::atomic<bool> is_sleeping{false};
rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
std::vector<double> latest_joint_positions_(24, 0.0);

// Right-arm joint trajectory logging (indices 8-14 = right_arm_0..right_arm_6)
constexpr int RIGHT_ARM_START_IDX = 8;
constexpr int RIGHT_ARM_JOINTS = 7;
constexpr double JOINT_CHANGE_THRESHOLD = 0.001;  // rad

std::vector<std::array<double, RIGHT_ARM_JOINTS>> right_arm_trajectory_buffer_;
std::array<double, RIGHT_ARM_JOINTS> last_saved_right_arm_;
std::atomic<bool> recording_right_arm_{false};
std::atomic<bool> right_arm_first_sample_{true};
std::mutex right_arm_buffer_mutex_;

void encoder_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (msg->position.size() < 15) {
        return;  // Need at least indices 0-14 for right arm
    }
    for (size_t i = 0; i < 24 && i < msg->position.size(); ++i) {
        latest_joint_positions_[i] = msg->position[i];
    }

    if (!recording_right_arm_) {
        return;
    }

    std::array<double, RIGHT_ARM_JOINTS> current;
    for (int i = 0; i < RIGHT_ARM_JOINTS; ++i) {
        current[i] = msg->position[RIGHT_ARM_START_IDX + i];
    }

    bool should_append = false;
    if (right_arm_first_sample_) {
        should_append = true;
        right_arm_first_sample_ = false;
    } else {
        for (int i = 0; i < RIGHT_ARM_JOINTS; ++i) {
            if (std::abs(current[i] - last_saved_right_arm_[i]) > JOINT_CHANGE_THRESHOLD) {
                should_append = true;
                break;
            }
        }
    }

    if (should_append) {
        std::lock_guard<std::mutex> lock(right_arm_buffer_mutex_);
        right_arm_trajectory_buffer_.push_back(current);
        last_saved_right_arm_ = current;
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

// ============================================================================
// Coverage Scan Helper Structures
// ============================================================================

// A valid joint configuration and which coverage point it serves
struct IKConfig {
    std::vector<double> joint_values;
    int coverage_point_idx;
};

// Graph node for coverage path planning
struct ConfigNode {
    std::vector<double> joints;
    int coverage_idx;
    std::vector<int> neighbors;
};

// ============================================================================
// Coverage Scan Helper Functions
// ============================================================================

// Generate N uniformly distributed points on a unit sphere using Fibonacci spiral
std::vector<Eigen::Vector3d> generateFibonacciSphere(int n) {
    std::vector<Eigen::Vector3d> points;
    double golden_ratio = (1.0 + std::sqrt(5.0)) / 2.0;
    for (int i = 0; i < n; i++) {
        double theta = 2.0 * M_PI * i / golden_ratio;
        double phi = std::acos(1.0 - 2.0 * (i + 0.5) / n);
        double x = std::sin(phi) * std::cos(theta);
        double y = std::sin(phi) * std::sin(theta);
        double z = std::cos(phi);
        points.emplace_back(x, y, z);
    }
    return points;
}

// Convert a Fibonacci sphere point to a target gripper orientation.
// Rotates the reference direction (0,0,1) onto the sphere point,
// then applies that rotation to the current gripper orientation.
Eigen::Quaterniond spherePointToOrientation(const Eigen::Vector3d& point,
                                            const Eigen::Quaterniond& current_orientation) {
    Eigen::Vector3d ref_dir(0.0, 0.0, 1.0);
    Eigen::Quaterniond rot = Eigen::Quaterniond::FromTwoVectors(ref_dir, point);
    return (rot * current_orientation).normalized();
}

// Write right-arm trajectory buffer to .npy file (NumPy format v1.0).
// Returns number of samples written, or 0 on failure.
size_t writeRightArmTrajectoryNpy(const std::string& filepath) {
    std::lock_guard<std::mutex> lock(right_arm_buffer_mutex_);
    const size_t N = right_arm_trajectory_buffer_.size();
    if (N == 0) {
        return 0;
    }

    std::ofstream out(filepath, std::ios::binary);
    if (!out) {
        return 0;
    }

    // NumPy .npy format v1.0
    const char magic[] = "\x93NUMPY";
    out.write(magic, 6);
    out.put(1);  // version major
    out.put(0);  // version minor

    std::ostringstream header_ss;
    header_ss << "{'descr': '<f8', 'fortran_order': False, 'shape': (" << N << ", " << RIGHT_ARM_JOINTS << "), }";
    std::string header_str = header_ss.str() + "\n";
    size_t header_len = header_str.size();
    // Pad to make (6 + 2 + 2 + header_len) % 64 == 0
    size_t pad = (64 - (10 + header_len) % 64) % 64;
    header_len += pad;
    header_str.append(pad, ' ');

    uint16_t header_len_le = static_cast<uint16_t>(header_len);
    out.write(reinterpret_cast<const char*>(&header_len_le), 2);
    out.write(header_str.data(), header_len);

    for (const auto& row : right_arm_trajectory_buffer_) {
        out.write(reinterpret_cast<const char*>(row.data()), RIGHT_ARM_JOINTS * sizeof(double));
    }

    return out.good() ? N : 0;
}

// Euclidean distance between two joint configurations
double jointSpaceDistance(const std::vector<double>& a, const std::vector<double>& b) {
    double sum = 0.0;
    for (size_t i = 0; i < a.size() && i < b.size(); i++) {
        double diff = a[i] - b[i];
        sum += diff * diff;
    }
    return std::sqrt(sum);
}

// Sample IK solutions for a target end-effector pose using random seed states.
// Seeds are drawn from the middle 60% of each joint's range.
std::vector<IKConfig> sampleIKConfigs(
    const moveit::core::RobotStatePtr& reference_state,
    const moveit::core::JointModelGroup* jmg,
    const Eigen::Isometry3d& target_pose,
    int coverage_idx,
    int num_seeds = 8)
{
    std::vector<IKConfig> configs;
    std::mt19937 rng(42 + coverage_idx * 137);

    const auto& active_joints = jmg->getActiveJointModels();

    for (int s = 0; s < num_seeds; s++) {
        moveit::core::RobotState seed_state(*reference_state);

        // First seed uses current state; rest use random seeds in middle 60% of range
        if (s > 0) {
            for (const auto* joint : active_joints) {
                const auto& bounds_vec = joint->getVariableBounds();
                for (size_t v = 0; v < bounds_vec.size(); v++) {
                    if (bounds_vec[v].position_bounded_) {
                        double range = bounds_vec[v].max_position_ - bounds_vec[v].min_position_;
                        double low = bounds_vec[v].min_position_ + 0.2 * range;
                        double high = bounds_vec[v].max_position_ - 0.2 * range;
                        std::uniform_real_distribution<double> dist(low, high);
                        double val = dist(rng);
                        seed_state.setJointPositions(joint, &val);
                    }
                }
            }
        }

        // Attempt IK from this seed (100ms timeout)
        if (seed_state.setFromIK(jmg, target_pose, 0.1)) {
            IKConfig config;
            seed_state.copyJointGroupPositions(jmg, config.joint_values);
            config.coverage_point_idx = coverage_idx;
            configs.push_back(config);
        }
    }

    return configs;
}

// Build a graph connecting joint configurations within max_distance of each other
std::vector<ConfigNode> buildConfigGraph(const std::vector<IKConfig>& configs,
                                         double max_distance = 2.0) {
    std::vector<ConfigNode> graph;
    graph.reserve(configs.size());

    for (const auto& c : configs) {
        ConfigNode node;
        node.joints = c.joint_values;
        node.coverage_idx = c.coverage_point_idx;
        graph.push_back(node);
    }

    // Connect nodes within max_distance in joint space
    for (size_t i = 0; i < graph.size(); i++) {
        for (size_t j = i + 1; j < graph.size(); j++) {
            if (jointSpaceDistance(graph[i].joints, graph[j].joints) < max_distance) {
                graph[i].neighbors.push_back(j);
                graph[j].neighbors.push_back(i);
            }
        }
    }

    return graph;
}

// Greedy nearest-neighbor: visit the closest unvisited coverage point at each step
std::vector<int> findCoveragePath(const std::vector<ConfigNode>& graph,
                                  const std::vector<double>& start_joints,
                                  int num_coverage_points) {
    std::vector<int> path;
    std::set<int> visited_coverage;
    std::vector<double> current = start_joints;

    while ((int)visited_coverage.size() < num_coverage_points) {
        double best_dist = std::numeric_limits<double>::max();
        int best_node = -1;

        // Find nearest config that covers an unvisited point
        for (size_t i = 0; i < graph.size(); i++) {
            if (visited_coverage.count(graph[i].coverage_idx)) continue;
            double d = jointSpaceDistance(current, graph[i].joints);
            if (d < best_dist) {
                best_dist = d;
                best_node = i;
            }
        }

        // Stop if nothing reachable (safety: 6 rad max jump)
        if (best_node == -1 || best_dist > 6.0) break;

        path.push_back(best_node);
        visited_coverage.insert(graph[best_node].coverage_idx);
        current = graph[best_node].joints;
    }

    return path;
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char * argv[])
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create a ROS 2 node
    auto const node = std::make_shared<rclcpp::Node>(
        "moveit_coverage_scan",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    auto const logger = rclcpp::get_logger("moveit_coverage_scan");

    //node->declare_parameter_if_not_declared("joint_trajectory_output_path",
    //    rclcpp::ParameterValue("joint_trajectory.npy"));

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

    auto flag_pub = node->create_publisher<sensor_msgs::msg::Temperature>("waiting_flag", 10);
    // Publisher thread (publishes at 10Hz)
    std::thread([&](){
        rclcpp::Rate rate(10); // 10Hz
        sensor_msgs::msg::Temperature flag_msg;
        while(rclcpp::ok()){
            flag_msg.header.stamp = node->now();
            flag_msg.temperature = is_sleeping ? 1.0 : 0.0;
            flag_pub->publish(flag_msg);
            rate.sleep();
        }
    }).detach();

    // Create the MoveGroupInterface for your planning group
    using moveit::planning_interface::MoveGroupInterface;
    auto left_arm = MoveGroupInterface(node, "rby1_left_arm");
    auto right_arm = MoveGroupInterface(node, "rby1_right_arm");
    std::cout << "End effector link left: " << left_arm.getEndEffectorLink() << std::endl;
    std::cout << "End effector link right: " << right_arm.getEndEffectorLink() << std::endl;

    // Set frame references
    left_arm.setPoseReferenceFrame("link_head_2");
    right_arm.setPoseReferenceFrame("link_head_2");

    // Set arm to initial manipulation position
    std::vector<double> joint_positions(7);
    joint_positions[0] = -0 * D2R;
    joint_positions[1] = -90 * D2R;
    joint_positions[2] = 0 * D2R;
    joint_positions[3] = -110 * D2R;
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
    joint_positions_head[0] = yaw;
    joint_positions_head[1] = pitch;
    head.setJointValueTarget(joint_positions_head);
    moveit::planning_interface::MoveGroupInterface::Plan joint_plan_head;
    if(head.plan(joint_plan_head) == moveit::core::MoveItErrorCode::SUCCESS) {
        head.execute(joint_plan_head);
    }

    // ========================================================================
    // COVERAGE SCAN: Systematically rotate object in front of fixed camera
    // ========================================================================

    std::cout << "Press Enter to start coverage scan..." << std::endl;
    std::cin.get();

    // Start recording right-arm joint trajectory (only when joints change)
    {
        std::lock_guard<std::mutex> lock(right_arm_buffer_mutex_);
        right_arm_trajectory_buffer_.clear();
    }
    right_arm_first_sample_ = true;
    recording_right_arm_ = true;

    const int NUM_SPHERE_POINTS = 14;
    const int NUM_IK_SEEDS = 8;

    // Get current robot state and end effector pose for IK
    auto current_state = right_arm.getCurrentState(10.0);
    if (!current_state) {
        RCLCPP_ERROR(logger, "Failed to get current robot state. Skipping coverage scan.");
    }

    if (current_state) {
        current_state->update();
        auto robot_model = right_arm.getRobotModel();
        const auto* jmg = robot_model->getJointModelGroup("rby1_right_arm");
        std::string ee_link = right_arm.getEndEffectorLink();

        // Current end effector pose (position + orientation in model frame)
        const Eigen::Isometry3d& current_ee_pose =
            current_state->getGlobalLinkTransform(ee_link);
        Eigen::Quaterniond current_quat(current_ee_pose.rotation());

        // Current joint values as starting point for path search
        std::vector<double> start_joints;
        current_state->copyJointGroupPositions(jmg, start_joints);

        std::cout << "Current EE position: ["
                  << current_ee_pose.translation().x() << ", "
                  << current_ee_pose.translation().y() << ", "
                  << current_ee_pose.translation().z() << "]" << std::endl;

        // --- Step 1: Generate target orientations from Fibonacci sphere ---
        auto sphere_points = generateFibonacciSphere(NUM_SPHERE_POINTS);
        std::cout << "Generated " << sphere_points.size()
                  << " Fibonacci sphere points" << std::endl;

        // --- Step 2: Sample IK configurations for each orientation ---
        std::vector<IKConfig> all_configs;
        int orientations_with_ik = 0;

        for (int i = 0; i < (int)sphere_points.size(); i++) {
            // Target orientation: rotate current orientation by sphere point
            Eigen::Quaterniond target_quat =
                spherePointToOrientation(sphere_points[i], current_quat);

            // Target pose: same position, new orientation
            Eigen::Isometry3d target_pose = Eigen::Isometry3d::Identity();
            target_pose.translation() = current_ee_pose.translation();
            target_pose.linear() = target_quat.toRotationMatrix();

            // Try multiple random IK seeds
            auto configs = sampleIKConfigs(
                current_state, jmg, target_pose, i, NUM_IK_SEEDS);

            if (!configs.empty()) {
                orientations_with_ik++;
                std::cout << "  Orientation " << i << ": "
                          << configs.size() << " IK solutions" << std::endl;
            }
            all_configs.insert(all_configs.end(), configs.begin(), configs.end());
        }

        std::cout << "IK sampling complete: " << all_configs.size()
                  << " total configs for " << orientations_with_ik << "/"
                  << NUM_SPHERE_POINTS << " orientations" << std::endl;

        if (all_configs.empty()) {
            std::cout << "WARNING: No IK solutions found. Skipping coverage scan."
                      << std::endl;
        } else {
            // --- Step 3: Build configuration graph ---
            auto graph = buildConfigGraph(all_configs, 2.0);
            std::cout << "Config graph: " << graph.size() << " nodes" << std::endl;

            // --- Step 4: Find coverage path (greedy nearest-neighbor) ---
            auto path = findCoveragePath(graph, start_joints, NUM_SPHERE_POINTS);
            std::cout << "Coverage path: " << path.size()
                      << " viewpoints planned" << std::endl;

            // --- Step 5: Execute coverage path in joint space ---
            // Scan initial position first
            is_sleeping = true;
            rclcpp::sleep_for(std::chrono::milliseconds(1000));
            is_sleeping = false;

            for (int step = 0; step < (int)path.size(); step++) {
                int node_idx = path[step];
                int cov_idx = graph[node_idx].coverage_idx;
                std::cout << "Viewpoint " << (step + 1) << "/"
                          << path.size() << " (coverage point "
                          << cov_idx << ")" << std::endl;

                bool executed_viewpoint = false;

                // First, try the primary IK config chosen for this viewpoint
                right_arm.setJointValueTarget(graph[node_idx].joints);
                if (right_arm.plan(joint_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                    right_arm.execute(joint_plan);
                    executed_viewpoint = true;
                } else {
                    std::cout << "  Planning failed for primary IK, trying alternative IK solutions for coverage point "
                              << cov_idx << std::endl;

                    // Fallback: try other IK configs that correspond to the same coverage point
                    for (size_t i = 0; i < graph.size(); ++i) {
                        if ((int)i == node_idx)
                            continue;  // already tried this one
                        if (graph[i].coverage_idx != cov_idx)
                            continue;  // different coverage orientation

                        right_arm.setJointValueTarget(graph[i].joints);
                        if (right_arm.plan(joint_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                            std::cout << "  Using alternative IK solution index " << i
                                      << " for coverage point " << cov_idx << std::endl;
                            right_arm.execute(joint_plan);
                            executed_viewpoint = true;
                            break;
                        }
                    }

                    if (!executed_viewpoint) {
                        std::cout << "  Planning failed for all IK solutions of coverage point "
                                  << cov_idx << ", skipping viewpoint " << (step + 1) << std::endl;
                        continue;
                    }
                }

                // Pause for camera capture
                is_sleeping = true;
                rclcpp::sleep_for(std::chrono::milliseconds(1000));
                is_sleeping = false;
            }

            std::cout << "Coverage scan complete. Visited "
                      << path.size() << " viewpoints." << std::endl;
        }

        // Return arm to initial recording position before reset
        joint_positions[0] = -0 * D2R;
        joint_positions[1] = -90 * D2R;
        joint_positions[2] = 0 * D2R;
        joint_positions[3] = -110 * D2R;
        joint_positions[4] = 0 * D2R;
        joint_positions[5] = -35 * D2R;
        joint_positions[6] = 0 * D2R;
        right_arm.setJointValueTarget(joint_positions);
        if (right_arm.plan(joint_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            right_arm.execute(joint_plan);
        }
    }

    // Stop recording and save right-arm trajectory to .npy
    recording_right_arm_ = false;
    std::string output_path = node->get_parameter("joint_trajectory_output_path").as_string();
    size_t saved_count = writeRightArmTrajectoryNpy(output_path);
    if (saved_count > 0) {
        std::cout << "Saved right-arm joint trajectory: " << saved_count
                  << " samples to " << output_path << std::endl;
    } else {
        std::cout << "No joint trajectory data to save (buffer empty or write failed)." << std::endl;
    }

    // ========================================================================
    // Reset and cleanup (unchanged from original)
    // ========================================================================

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
