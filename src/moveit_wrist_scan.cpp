#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"
#include "rby1-sdk/upc/device.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <random>
#include <set>
#include <limits>
#include <cmath>
#include <mutex>
#include <array>

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

void encoder_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (msg->position.size() < 15) {
        return;
    }
    for (size_t i = 0; i < 24 && i < msg->position.size(); ++i) {
        latest_joint_positions_[i] = msg->position[i];
    }
}

double get_encoder(const std::string& name) {
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
// Wrist Scan Helper Structures
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
// Wrist Scan Helper Functions
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

// Compute an EE orientation where -Z axis points from camera_pos toward object_center.
// Matches the camera mounting: optical axis = -Z of link_left_arm_6.
Eigen::Quaterniond lookAtObject(const Eigen::Vector3d& camera_pos,
                                const Eigen::Vector3d& object_center) {
    Eigen::Vector3d z_axis = (camera_pos - object_center).normalized();  // -Z toward object
    Eigen::Vector3d world_up(0, 0, 1);
    Eigen::Vector3d x_candidate = world_up.cross(z_axis);
    Eigen::Vector3d x_axis = (x_candidate.norm() < 1e-6)
        ? Eigen::Vector3d(1, 0, 0)
        : x_candidate.normalized();
    Eigen::Vector3d y_axis = z_axis.cross(x_axis);
    Eigen::Matrix3d R;
    R.col(0) = x_axis;
    R.col(1) = y_axis;
    R.col(2) = z_axis;
    return Eigen::Quaterniond(R).normalized();
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

// Greedy nearest-neighbor: visit the closest unvisited coverage point at each step.
// Safety threshold: reject jumps larger than 3.0 rad (tighter than coverage scan's 5.2).
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

        // Stop if nothing reachable (safety: 5.5 rad max jump)
        if (best_node == -1 || best_dist > 5.5) break;

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
        "moveit_wrist_scan",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    auto const logger = rclcpp::get_logger("moveit_wrist_scan");

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

    std::cout << "Press Enter to begin wrist scan setup..." << std::endl;
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
    auto marker_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("/wrist_scan_markers", 10);
    // Publisher thread (publishes at 10Hz).
    // is_sleeping=true signals the capture node that the arm has settled at a viewpoint.
    // TODO (RealSense integration): capture node subscribes to waiting_flag and triggers
    //   depth frame acquisition when temperature==1.0.
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

    // ========================================================================
    // WRIST SCAN: Left arm sweeps RealSense around stationary object held by right arm
    // ========================================================================

    using moveit::planning_interface::MoveGroupInterface;

    // Hardcoded constants
    //Eigen::Vector3d object_center(0.338, -0.191, 1.362);  // joints: 0,-90,0,-135,0,-15,0
    //Eigen::Vector3d object_center(0.415, -0.1500, 1.310);
    Eigen::Vector3d object_center(0.427, -0.005, 1.365);   // joints: -71,-60,62,-113,56,3,-56
    double scan_radius = 0.16;                             // camera orbit radius [m]
    int NUM_SPHERE_POINTS = 32;
    int NUM_IK_SEEDS = 6;

    // -------------------------------------------------------------------------
    // Step A: Command RIGHT ARM to holding pose (joint space)
    // -------------------------------------------------------------------------
    auto right_arm = MoveGroupInterface(node, "rby1_right_arm");
    moveit::planning_interface::MoveGroupInterface::Plan right_plan;

    std::vector<double> right_hold_joints(7);
    // right_hold_joints[0] = 0   * D2R;  // pose A: 0,-90,0,-135,0,-15,0
    // right_hold_joints[1] = -90 * D2R;
    // right_hold_joints[2] = 0   * D2R;
    // right_hold_joints[3] = -135 * D2R;
    // right_hold_joints[4] = 0   * D2R;
    // right_hold_joints[5] = -15 * D2R;
    // right_hold_joints[6] = 0   * D2R;
    right_hold_joints[0] = -71  * D2R;  // pose B: -71,-60,62,-113,56,3,-56
    right_hold_joints[1] = -60  * D2R;
    right_hold_joints[2] = 62   * D2R;
    right_hold_joints[3] = -113 * D2R;
    right_hold_joints[4] = 56   * D2R;
    right_hold_joints[5] = 3    * D2R;
    right_hold_joints[6] = -56  * D2R;

    RCLCPP_INFO(logger, "Moving right arm to holding pose...");
    right_arm.setJointValueTarget(right_hold_joints);
    if (right_arm.plan(right_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        right_arm.execute(right_plan);
    } else {
        RCLCPP_WARN(logger, "Right arm plan to holding pose failed.");
    }
    rclcpp::sleep_for(std::chrono::milliseconds(1000));

    // -------------------------------------------------------------------------
    // Step B: Get LEFT ARM MoveGroupInterface and current state
    // -------------------------------------------------------------------------
    auto left_arm = MoveGroupInterface(node, "rby1_left_arm");
    moveit::planning_interface::MoveGroupInterface::Plan left_plan;

    auto current_state = left_arm.getCurrentState(10.0);
    if (!current_state) {
        RCLCPP_ERROR(logger, "Failed to get left arm current state. Aborting scan.");
        running = false;
        spin_thread.join();
        rclcpp::shutdown();
        return 1;
    }
    current_state->update();

    auto robot_model = left_arm.getRobotModel();
    const auto* left_jmg = robot_model->getJointModelGroup("rby1_left_arm");

    std::vector<double> start_joints;
    current_state->copyJointGroupPositions(left_jmg, start_joints);

    // -------------------------------------------------------------------------
    // Planning scene collision objects
    // -------------------------------------------------------------------------
    moveit::planning_interface::PlanningSceneInterface psi;

    // *** TUNE THESE to align collision objects in RViz ***
    double camera_box_x = 0.0;   // wrist camera offset from link_left_arm_6
    double camera_box_y = 0.05;  // 5cm to the left
    double camera_box_z = -0.10; // 5cm original - 15cm = -10cm
    double sphere_z     = -0.08; // 8cm forward from ee_right

    // Object 1: Wrist camera box on left EE (75x75x40mm)
    {
        moveit_msgs::msg::AttachedCollisionObject obj;
        obj.link_name = "link_left_arm_6";
        obj.object.id = "wrist_camera";
        obj.object.header.frame_id = "link_left_arm_6";
        obj.object.header.stamp = node->now();
        shape_msgs::msg::SolidPrimitive box;
        box.type = shape_msgs::msg::SolidPrimitive::BOX;
        box.dimensions = {0.0975, 0.0975, 0.052};
        geometry_msgs::msg::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.x = camera_box_x;
        pose.position.y = camera_box_y;
        pose.position.z = camera_box_z;
        obj.object.primitives.push_back(box);
        obj.object.primitive_poses.push_back(pose);
        obj.object.operation = moveit_msgs::msg::CollisionObject::ADD;
        obj.touch_links = {"link_left_arm_3", "link_left_arm_4", "link_left_arm_5", "link_left_arm_6", "ee_left", "ee_finger_l1", "ee_finger_l2", "FT_sensor_L"};
        psi.applyAttachedCollisionObject(obj);
        RCLCPP_INFO(logger, "Collision object added: wrist_camera (box) on link_left_arm_6");
    }

    // Object 2: Grasped object buffer sphere on right EE (radius 90mm)
    {
        moveit_msgs::msg::AttachedCollisionObject obj;
        obj.link_name = "ee_right";
        obj.object.id = "grasped_object";
        obj.object.header.frame_id = "ee_right";
        obj.object.header.stamp = node->now();
        shape_msgs::msg::SolidPrimitive sphere;
        sphere.type = shape_msgs::msg::SolidPrimitive::SPHERE;
        sphere.dimensions = {0.09};
        geometry_msgs::msg::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.z = sphere_z;
        obj.object.primitives.push_back(sphere);
        obj.object.primitive_poses.push_back(pose);
        obj.object.operation = moveit_msgs::msg::CollisionObject::ADD;
        obj.touch_links = {"link_right_arm_3", "link_right_arm_4", "link_right_arm_5", "link_right_arm_6", "ee_right", "ee_finger_r1", "ee_finger_r2", "FT_sensor_R"};
        psi.applyAttachedCollisionObject(obj);
        RCLCPP_INFO(logger, "Collision object added: grasped_object (sphere r=0.09) on ee_right");
    }

    // Object 3: ZED head camera box (170x40x40mm, at camera_link origin)
    {
        moveit_msgs::msg::AttachedCollisionObject obj;
        obj.link_name = "link_head_2";
        obj.object.id = "zed_camera";
        obj.object.header.frame_id = "link_head_2";
        obj.object.header.stamp = node->now();
        shape_msgs::msg::SolidPrimitive box;
        box.type = shape_msgs::msg::SolidPrimitive::BOX;
        box.dimensions = {0.040, 0.170, 0.040};
        geometry_msgs::msg::Pose pose;
        pose.position.z = 0.05;
        pose.orientation.w = 1.0;
        obj.object.primitives.push_back(box);
        obj.object.primitive_poses.push_back(pose);
        obj.object.operation = moveit_msgs::msg::CollisionObject::ADD;
        obj.touch_links = {"link_head_0", "link_head_1", "link_head_2", "link_torso_0", "link_torso_1", "link_torso_2", "link_torso_3", "link_torso_4", "link_torso_5"};
        psi.applyAttachedCollisionObject(obj);
        RCLCPP_INFO(logger, "Collision object added: zed_camera (box) on link_head_2");
    }

    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // -------------------------------------------------------------------------
    // Step C: Generate Fibonacci sphere viewpoints, sample IK for left arm
    // -------------------------------------------------------------------------
    RCLCPP_INFO(logger, "Generating %d Fibonacci sphere viewpoints around object...", NUM_SPHERE_POINTS);
    auto sphere_points = generateFibonacciSphere(NUM_SPHERE_POINTS);

    std::vector<IKConfig> all_configs;
    int orientations_with_ik = 0;

    for (int i = 0; i < (int)sphere_points.size(); i++) {
        // Camera position: on a sphere of scan_radius centered at the object
        Eigen::Vector3d camera_pos = object_center + scan_radius * sphere_points[i];

        // EE target: X-axis toward object, position offset so camera lands at camera_pos
        Eigen::Matrix3d R = lookAtObject(camera_pos, object_center).toRotationMatrix();
        Eigen::Vector3d camera_offset(camera_box_x, camera_box_y, camera_box_z);
        Eigen::Isometry3d target_pose = Eigen::Isometry3d::Identity();
        target_pose.translation() = camera_pos - R * camera_offset;
        target_pose.linear() = R;

        auto configs = sampleIKConfigs(current_state, left_jmg, target_pose, i, NUM_IK_SEEDS);

        if (!configs.empty()) {
            orientations_with_ik++;
            RCLCPP_DEBUG(logger, "  Viewpoint %d: %zu IK solutions", i, configs.size());
        }
        all_configs.insert(all_configs.end(), configs.begin(), configs.end());
    }

    RCLCPP_INFO(logger, "IK sampling complete: %zu configs for %d/%d viewpoints",
                all_configs.size(), orientations_with_ik, NUM_SPHERE_POINTS);

    if (all_configs.empty()) {
        RCLCPP_ERROR(logger, "No IK solutions found. Aborting scan.");
        running = false;
        spin_thread.join();
        rclcpp::shutdown();
        return 1;
    }

    // -------------------------------------------------------------------------
    // Step D: Build configuration graph and find greedy coverage path
    // -------------------------------------------------------------------------
    auto graph = buildConfigGraph(all_configs, 2.0);
    RCLCPP_INFO(logger, "Config graph: %zu nodes", graph.size());

    auto path = findCoveragePath(graph, start_joints, NUM_SPHERE_POINTS);
    RCLCPP_INFO(logger, "Coverage path: %zu viewpoints planned", path.size());

    // -------------------------------------------------------------------------
    // Visualize viewpoints and planned path in RViz (/wrist_scan_markers)
    // -------------------------------------------------------------------------
    visualization_msgs::msg::MarkerArray marker_array;
    {
        int mid = 0;
        auto now = node->now();

        // Build set of coverage indices that have IK solutions
        std::set<int> covered_indices;
        for (const auto& cfg : all_configs) covered_indices.insert(cfg.coverage_point_idx);

        // Object center reference sphere (yellow)
        visualization_msgs::msg::Marker obj;
        obj.header.frame_id = "base";
        obj.header.stamp = now;
        obj.ns = "object"; obj.id = mid++;
        obj.type = visualization_msgs::msg::Marker::SPHERE;
        obj.action = visualization_msgs::msg::Marker::ADD;
        obj.pose.position.x = object_center.x();
        obj.pose.position.y = object_center.y();
        obj.pose.position.z = object_center.z();
        obj.pose.orientation.w = 1.0;
        obj.scale.x = obj.scale.y = obj.scale.z = 0.06;
        obj.color.r = 1.0; obj.color.g = 1.0; obj.color.b = 0.0; obj.color.a = 1.0;
        marker_array.markers.push_back(obj);

        // Camera direction arrow attached to moving link (orange, moves with arm)
        {
            visualization_msgs::msg::Marker cam_arrow;
            cam_arrow.header.frame_id = "link_left_arm_6"; cam_arrow.header.stamp = now;
            cam_arrow.ns = "camera_dir"; cam_arrow.id = mid++;
            cam_arrow.type = visualization_msgs::msg::Marker::ARROW;
            cam_arrow.action = visualization_msgs::msg::Marker::ADD;
            cam_arrow.pose.position.x = camera_box_x;
            cam_arrow.pose.position.y = camera_box_y;
            cam_arrow.pose.position.z = camera_box_z;
            {
                Eigen::Quaterniond cam_q = Eigen::Quaterniond::FromTwoVectors(
                    Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 0, -1));
                cam_arrow.pose.orientation.x = cam_q.x();
                cam_arrow.pose.orientation.y = cam_q.y();
                cam_arrow.pose.orientation.z = cam_q.z();
                cam_arrow.pose.orientation.w = cam_q.w();
            }
            cam_arrow.frame_locked = true;
            cam_arrow.scale.x = 0.12; cam_arrow.scale.y = 0.015; cam_arrow.scale.z = 0.015;
            cam_arrow.color.r = 1.0; cam_arrow.color.g = 0.5; cam_arrow.color.b = 0.0; cam_arrow.color.a = 1.0;
            marker_array.markers.push_back(cam_arrow);
        }

        // Viewpoint spheres + look-at arrows (green=IK found, red=no IK)
        for (int i = 0; i < (int)sphere_points.size(); i++) {
            Eigen::Vector3d cam = object_center + scan_radius * sphere_points[i];
            bool has_ik = covered_indices.count(i) > 0;

            visualization_msgs::msg::Marker vp;
            vp.header.frame_id = "base"; vp.header.stamp = now;
            vp.ns = "viewpoints"; vp.id = mid++;
            vp.type = visualization_msgs::msg::Marker::SPHERE;
            vp.action = visualization_msgs::msg::Marker::ADD;
            vp.pose.position.x = cam.x();
            vp.pose.position.y = cam.y();
            vp.pose.position.z = cam.z();
            vp.pose.orientation.w = 1.0;
            vp.scale.x = vp.scale.y = vp.scale.z = 0.04;
            vp.color.r = has_ik ? 0.0f : 1.0f;
            vp.color.g = has_ik ? 1.0f : 0.0f;
            vp.color.b = 0.0; vp.color.a = has_ik ? 1.0f : 0.4f;
            marker_array.markers.push_back(vp);

            // Arrow pointing from camera toward object (RViz arrows point along X-axis)
            Eigen::Vector3d toward = (object_center - cam).normalized();
            Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(1, 0, 0), toward);
            visualization_msgs::msg::Marker arrow;
            arrow.header.frame_id = "base"; arrow.header.stamp = now;
            arrow.ns = "directions"; arrow.id = mid++;
            arrow.type = visualization_msgs::msg::Marker::ARROW;
            arrow.action = visualization_msgs::msg::Marker::ADD;
            arrow.pose.position.x = cam.x();
            arrow.pose.position.y = cam.y();
            arrow.pose.position.z = cam.z();
            arrow.pose.orientation.x = q.x();
            arrow.pose.orientation.y = q.y();
            arrow.pose.orientation.z = q.z();
            arrow.pose.orientation.w = q.w();
            arrow.scale.x = 0.08; arrow.scale.y = 0.01; arrow.scale.z = 0.01;
            arrow.color.r = has_ik ? 0.0f : 0.8f;
            arrow.color.g = has_ik ? 0.8f : 0.0f;
            arrow.color.b = 0.0; arrow.color.a = has_ik ? 1.0f : 0.3f;
            marker_array.markers.push_back(arrow);
        }

        // Planned path as a blue LINE_STRIP
        if (!path.empty()) {
            visualization_msgs::msg::Marker line;
            line.header.frame_id = "base"; line.header.stamp = now;
            line.ns = "path"; line.id = mid++;
            line.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line.action = visualization_msgs::msg::Marker::ADD;
            line.pose.orientation.w = 1.0;
            line.scale.x = 0.012;
            line.color.r = 0.0; line.color.g = 0.5; line.color.b = 1.0; line.color.a = 1.0;
            for (int node_idx : path) {
                Eigen::Vector3d cam = object_center + scan_radius * sphere_points[graph[node_idx].coverage_idx];
                geometry_msgs::msg::Point pt;
                pt.x = cam.x(); pt.y = cam.y(); pt.z = cam.z();
                line.points.push_back(pt);
            }
            marker_array.markers.push_back(line);
        }

    }

    // Republish markers continuously until user starts the scan
    std::atomic<bool> stop_markers{false};
    std::thread marker_thread([&]() {
        while (!stop_markers) {
            auto ts = node->now();
            for (auto& m : marker_array.markers) m.header.stamp = ts;
            marker_pub->publish(marker_array);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    });
    RCLCPP_INFO(logger, "Publishing markers to /wrist_scan_markers — set Fixed Frame to 'base' in RViz, then Add > By Topic > MarkerArray.");

    std::cout << "Press Enter to start wrist scan..." << std::endl;
    std::cin.get();
    stop_markers = true;
    marker_thread.join();

    // -------------------------------------------------------------------------
    // Step E: Execute path on LEFT ARM
    // -------------------------------------------------------------------------
    for (int step = 0; step < (int)path.size(); step++) {
        int node_idx = path[step];
        int cov_idx = graph[node_idx].coverage_idx;
        RCLCPP_INFO(logger, "Viewpoint %d/%d (coverage point %d)",
                    step + 1, (int)path.size(), cov_idx);

        // Fetch fresh state before each move
        current_state = left_arm.getCurrentState(5.0);

        bool executed = false;

        left_arm.setJointValueTarget(graph[node_idx].joints);
        if (left_arm.plan(left_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            left_arm.execute(left_plan);
            executed = true;
        } else {
            RCLCPP_WARN(logger, "  Primary IK failed for coverage point %d, trying alternatives...", cov_idx);

            // Fallback: try other IK configs for the same coverage point
            for (size_t i = 0; i < graph.size(); ++i) {
                if ((int)i == node_idx) continue;
                if (graph[i].coverage_idx != cov_idx) continue;

                left_arm.setJointValueTarget(graph[i].joints);
                if (left_arm.plan(left_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
                    RCLCPP_INFO(logger, "  Using alternative IK solution %zu for coverage point %d", i, cov_idx);
                    left_arm.execute(left_plan);
                    executed = true;
                    break;
                }
            }
        }

        if (!executed) {
            RCLCPP_WARN(logger, "  Skipping viewpoint %d/%d (coverage point %d) — no valid plan",
                        step + 1, (int)path.size(), cov_idx);
            continue;
        }

        // Arm has settled at viewpoint — signal capture node and pause
        // TODO (RealSense integration): capture node triggers on is_sleeping==true to
        //   acquire one depth frame, project to point cloud in base frame, and accumulate.
        is_sleeping = true;
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        is_sleeping = false;
    }

    RCLCPP_INFO(logger, "Wrist scan complete. Visited %zu viewpoints.", path.size());

    // -------------------------------------------------------------------------
    // Step F: Return LEFT ARM to zero joints
    // -------------------------------------------------------------------------
    RCLCPP_INFO(logger, "Returning left arm to zero...");
    std::vector<double> zero_joints(7, 0.0);
    left_arm.setJointValueTarget(zero_joints);
    if (left_arm.plan(left_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        left_arm.execute(left_plan);
    } else {
        RCLCPP_WARN(logger, "Left arm reset plan failed.");
    }

    // -------------------------------------------------------------------------
    // Step G: Return RIGHT ARM to zero joints
    // -------------------------------------------------------------------------
    RCLCPP_INFO(logger, "Returning right arm to zero...");
    right_arm.setJointValueTarget(zero_joints);
    if (right_arm.plan(right_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        right_arm.execute(right_plan);
    } else {
        RCLCPP_WARN(logger, "Right arm reset plan failed.");
    }

    // -------------------------------------------------------------------------
    // Step H: Open gripper to release object
    // -------------------------------------------------------------------------
    std::cout << "Press Enter to open gripper..." << std::endl;
    std::cin.get();

    gripper_position_command(portHandler_gripper, packetHandler_gripper, activeIDs_gripper, 0.1, 0.1);

    // Stop spinning and shutdown
    running = false;
    spin_thread.join();

    rclcpp::shutdown();
    return 0;
}
