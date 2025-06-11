#include <memory>
#include <cmath>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include "rby1-sdk/model.h"
#include "rby1-sdk/robot.h"
#include "rby1-sdk/upc/device.h"
#include <chrono>

#define DISTANCE_TO_WHEEL_ANGLE 10
#define SPEED_TO_WHEEL_W 1 //10
#define ANGLE_TO_WHEEL_ANGLE 2.65
#define W_TO_WHEEL_W 2.65
#define M_PI 3.14159265358979323846

using namespace rb;
const std::string kAll = ".*";
std::atomic<bool> running{true};
rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
std::vector<double> latest_joint_positions_(2, 0.0);

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

void set_SE2_speed(double linear, double angular, const rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr& publisher,
    trajectory_msgs::msg::JointTrajectory& msg, trajectory_msgs::msg::JointTrajectoryPoint& point){
    point.positions = {linear*SPEED_TO_WHEEL_W, angular*W_TO_WHEEL_W};
    msg.points.clear();
    msg.points.push_back(point);
    publisher->publish(msg);
}


void move_linear(double distance, double speed, const rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr& publisher,
    trajectory_msgs::msg::JointTrajectory& msg, trajectory_msgs::msg::JointTrajectoryPoint& point){
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
        set_SE2_speed(speed, 0.0, publisher, msg, point);
        rate.sleep();
    }

    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "Linear command loop execution time: " << elapsed_seconds.count() << " seconds" << std::endl;
    std::cout << "Wheel angle traversed: " << std::abs(get_wheel_position("right_wheel") - start_pos_r) << " rad" << std::endl;

    std::cout << "Current right wheel position after linear: " << get_wheel_position("right_wheel") << " rad" << std::endl;
    std::cout << "Current left wheel position after linear: " << get_wheel_position("left_wheel") << " rad" << std::endl;

    // sending stop command
    set_SE2_speed(0.0, 0.0, publisher, msg, point);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    set_SE2_speed(0.0, 0.0, publisher, msg, point);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    set_SE2_speed(0.0, 0.0, publisher, msg, point);
}

void move_angular(double angle, double w, const rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr& publisher,
    trajectory_msgs::msg::JointTrajectory& msg, trajectory_msgs::msg::JointTrajectoryPoint& point){
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
        set_SE2_speed(0.0, w, publisher, msg, point);
        rate.sleep();
    }

    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "Angular command loop execution time: " << elapsed_seconds.count() << " seconds" << std::endl;
    std::cout << "Wheel angle traversed: " << std::abs(get_wheel_position("right_wheel") - start_pos_r) << " rad" << std::endl;

    std::cout << "Current right wheel position after angular: " << get_wheel_position("right_wheel") << " rad" << std::endl;
    std::cout << "Current left wheel position after angular: " << get_wheel_position("left_wheel") << " rad" << std::endl;

    // sending stop command
    set_SE2_speed(0.0, 0.0, publisher, msg, point);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    set_SE2_speed(0.0, 0.0, publisher, msg, point);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    set_SE2_speed(0.0, 0.0, publisher, msg, point);
}

int main(int argc, char * argv[])
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create a ROS 2 node

    auto const node = std::make_shared<rclcpp::Node>(
        "simple_base",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );
    

    auto const logger = rclcpp::get_logger("simple_gripper");

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

    // initializing trajectory publisher
    auto publisher = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/rby1_base_controller/joint_trajectory", 10);

    // initializing message
    trajectory_msgs::msg::JointTrajectory msg;
    msg.joint_names = {"right_wheel", "left_wheel"};
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {0.0, 0.0};
    point.time_from_start.sec = 0;
    point.time_from_start.nanosec = 0;
    msg.points.push_back(point);

    std::this_thread::sleep_for(std::chrono::seconds(5));

    move_linear(0.3, -0.2, publisher, msg, point);
    move_angular(M_PI/2, 0.3, publisher, msg, point);
    move_linear(0.3, 0.2, publisher, msg, point);

    /*
    move_linear(1, 0.2, publisher, msg, point);
    move_angular(M_PI/2, 0.2, publisher, msg, point);
    set_SE2_speed(0.0, 1.0, publisher, msg, point);
    std::this_thread::sleep_for(std::chrono::seconds(5));
    set_SE2_speed(0.0, 0.0, publisher, msg, point);
    */


    /*
        
    // creating forward message
    trajectory_msgs::msg::JointTrajectory msg;
    msg.joint_names = {"right_wheel", "left_wheel"};
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {0.3, 0.0};
    point.time_from_start.sec = 0;
    point.time_from_start.nanosec = 0;
    msg.points.push_back(point);

    // declaring current wheel position
    std::cout << "Current right wheel position from listener: " << get_wheel_position("right_wheel") << " rad" << std::endl;
    std::cout << "Current left wheel position from listener: " << get_wheel_position("left_wheel") << " rad" << std::endl;


    // publishing forward message
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    publisher->publish(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    publisher->publish(msg);
    RCLCPP_INFO(logger, "Published base wheel trajectory command");

    // wait to complete two full wheel revolutions
    const double target_position = -3.14 * 4;
    rclcpp::Rate rate(100); // 100 Hz
    while (rclcpp::ok() && get_wheel_position("left_wheel") > target_position) {
        rate.sleep();
    }
    
    // creating stop message
    point.positions = {0.0, 0.0};
    point.time_from_start.sec = 0;
    point.time_from_start.nanosec = 0;
    msg.points.clear();
    msg.points.push_back(point);
    // publishing stop message
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    publisher->publish(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    publisher->publish(msg);
    RCLCPP_INFO(logger, "Published base wheel trajectory command");

    // declaring final wheel position
    std::cout << "Current right wheel position from listener: " << get_wheel_position("right_wheel") << " rad" << std::endl;
    std::cout << "Current left wheel position from listener: " << get_wheel_position("left_wheel") << " rad" << std::endl;

    */

    // Stop spinning
    running = false;
    spin_thread.join();
    
    //Shutdown
    rclcpp::shutdown();
    return 0;
}