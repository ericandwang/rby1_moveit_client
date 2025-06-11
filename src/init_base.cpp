#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>

using namespace std::chrono_literals;

class TrajectoryPublisher : public rclcpp::Node {
public:
    TrajectoryPublisher()
    : Node("init_base"), count_(0) {
        // Controller state client
        controller_client_ = create_client<controller_manager_msgs::srv::ListControllers>(
            "/controller_manager/list_controllers"
        );

        // Publisher setup
        publisher_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/rby1_base_controller/joint_trajectory", 10
        );

        // Timer with async context
        timer_ = create_wall_timer(100ms, [this]() {
            // Async service call wrapper
            auto check_and_publish = [this]() {
                auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
                
                controller_client_->async_send_request(request,
                    [this](rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedFuture future) {
                        auto response = future.get();
                        bool controller_active = false;
                        
                        // Check controller state
                        for (const auto& controller : response->controller) {
                            if (controller.name == "rby1_base_controller" && 
                                controller.state == "active") {
                                controller_active = true;
                                break;
                            }
                        }

                        // Only increment and publish if controller is active
                        if (controller_active) {
                            publish_trajectory();
                            if (++count_ > 15) rclcpp::shutdown();
                        }
                    }
                );
            };
            
            // Ensure service availability
            if (!controller_client_->service_is_ready()) {
                RCLCPP_WARN(get_logger(), "Controller manager service not available");
                return;
            }
            check_and_publish();
        });
    }

private:
    void publish_trajectory() {
        trajectory_msgs::msg::JointTrajectory msg;
        msg.joint_names = {"right_wheel", "left_wheel"};
        
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {0.0, 0.0};
        point.time_from_start.sec = 0;
        msg.points.push_back(point);
        
        publisher_->publish(msg);
    }

    rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr controller_client_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int count_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryPublisher>());
    rclcpp::shutdown();
    return 0;
}
