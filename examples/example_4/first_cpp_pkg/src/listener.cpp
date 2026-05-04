#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "first_cpp_pkg/msg/robot_status.hpp"

// -----------------------------
// Listener node
// -----------------------------
class Listener : public rclcpp::Node
{
public:
    Listener() : Node("listener")
    {
        // Create a subscriber
        // Topic: "robot_status"
        // Queue size: 10
        subscription_ = this->create_subscription<first_cpp_pkg::msg::RobotStatus>(
            "robot_status",
            10,
            std::bind(&Listener::callback, this, std::placeholders::_1));
    }

private:
    // This function is called whenever a message is received
    void callback(const first_cpp_pkg::msg::RobotStatus::SharedPtr msg) const
    {
        // Print received message
        RCLCPP_INFO(
            this->get_logger(),
            "I heard: name='%s', count=%d, battery=%.1f%%, is_moving=%s",
            msg->name.c_str(),
            msg->count,
            msg->battery_percent,
            msg->is_moving ? "true" : "false");
    }

    // ROS 2 subscriber object
    rclcpp::Subscription<first_cpp_pkg::msg::RobotStatus>::SharedPtr subscription_;
};

// -----------------------------
// Main function
// -----------------------------
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<Listener>());

    rclcpp::shutdown();

    return 0;
}
