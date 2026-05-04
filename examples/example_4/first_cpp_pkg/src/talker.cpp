#include <chrono>   // for timing (sleep / timer)
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"                         // ROS 2 C++ API
#include "first_cpp_pkg/msg/robot_status.hpp"        // custom message type

using namespace std::chrono_literals;

// -----------------------------
// This class defines a ROS 2 node
// -----------------------------
class Talker : public rclcpp::Node
{
public:
    // Constructor: runs when node starts
    Talker() : Node("talker"), count_(0)
    {
        // Create a publisher
        // Topic name: "robot_status"
        // Queue size: 10
        publisher_ = this->create_publisher<first_cpp_pkg::msg::RobotStatus>("robot_status", 10);

        // Create a timer that calls a function every second
        timer_ = this->create_wall_timer(
            1s, std::bind(&Talker::publish_message, this));
    }

private:
    // Function that gets called periodically
    void publish_message()
    {
        // Create a custom message
        first_cpp_pkg::msg::RobotStatus msg;

        // Fill message content
        msg.name = "tutorial_bot";
        msg.count = count_++;
        msg.battery_percent = 100.0f - static_cast<float>(msg.count) * 0.5f;
        msg.is_moving = (msg.count % 2 == 0);

        // Print to terminal
        RCLCPP_INFO(
            this->get_logger(),
            "Publishing: name='%s', count=%d, battery=%.1f%%, is_moving=%s",
            msg.name.c_str(),
            msg.count,
            msg.battery_percent,
            msg.is_moving ? "true" : "false");

        // Publish the message
        publisher_->publish(msg);
    }

    // ROS 2 publisher object
    rclcpp::Publisher<first_cpp_pkg::msg::RobotStatus>::SharedPtr publisher_;

    // Timer object (calls function periodically)
    rclcpp::TimerBase::SharedPtr timer_;

    // Simple counter
    int count_;
};

// -----------------------------
// Main function (entry point)
// -----------------------------
int main(int argc, char * argv[])
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create and run node
    rclcpp::spin(std::make_shared<Talker>());

    // Shutdown ROS 2
    rclcpp::shutdown();

    return 0;
}
