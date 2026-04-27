#include <chrono>              // for timing (sleep / timer)
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"  // ROS 2 C++ API
#include "std_msgs/msg/string.hpp"  // message type

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
        // Topic name: "chatter"
        // Queue size: 10
        publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);

        // Create a timer that calls a function every 1 second
        timer_ = this->create_wall_timer(
            1s, std::bind(&Talker::publish_message, this));
    }

private:
    // Function that gets called every second
    void publish_message()
    {
        // Create a message
        std_msgs::msg::String msg;

        // Fill message content
        msg.data = "Hello from ROS 2! Count: " + std::to_string(count_++);

        // Print to terminal
        RCLCPP_INFO(this->get_logger(), "%s", msg.data.c_str());

        // Publish the message
        publisher_->publish(msg);
    }

    // ROS 2 publisher object
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

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