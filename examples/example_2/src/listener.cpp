#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// -----------------------------
// Listener node
// -----------------------------
class Listener : public rclcpp::Node
{
public:
    Listener() : Node("listener")
    {
        // Create a subscriber
        // Topic: "chatter"
        // Queue size: 10
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "chatter",
            10,
            std::bind(&Listener::callback, this, std::placeholders::_1));
    }

private:
    // This function is called whenever a message is received
    void callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        // Print received message
        RCLCPP_INFO(this->get_logger(), "I heard: %s", msg->data.c_str());
    }

    // ROS 2 subscriber object
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
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