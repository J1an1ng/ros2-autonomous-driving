#include <chrono>   // for timing (sleep / timer)
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"                         // ROS 2 C++ API
#include "first_cpp_pkg/msg/robot_status.hpp"        // custom message type

// -----------------------------
// This class defines a ROS 2 node
// -----------------------------
class Talker : public rclcpp::Node
{
public:
    // Constructor: runs when node starts
    Talker() : Node("talker"), count_(0)
    {
        // Declare parameters with default values
        this->declare_parameter<double>("publish_frequency", 1.0);
        this->declare_parameter<std::string>("robot_name", "tutorial_bot");
        this->declare_parameter<double>("battery_drain_per_message", 0.5);

        // Read parameters
        publish_frequency_ = this->get_parameter("publish_frequency").as_double();
        robot_name_ = this->get_parameter("robot_name").as_string();
        battery_drain_per_message_ =
            this->get_parameter("battery_drain_per_message").as_double();

        // Safety check
        if (publish_frequency_ <= 0.0) {
            RCLCPP_WARN(
                this->get_logger(),
                "publish_frequency must be > 0. Using default value 1.0 Hz.");
            publish_frequency_ = 1.0;
        }

        // Create a publisher
        publisher_ = this->create_publisher<first_cpp_pkg::msg::RobotStatus>(
            "robot_status", 10);

        // Convert frequency in Hz to timer period
        auto period = std::chrono::duration<double>(1.0 / publish_frequency_);

        // Create a timer using the configured frequency
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&Talker::publish_message, this));

        RCLCPP_INFO(
            this->get_logger(),
            "Talker started with robot_name='%s', publish_frequency=%.2f Hz, battery_drain_per_message=%.2f",
            robot_name_.c_str(),
            publish_frequency_,
            battery_drain_per_message_);
    }

private:
    // Function that gets called periodically
    void publish_message()
    {
        first_cpp_pkg::msg::RobotStatus msg;

        msg.name = robot_name_;
        msg.count = count_++;
        msg.battery_percent =
            100.0f - static_cast<float>(msg.count * battery_drain_per_message_);
        msg.is_moving = (msg.count % 2 == 0);

        RCLCPP_INFO(
            this->get_logger(),
            "Publishing: name='%s', count=%d, battery=%.1f%%, is_moving=%s",
            msg.name.c_str(),
            msg.count,
            msg.battery_percent,
            msg.is_moving ? "true" : "false");

        publisher_->publish(msg);
    }

    rclcpp::Publisher<first_cpp_pkg::msg::RobotStatus>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    int count_;

    double publish_frequency_;
    std::string robot_name_;
    double battery_drain_per_message_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Talker>());
    rclcpp::shutdown();

    return 0;
}