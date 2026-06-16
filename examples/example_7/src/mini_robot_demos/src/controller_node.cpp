#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode() : Node("controller_node")
  {
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/mini_robot/cmd_vel",
      10,
      std::bind(&ControllerNode::onCommand, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Waiting for /mini_robot/cmd_vel");
  }

private:
  void onCommand(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (msg->linear.x == 0.0) {
      RCLCPP_WARN(get_logger(), "Controller received: STOP");
    } else {
      RCLCPP_INFO(get_logger(), "Controller received: FORWARD with %.2f m/s", msg->linear.x);
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
