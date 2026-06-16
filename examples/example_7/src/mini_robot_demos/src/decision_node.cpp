#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class DecisionNode : public rclcpp::Node
{
public:
  DecisionNode() : Node("decision_node")
  {
    declare_parameter<std::string>("distance_topic", "/mini_robot/distance");
    declare_parameter<double>("stop_distance", 0.8);

    const auto distance_topic = get_parameter("distance_topic").as_string();
    stop_distance_ = get_parameter("stop_distance").as_double();

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/mini_robot/cmd_vel", 10);

    distance_sub_ = create_subscription<std_msgs::msg::Float64>(
      distance_topic,
      10,
      std::bind(&DecisionNode::onDistance, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Subscribing to distance topic: %s", distance_topic.c_str());
    RCLCPP_INFO(get_logger(), "Publishing commands on: /mini_robot/cmd_vel");
  }

private:
  void onDistance(const std_msgs::msg::Float64::SharedPtr msg)
  {
    geometry_msgs::msg::Twist cmd;

    if (msg->data < stop_distance_) {
      cmd.linear.x = 0.0;
      RCLCPP_WARN(get_logger(), "Obstacle close: %.2f m -> STOP", msg->data);
    } else {
      cmd.linear.x = 0.5;
      RCLCPP_INFO(get_logger(), "Distance okay: %.2f m -> FORWARD", msg->data);
    }

    cmd_pub_->publish(cmd);
  }

  double stop_distance_ = 0.8;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr distance_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DecisionNode>());
  rclcpp::shutdown();
  return 0;
}
