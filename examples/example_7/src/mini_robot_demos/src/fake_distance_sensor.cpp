#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class FakeDistanceSensor : public rclcpp::Node
{
public:
  FakeDistanceSensor() : Node("fake_distance_sensor")
  {
    publisher_ = create_publisher<std_msgs::msg::Float64>("/mini_robot/distance", 10);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&FakeDistanceSensor::onTimer, this));

    RCLCPP_INFO(get_logger(), "Publishing fake distance on /mini_robot/distance");
  }

private:
  void onTimer()
  {
    t_ += 0.2;

    // Fake obstacle distance oscillates between roughly 0.4 m and 2.0 m.
    std_msgs::msg::Float64 msg;
    msg.data = 1.2 + 0.8 * std::sin(t_);
    publisher_->publish(msg);

    RCLCPP_INFO(get_logger(), "distance = %.2f m", msg.data);
  }

  double t_ = 0.0;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeDistanceSensor>());
  rclcpp::shutdown();
  return 0;
}
