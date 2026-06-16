#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

class LectureClockPublisher : public rclcpp::Node
{
public:
  LectureClockPublisher() : Node("lecture_clock")
  {
    declare_parameter<double>("start_seconds", 1000.0);
    declare_parameter<double>("step_seconds", 0.5);
    declare_parameter<double>("publish_period_seconds", 0.5);

    sim_time_seconds_ = get_parameter("start_seconds").as_double();
    const double period = get_parameter("publish_period_seconds").as_double();

    clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(period)),
      std::bind(&LectureClockPublisher::onTimer, this));

    RCLCPP_INFO(get_logger(), "Publishing /clock.");
    RCLCPP_INFO(get_logger(), "Change speed with: ros2 param set /lecture_clock step_seconds 0.0");
  }

private:
  void onTimer()
  {
    const double step = get_parameter("step_seconds").as_double();
    sim_time_seconds_ += step;

    rosgraph_msgs::msg::Clock msg;
    const auto sec = static_cast<int32_t>(std::floor(sim_time_seconds_));
    const auto nanosec = static_cast<uint32_t>((sim_time_seconds_ - static_cast<double>(sec)) * 1e9);

    msg.clock.sec = sec;
    msg.clock.nanosec = nanosec;
    clock_pub_->publish(msg);

    RCLCPP_INFO(get_logger(), "published /clock = %d.%09u s, step = %.3f s", sec, nanosec, step);
  }

  double sim_time_seconds_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LectureClockPublisher>());
  rclcpp::shutdown();
  return 0;
}
