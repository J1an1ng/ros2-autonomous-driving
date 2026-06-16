#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

class TimeObserver : public rclcpp::Node
{
public:
  TimeObserver() : Node("time_observer")
  {
    start_steady_ = std::chrono::steady_clock::now();

    timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&TimeObserver::onTimer, this));

    RCLCPP_INFO(get_logger(), "Time observer started.");
    RCLCPP_INFO(get_logger(), "Try running with: --ros-args -p use_sim_time:=true");
  }

private:
  void onTimer()
  {
    const auto ros_now = this->get_clock()->now();

    const auto wall_now = std::chrono::system_clock::now();
    const auto wall_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      wall_now.time_since_epoch()).count();

    const auto steady_now = std::chrono::steady_clock::now();
    const auto steady_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      steady_now - start_steady_).count();

    const double wall_seconds = static_cast<double>(wall_ms) / 1000.0;
    const double steady_seconds = static_cast<double>(steady_ms) / 1000.0;
    const double ros_seconds = ros_now.seconds();

    RCLCPP_INFO(
      get_logger(),
      "wall=%.3f s | steady_since_start=%.3f s | ros=%.3f s",
      wall_seconds,
      steady_seconds,
      ros_seconds);
  }

  std::chrono::steady_clock::time_point start_steady_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TimeObserver>());
  rclcpp::shutdown();
  return 0;
}
