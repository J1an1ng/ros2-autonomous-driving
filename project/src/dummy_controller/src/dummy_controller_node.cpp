#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "simulation/msg/vehicle_control.hpp"

using namespace std::chrono_literals;

class DummyController : public rclcpp::Node
{
public:
  DummyController()
  : Node("dummy_controller_node")
  {
    publisher_ = create_publisher<simulation::msg::VehicleControl>("car_command", 1);
    timer_ = create_wall_timer(50ms, [this]() {publish_command();});
  }

private:
  void publish_command()
  {
    simulation::msg::VehicleControl msg;
    msg.throttle = 0.5F;
    msg.steering = std::sin(2.0F * std::acos(-1.0F) * elapsed_time_) * 0.5F;
    msg.brake = 0.0F;
    msg.reserved = 0.0F;

    publisher_->publish(msg);
    elapsed_time_ += 0.05F;
  }

  rclcpp::Publisher<simulation::msg::VehicleControl>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  float elapsed_time_{0.0F};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummyController>());
  rclcpp::shutdown();
  return 0;
}
