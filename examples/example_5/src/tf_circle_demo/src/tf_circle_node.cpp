#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"

using namespace std::chrono_literals;

class TFCircleNode : public rclcpp::Node
{
public:
  TFCircleNode()
  : Node("tf_circle_node")
  {
    dynamic_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    start_time_ = this->now();

    publish_static_transform();

    timer_ = this->create_wall_timer(
      50ms,
      std::bind(&TFCircleNode::publish_dynamic_transform, this));
  }


private:
  void publish_static_transform()
  {
    geometry_msgs::msg::TransformStamped static_tf;

    static_tf.header.stamp = this->now();
    static_tf.header.frame_id = "robot";
    static_tf.child_frame_id = "sensor";

    static_tf.transform.translation.x = 0.5;
    static_tf.transform.translation.y = 0.0;
    static_tf.transform.translation.z = 0.2;

    static_tf.transform.rotation.x = 0.0;
    static_tf.transform.rotation.y = 0.0;
    static_tf.transform.rotation.z = 0.0;
    static_tf.transform.rotation.w = 1.0;

    static_broadcaster_->sendTransform(static_tf);

    RCLCPP_INFO(this->get_logger(), "Published static transform robot -> sensor");
  }

  void publish_dynamic_transform()
  {
    rclcpp::Time current_time = this->now();
    double t = (current_time - start_time_).seconds();

    double radius = 2.0;
    double angular_speed = 0.5;
    double angle = angular_speed * t;

    geometry_msgs::msg::TransformStamped tf_msg;

    tf_msg.header.stamp = current_time;
    tf_msg.header.frame_id = "world";
    tf_msg.child_frame_id = "robot";

    tf_msg.transform.translation.x = radius * std::cos(angle);
    tf_msg.transform.translation.y = radius * std::sin(angle);
    tf_msg.transform.translation.z = 0.0;

    tf_msg.transform.rotation.x = 0.0;
    tf_msg.transform.rotation.y = 0.0;
    tf_msg.transform.rotation.z = std::sin(angle / 2.0);
    tf_msg.transform.rotation.w = std::cos(angle / 2.0);

    dynamic_broadcaster_->sendTransform(tf_msg);
  }

  rclcpp::Time start_time_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> dynamic_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TFCircleNode>());
  rclcpp::shutdown();
  return 0;
}