#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("state_publisher");
  auto joint_pub = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  tf2_ros::TransformBroadcaster broadcaster(node);
  rclcpp::Rate loop_rate(30.0);

  constexpr double pi = 3.14159265358979323846;
  const double degree = pi / 180.0;

  // Robot state.
  double tilt = 0.0;
  double tinc = degree;
  double swivel = 0.0;
  double angle = 0.0;
  double height = 0.0;
  double hinc = 0.005;

  geometry_msgs::msg::TransformStamped odom_trans;
  sensor_msgs::msg::JointState joint_state;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "axis";

  while (rclcpp::ok()) {
    const auto now = node->get_clock()->now();

    // Update joint_state.
    joint_state.header.stamp = now;
    joint_state.name.resize(3);
    joint_state.position.resize(3);
    joint_state.name[0] = "swivel";
    joint_state.position[0] = swivel;
    joint_state.name[1] = "tilt";
    joint_state.position[1] = tilt;
    joint_state.name[2] = "periscope";
    joint_state.position[2] = height;

    // Update transform. The robot moves in a circle with radius 2.
    odom_trans.header.stamp = now;
    odom_trans.transform.translation.x = std::cos(angle) * 2.0;
    odom_trans.transform.translation.y = std::sin(angle) * 2.0;
    odom_trans.transform.translation.z = 0.7;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, angle + pi / 2.0);
    odom_trans.transform.rotation.x = q.x();
    odom_trans.transform.rotation.y = q.y();
    odom_trans.transform.rotation.z = q.z();
    odom_trans.transform.rotation.w = q.w();

    // Send the joint state and transform.
    joint_pub->publish(joint_state);
    broadcaster.sendTransform(odom_trans);

    // Create new robot state.
    tilt += tinc;
    if (tilt < -0.5 || tilt > 0.0) {
      tinc *= -1.0;
    }

    height += hinc;
    if (height > 0.2 || height < 0.0) {
      hinc *= -1.0;
    }

    swivel += degree;
    angle += degree / 4.0;

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
