// Copyright 2026 Team 20

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace
{

struct Point3
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
};

struct RigidTransform
{
  double tx{0.0};
  double ty{0.0};
  double tz{0.0};
  double r00{1.0};
  double r01{0.0};
  double r02{0.0};
  double r10{0.0};
  double r11{1.0};
  double r12{0.0};
  double r20{0.0};
  double r21{0.0};
  double r22{1.0};

  Point3 Apply(const Point3 & point) const
  {
    return Point3{
      static_cast<float>(r00 * point.x + r01 * point.y + r02 * point.z + tx),
      static_cast<float>(r10 * point.x + r11 * point.y + r12 * point.z + ty),
      static_cast<float>(r20 * point.x + r21 * point.y + r22 * point.z + tz)};
  }
};

RigidTransform ToRigidTransform(const geometry_msgs::msg::TransformStamped & tf)
{
  const auto & q_msg = tf.transform.rotation;
  const double norm =
    std::sqrt(q_msg.x * q_msg.x + q_msg.y * q_msg.y + q_msg.z * q_msg.z + q_msg.w * q_msg.w);
  const double x = norm > 1e-9 ? q_msg.x / norm : 0.0;
  const double y = norm > 1e-9 ? q_msg.y / norm : 0.0;
  const double z = norm > 1e-9 ? q_msg.z / norm : 0.0;
  const double w = norm > 1e-9 ? q_msg.w / norm : 1.0;

  RigidTransform result;
  result.tx = tf.transform.translation.x;
  result.ty = tf.transform.translation.y;
  result.tz = tf.transform.translation.z;
  result.r00 = 1.0 - 2.0 * (y * y + z * z);
  result.r01 = 2.0 * (x * y - z * w);
  result.r02 = 2.0 * (x * z + y * w);
  result.r10 = 2.0 * (x * y + z * w);
  result.r11 = 1.0 - 2.0 * (x * x + z * z);
  result.r12 = 2.0 * (y * z - x * w);
  result.r20 = 2.0 * (x * z - y * w);
  result.r21 = 2.0 * (y * z + x * w);
  result.r22 = 1.0 - 2.0 * (x * x + y * y);
  return result;
}

sensor_msgs::msg::PointCloud2 BuildPointCloud(
  const std::vector<Point3> & points,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = frame_id;
  cloud.header.stamp = stamp;
  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(points.size());
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = 3 * sizeof(float);
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.resize(3);

  const char *names[] = {"x", "y", "z"};
  for (size_t i = 0; i < cloud.fields.size(); ++i) {
    cloud.fields[i].name = names[i];
    cloud.fields[i].offset = static_cast<uint32_t>(i * sizeof(float));
    cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud.fields[i].count = 1;
  }

  cloud.data.resize(points.size() * cloud.point_step);
  for (size_t i = 0; i < points.size(); ++i) {
    const float values[3] = {points[i].x, points[i].y, points[i].z};
    std::memcpy(cloud.data.data() + i * cloud.point_step, values, sizeof(values));
  }
  return cloud;
}

double ReadDepthMeters(const sensor_msgs::msg::Image & image, size_t index)
{
  if (image.encoding == "16UC1" || image.encoding == "mono16") {
    const size_t byte_index = index * 2;
    if (byte_index + 1 >= image.data.size()) {
      return 0.0;
    }
    uint16_t raw = 0;
    if (image.is_bigendian) {
      raw = static_cast<uint16_t>((image.data[byte_index] << 8U) | image.data[byte_index + 1]);
    } else {
      raw = static_cast<uint16_t>(image.data[byte_index] | (image.data[byte_index + 1] << 8U));
    }
    return static_cast<double>(raw) * 0.001;
  }

  if (image.encoding == "32FC1") {
    const size_t byte_index = index * sizeof(float);
    if (byte_index + sizeof(float) > image.data.size()) {
      return 0.0;
    }
    float raw = 0.0F;
    std::memcpy(&raw, image.data.data() + byte_index, sizeof(float));
    return static_cast<double>(raw);
  }

  return 0.0;
}

}  // namespace

class OccupancyMapNode : public rclcpp::Node {
public:
  OccupancyMapNode()
  : Node("occupancy_map_node"),
    tf_buffer_(std::make_unique<tf2_ros::Buffer>(get_clock())),
    tf_listener_(std::make_unique<tf2_ros::TransformListener>(*tf_buffer_))
  {
    image_topic_ = declare_parameter<std::string>(
        "image_topic", "/OurCar/Sensors/DepthCamera/image_raw");
    camera_info_topic_ = declare_parameter<std::string>(
        "camera_info_topic", "/OurCar/Sensors/DepthCamera/camera_info");
    alternate_image_topic_ = declare_parameter<std::string>(
        "alternate_image_topic", "/Unity_ROS_message_Rx/OurCar/Sensors/DepthCamera/image_raw");
    alternate_camera_info_topic_ = declare_parameter<std::string>(
        "alternate_camera_info_topic",
      "/Unity_ROS_message_Rx/OurCar/Sensors/DepthCamera/camera_info");
    target_frame_ = declare_parameter<std::string>("target_frame", "world");
    use_latest_tf_ = declare_parameter<bool>("use_latest_tf", true);

    pointcloud_topic_ = declare_parameter<std::string>(
        "pointcloud_topic", "/perception/pcl/points");
    preprocessed_pointcloud_topic_ = declare_parameter<std::string>(
        "preprocessed_pointcloud_topic", "/perception/pcl/points_preprocessed");
    projected_map_topic_ = declare_parameter<std::string>(
        "projected_map_topic", "/perception/octomap/projected_map");
    extended_map_topic_ = declare_parameter<std::string>(
        "extended_map_topic", "/perception/extended_map");

    pixel_step_ = std::max(1, static_cast<int>(declare_parameter<int>("pixel_step", 4)));
    min_range_m_ = declare_parameter<double>("min_range_m", 0.25);
    max_range_m_ = declare_parameter<double>("max_range_m", 35.0);
    min_obstacle_z_ = declare_parameter<double>("min_obstacle_z", 0.15);
    max_obstacle_z_ = declare_parameter<double>("max_obstacle_z", 2.5);
    map_resolution_m_ = declare_parameter<double>("map_resolution_m", 0.25);
    map_width_m_ = declare_parameter<double>("map_width_m", 80.0);
    map_height_m_ = declare_parameter<double>("map_height_m", 80.0);
    inflation_radius_m_ = declare_parameter<double>("inflation_radius_m", 0.75);
    use_unknown_space_ = declare_parameter<bool>("use_unknown_space", false);

    pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic_, 5);
    preprocessed_pointcloud_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(preprocessed_pointcloud_topic_, 5);
    projected_map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(projected_map_topic_, 1);
    extended_map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(extended_map_topic_, 1);

    SubscribeCameraInfo(camera_info_topic_);
    SubscribeCameraInfo(alternate_camera_info_topic_);
    SubscribeDepthImage(image_topic_);
    SubscribeDepthImage(alternate_image_topic_);
    diagnostics_timer_ = create_wall_timer(
        std::chrono::seconds(3), std::bind(&OccupancyMapNode::PublishDiagnostics, this));

    RCLCPP_INFO(get_logger(), "Occupancy map node started.");
    RCLCPP_INFO(get_logger(), "Depth image topics: %s, %s",
                image_topic_.c_str(), alternate_image_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Camera info topics: %s, %s",
                camera_info_topic_.c_str(), alternate_camera_info_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Publishing projected map: %s", projected_map_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Publishing extended map: %s", extended_map_topic_.c_str());
  }

private:
  void SubscribeCameraInfo(const std::string & topic)
  {
    if (topic.empty() || HasTopic(camera_info_topics_, topic)) {
      return;
    }
    camera_info_topics_.push_back(topic);
    camera_info_subs_.push_back(create_subscription<sensor_msgs::msg::CameraInfo>(
        topic, rclcpp::SensorDataQoS(),
        std::bind(&OccupancyMapNode::OnCameraInfo, this, std::placeholders::_1)));
  }

  void SubscribeDepthImage(const std::string & topic)
  {
    if (topic.empty() || HasTopic(image_topics_, topic)) {
      return;
    }
    image_topics_.push_back(topic);
    image_subs_.push_back(create_subscription<sensor_msgs::msg::Image>(
        topic, rclcpp::SensorDataQoS(),
        std::bind(&OccupancyMapNode::OnDepthImage, this, std::placeholders::_1)));
  }

  static bool HasTopic(const std::vector<std::string> & topics, const std::string & topic)
  {
    return std::find(topics.begin(), topics.end(), topic) != topics.end();
  }

  void OnCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    latest_camera_info_ = msg;
    ++camera_info_count_;
  }

  void OnDepthImage(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    ++depth_image_count_;
    if (!latest_camera_info_) {
      last_status_ = "waiting for CameraInfo";
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Waiting for CameraInfo before generating occupancy map.");
      return;
    }

    if (msg->encoding != "16UC1" && msg->encoding != "mono16" && msg->encoding != "32FC1") {
      last_status_ = "unsupported depth encoding " + msg->encoding;
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Unsupported depth encoding '%s'. Supported encodings: 16UC1, mono16, 32FC1.",
          msg->encoding.c_str());
      return;
    }

    const auto transform = LookupTransform(*msg);
    if (!transform) {
      return;
    }

    const double fx = latest_camera_info_->k[0] > 1e-9 ? latest_camera_info_->k[0] :
      latest_camera_info_->p[0];
    const double fy = latest_camera_info_->k[4] > 1e-9 ? latest_camera_info_->k[4] :
      latest_camera_info_->p[5];
    const double cx = latest_camera_info_->k[2] > 1e-9 ? latest_camera_info_->k[2] :
      latest_camera_info_->p[2];
    const double cy = latest_camera_info_->k[5] > 1e-9 ? latest_camera_info_->k[5] :
      latest_camera_info_->p[6];
    if (fx <= 1e-9 || fy <= 1e-9) {
      last_status_ = "invalid camera intrinsics";
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Camera intrinsics are invalid.");
      return;
    }

    std::vector<Point3> camera_points;
    std::vector<Point3> obstacle_points;
    camera_points.reserve((msg->width / pixel_step_ + 1) * (msg->height / pixel_step_ + 1));
    obstacle_points.reserve(camera_points.capacity());

    for (uint32_t v = 0; v < msg->height; v += static_cast<uint32_t>(pixel_step_)) {
      for (uint32_t u = 0; u < msg->width; u += static_cast<uint32_t>(pixel_step_)) {
        const size_t index = static_cast<size_t>(v) * msg->width + u;
        const double depth_m = ReadDepthMeters(*msg, index);
        if (!std::isfinite(depth_m) || depth_m < min_range_m_ || depth_m > max_range_m_) {
          continue;
        }

        const Point3 camera_point{
          static_cast<float>((static_cast<double>(u) - cx) * depth_m / fx),
          static_cast<float>((static_cast<double>(v) - cy) * depth_m / fy),
          static_cast<float>(depth_m)};
        camera_points.push_back(camera_point);

        const Point3 world_point = transform->Apply(camera_point);
        if (world_point.z >= min_obstacle_z_ && world_point.z <= max_obstacle_z_) {
          obstacle_points.push_back(world_point);
        }
      }
    }

    const rclcpp::Time stamp(msg->header.stamp);
    pointcloud_pub_->publish(BuildPointCloud(camera_points, msg->header.frame_id, stamp));
    preprocessed_pointcloud_pub_->publish(BuildPointCloud(obstacle_points, target_frame_, stamp));
    PublishMaps(obstacle_points, *transform, msg->header.stamp);
    ++published_frames_;
    last_status_ = "published frame: raw_points=" + std::to_string(camera_points.size()) +
      ", obstacle_points=" + std::to_string(obstacle_points.size());
  }

  std::unique_ptr<RigidTransform> LookupTransform(const sensor_msgs::msg::Image & msg)
  {
    try {
      const rclcpp::Time lookup_time =
        use_latest_tf_ ? rclcpp::Time(0, 0, get_clock()->get_clock_type()) :
        rclcpp::Time(msg.header.stamp);
      const auto tf_msg =
        tf_buffer_->lookupTransform(target_frame_, msg.header.frame_id, lookup_time);
      return std::make_unique<RigidTransform>(ToRigidTransform(tf_msg));
    } catch (const tf2::TransformException & ex) {
      last_status_ = "TF lookup failed from " + msg.header.frame_id + " to " + target_frame_ +
        ": " + ex.what();
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Cannot transform depth camera frame '%s' to '%s': %s",
          msg.header.frame_id.c_str(), target_frame_.c_str(), ex.what());
      return nullptr;
    }
  }

  void PublishDiagnostics()
  {
    RCLCPP_INFO(
        get_logger(),
        "occupancy diagnostics: depth_images=%zu, camera_infos=%zu, "
        "published_frames=%zu, status=%s",
        depth_image_count_, camera_info_count_, published_frames_, last_status_.c_str());
  }

  void PublishMaps(
    const std::vector<Point3> & obstacle_points,
    const RigidTransform & camera_to_world,
    const builtin_interfaces::msg::Time & stamp)
  {
    const int width_cells = std::max(1,
      static_cast<int>(std::ceil(map_width_m_ / map_resolution_m_)));
    const int height_cells = std::max(1,
      static_cast<int>(std::ceil(map_height_m_ / map_resolution_m_)));
    const double origin_x = camera_to_world.tx - 0.5 * map_width_m_;
    const double origin_y = camera_to_world.ty - 0.5 * map_height_m_;

    nav_msgs::msg::OccupancyGrid projected_map = BuildEmptyMap(
        width_cells, height_cells, origin_x, origin_y, stamp);
    nav_msgs::msg::OccupancyGrid extended_map = projected_map;

    for (const auto & point : obstacle_points) {
      const int x_cell = static_cast<int>(std::floor((point.x - origin_x) / map_resolution_m_));
      const int y_cell = static_cast<int>(std::floor((point.y - origin_y) / map_resolution_m_));
      if (!IsInside(x_cell, y_cell, width_cells, height_cells)) {
        continue;
      }
      projected_map.data[ToIndex(x_cell, y_cell, width_cells)] = 100;
      MarkInflated(extended_map, x_cell, y_cell, width_cells, height_cells);
    }

    projected_map_pub_->publish(projected_map);
    extended_map_pub_->publish(extended_map);
  }

  nav_msgs::msg::OccupancyGrid BuildEmptyMap(
    int width_cells,
    int height_cells,
    double origin_x,
    double origin_y,
    const builtin_interfaces::msg::Time & stamp) const
  {
    nav_msgs::msg::OccupancyGrid map;
    map.header.frame_id = target_frame_;
    map.header.stamp = stamp;
    map.info.resolution = static_cast<float>(map_resolution_m_);
    map.info.width = static_cast<uint32_t>(width_cells);
    map.info.height = static_cast<uint32_t>(height_cells);
    map.info.origin.position.x = origin_x;
    map.info.origin.position.y = origin_y;
    map.info.origin.position.z = 0.0;
    map.info.origin.orientation.w = 1.0;
    map.data.assign(
        static_cast<size_t>(width_cells) * static_cast<size_t>(height_cells),
        use_unknown_space_ ? -1 : 0);
    return map;
  }

  void MarkInflated(
    nav_msgs::msg::OccupancyGrid & map,
    int x_cell,
    int y_cell,
    int width_cells,
    int height_cells) const
  {
    const int radius_cells =
      std::max(0, static_cast<int>(std::ceil(inflation_radius_m_ / map_resolution_m_)));
    const int radius_squared = radius_cells * radius_cells;
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        if (dx * dx + dy * dy > radius_squared) {
          continue;
        }
        const int nx = x_cell + dx;
        const int ny = y_cell + dy;
        if (IsInside(nx, ny, width_cells, height_cells)) {
          map.data[ToIndex(nx, ny, width_cells)] = 100;
        }
      }
    }
  }

  static bool IsInside(int x_cell, int y_cell, int width_cells, int height_cells)
  {
    return x_cell >= 0 && y_cell >= 0 && x_cell < width_cells && y_cell < height_cells;
  }

  static size_t ToIndex(int x_cell, int y_cell, int width_cells)
  {
    return static_cast<size_t>(y_cell) * static_cast<size_t>(width_cells) +
           static_cast<size_t>(x_cell);
  }

  std::string image_topic_;
  std::string camera_info_topic_;
  std::string alternate_image_topic_;
  std::string alternate_camera_info_topic_;
  std::string target_frame_;
  std::string pointcloud_topic_;
  std::string preprocessed_pointcloud_topic_;
  std::string projected_map_topic_;
  std::string extended_map_topic_;
  bool use_latest_tf_{true};
  int pixel_step_{4};
  double min_range_m_{0.25};
  double max_range_m_{35.0};
  double min_obstacle_z_{0.15};
  double max_obstacle_z_{2.5};
  double map_resolution_m_{0.25};
  double map_width_m_{80.0};
  double map_height_m_{80.0};
  double inflation_radius_m_{0.75};
  bool use_unknown_space_{false};

  sensor_msgs::msg::CameraInfo::SharedPtr latest_camera_info_;
  std::vector<std::string> camera_info_topics_;
  std::vector<std::string> image_topics_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> camera_info_subs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> image_subs_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr preprocessed_pointcloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr projected_map_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr extended_map_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  size_t depth_image_count_{0};
  size_t camera_info_count_{0};
  size_t published_frames_{0};
  std::string last_status_{"waiting for first depth image"};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OccupancyMapNode>());
  rclcpp::shutdown();
  return 0;
}
