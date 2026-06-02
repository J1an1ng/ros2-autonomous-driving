#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace
{
constexpr uint32_t kMagic = 0xDEADC0DEu;
constexpr uint32_t kUnityCameraType = 1u;
constexpr size_t kBytesPerPixelRgb = 3u;

class SocketHandle
{
public:
  SocketHandle() = default;
  explicit SocketHandle(int fd) : fd_(fd) {}
  ~SocketHandle() { reset(); }

  SocketHandle(const SocketHandle &) = delete;
  SocketHandle & operator=(const SocketHandle &) = delete;

  SocketHandle(SocketHandle && other) noexcept : fd_(other.fd_) { other.fd_ = -1; }
  SocketHandle & operator=(SocketHandle && other) noexcept
  {
    if (this != &other) {
      reset();
      fd_ = other.fd_;
      other.fd_ = -1;
    }
    return *this;
  }

  int get() const { return fd_; }
  bool valid() const { return fd_ >= 0; }
  void reset(int new_fd = -1)
  {
    if (fd_ >= 0) {
      ::close(fd_);
    }
    fd_ = new_fd;
  }

private:
  int fd_{-1};
};

void read_exact(int fd, void * dst, size_t num_bytes)
{
  auto * out = static_cast<uint8_t *>(dst);
  size_t done = 0;
  while (done < num_bytes) {
    const ssize_t n = ::recv(fd, out + done, num_bytes - done, 0);
    if (n == 0) {
      throw std::runtime_error("Unity TCP connection closed");
    }
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      throw std::runtime_error(std::string("recv failed: ") + std::strerror(errno));
    }
    done += static_cast<size_t>(n);
  }
}

template<typename T>
T read_pod(int fd)
{
  T value{};
  read_exact(fd, &value, sizeof(T));
  return value;
}

std::string read_c_string(int fd)
{
  std::string out;
  char c = '\0';
  do {
    read_exact(fd, &c, 1);
    if (c != '\0') {
      out.push_back(c);
    }
  } while (c != '\0');
  return out;
}

std::string sanitize_relative_topic(std::string name)
{
  std::replace(name.begin(), name.end(), ' ', '_');
  while (!name.empty() && name.front() == '/') {
    name.erase(name.begin());
  }
  return name;
}

std::vector<uint8_t> flip_rgb_vertical(const std::vector<uint8_t> & in, int width, int height)
{
  const size_t stride = static_cast<size_t>(width) * kBytesPerPixelRgb;
  std::vector<uint8_t> out(in.size());
  for (int row = 0; row < height; ++row) {
    const auto src_offset = static_cast<size_t>(height - row - 1) * stride;
    const auto dst_offset = static_cast<size_t>(row) * stride;
    std::copy_n(in.data() + src_offset, stride, out.data() + dst_offset);
  }
  return out;
}

std::vector<uint8_t> rgb_to_mono8(const std::vector<uint8_t> & rgb, int width, int height)
{
  std::vector<uint8_t> mono(static_cast<size_t>(width) * static_cast<size_t>(height));
  for (size_t i = 0, j = 0; i < rgb.size(); i += 3, ++j) {
    const float r = static_cast<float>(rgb[i + 0]);
    const float g = static_cast<float>(rgb[i + 1]);
    const float b = static_cast<float>(rgb[i + 2]);
    mono[j] = static_cast<uint8_t>(std::clamp(0.299f * r + 0.587f * g + 0.114f * b, 0.0f, 255.0f));
  }
  return mono;
}

builtin_interfaces::msg::Time stamp_from_unity_ticks(uint64_t ticks_100ns_since_unix_epoch)
{
  const uint64_t ns = ticks_100ns_since_unix_epoch * 100ull;
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(ns / 1000000000ull);
  stamp.nanosec = static_cast<uint32_t>(ns % 1000000000ull);
  return stamp;
}
}  // namespace

class UnityRgbBridgeNode : public rclcpp::Node
{
public:
  UnityRgbBridgeNode() : Node("unity_rgb_bridge")
  {
    bind_host_ = declare_parameter<std::string>("bind_host", "127.0.0.1");
    port_ = declare_parameter<int>("port", 9998);
    flip_vertical_ = declare_parameter<bool>("flip_vertical", true);
    publish_camera_info_ = declare_parameter<bool>("publish_camera_info", true);
    topic_prefix_ = declare_parameter<std::string>("topic_prefix", "");
  }

  void run()
  {
    server_ = make_server_socket(bind_host_, port_);
    RCLCPP_INFO(get_logger(), "Waiting for Unity TCP connection on %s:%d", bind_host_.c_str(), port_);

    while (rclcpp::ok()) {
      SocketHandle client(::accept(server_.get(), nullptr, nullptr));
      if (!client.valid()) {
        if (errno == EINTR) {
          continue;
        }
        RCLCPP_ERROR(get_logger(), "accept failed: %s", std::strerror(errno));
        continue;
      }

      RCLCPP_INFO(get_logger(), "Unity connected");
      try {
        read_loop(client.get());
      } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(), "Connection ended: %s", e.what());
      }
    }
  }

private:
  struct Publishers
  {
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info;
  };

  SocketHandle make_server_socket(const std::string & host, int port)
  {
    SocketHandle fd(::socket(AF_INET, SOCK_STREAM, 0));
    if (!fd.valid()) {
      throw std::runtime_error(std::string("socket failed: ") + std::strerror(errno));
    }

    int reuse = 1;
    ::setsockopt(fd.get(), SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (::inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
      throw std::runtime_error("bind_host must be an IPv4 address, e.g. 127.0.0.1 or 0.0.0.0");
    }

    if (::bind(fd.get(), reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
      throw std::runtime_error(std::string("bind failed: ") + std::strerror(errno));
    }
    if (::listen(fd.get(), 1) != 0) {
      throw std::runtime_error(std::string("listen failed: ") + std::strerror(errno));
    }
    return fd;
  }

  void read_loop(int client_fd)
  {
    while (rclcpp::ok()) {
      const uint32_t magic = read_pod<uint32_t>(client_fd);
      if (magic != kMagic) {
        throw std::runtime_error("stream corrupted: wrong magic number");
      }

      const uint32_t type = read_pod<uint32_t>(client_fd);
      const uint64_t timestamp_ticks = read_pod<uint64_t>(client_fd);
      const std::string raw_name = read_c_string(client_fd);

      if (type == kUnityCameraType) {
        read_and_publish_rgb_camera(client_fd, raw_name, timestamp_ticks);
      } else {
        throw std::runtime_error("received non-camera message type; start only RGBCamera.cs or extend the parser");
      }

      rclcpp::spin_some(shared_from_this());
    }
  }

  Publishers & publishers_for(const std::string & raw_name)
  {
    const std::string base = topic_prefix_ + sanitize_relative_topic(raw_name);
    auto it = publishers_.find(base);
    if (it != publishers_.end()) {
      return it->second;
    }

    Publishers pubs;
    pubs.image = create_publisher<sensor_msgs::msg::Image>(base + "/image_raw", rclcpp::SensorDataQoS());
    if (publish_camera_info_) {
      pubs.info = create_publisher<sensor_msgs::msg::CameraInfo>(base + "/camera_info", rclcpp::QoS(10));
    }
    auto [inserted_it, _] = publishers_.emplace(base, pubs);
    RCLCPP_INFO(get_logger(), "Publishing Unity RGB camera on '%s/image_raw'", base.c_str());
    return inserted_it->second;
  }

  void read_and_publish_rgb_camera(int fd, const std::string & raw_name, uint64_t timestamp_ticks)
  {
    const bool monochrome = read_pod<int32_t>(fd) == 1;
    const float fov_deg = read_pod<float>(fd);
    [[maybe_unused]] const float far_clip_plane = read_pod<float>(fd);
    const int32_t width = read_pod<int32_t>(fd);
    const int32_t height = read_pod<int32_t>(fd);

    if (width <= 0 || height <= 0) {
      throw std::runtime_error("invalid image size");
    }

    const size_t rgb_size = static_cast<size_t>(width) * static_cast<size_t>(height) * kBytesPerPixelRgb;
    std::vector<uint8_t> rgb(rgb_size);
    read_exact(fd, rgb.data(), rgb.size());

    if (flip_vertical_) {
      rgb = flip_rgb_vertical(rgb, width, height);
    }

    const auto stamp = stamp_from_unity_ticks(timestamp_ticks);
    const std::string frame_id = sanitize_relative_topic(raw_name);
    auto & pubs = publishers_for(raw_name);

    sensor_msgs::msg::Image image_msg;
    image_msg.header.stamp = stamp;
    image_msg.header.frame_id = frame_id;
    image_msg.width = static_cast<uint32_t>(width);
    image_msg.height = static_cast<uint32_t>(height);
    image_msg.is_bigendian = false;

    if (monochrome) {
      image_msg.encoding = "mono8";
      image_msg.step = static_cast<uint32_t>(width);
      image_msg.data = rgb_to_mono8(rgb, width, height);
    } else {
      image_msg.encoding = "rgb8";
      image_msg.step = static_cast<uint32_t>(width) * 3u;
      image_msg.data = std::move(rgb);
    }

    pubs.image->publish(image_msg);

    if (publish_camera_info_ && pubs.info) {
      sensor_msgs::msg::CameraInfo info_msg;
      fill_camera_info(image_msg, fov_deg, info_msg);
      pubs.info->publish(info_msg);
    }
  }

  static void fill_camera_info(
    const sensor_msgs::msg::Image & image_msg,
    float vertical_fov_deg,
    sensor_msgs::msg::CameraInfo & info_msg)
  {
    info_msg.header = image_msg.header;
    info_msg.width = image_msg.width;
    info_msg.height = image_msg.height;
    info_msg.distortion_model = "plumb_bob";
    info_msg.d = {0.0, 0.0, 0.0, 0.0, 0.0};

    const double cx = static_cast<double>(image_msg.width) * 0.5;
    const double cy = static_cast<double>(image_msg.height) * 0.5;
    const double fy = 0.5 * static_cast<double>(image_msg.height) /
      std::tan(static_cast<double>(vertical_fov_deg) * M_PI / 360.0);
    const double fx = fy;

    info_msg.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
    info_msg.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    info_msg.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
  }

  std::string bind_host_;
  int port_{};
  bool flip_vertical_{};
  bool publish_camera_info_{};
  std::string topic_prefix_;
  SocketHandle server_;
  std::map<std::string, Publishers> publishers_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UnityRgbBridgeNode>();
  try {
    node->run();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node->get_logger(), "%s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
