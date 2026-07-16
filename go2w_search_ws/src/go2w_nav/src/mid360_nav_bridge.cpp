#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <stdexcept>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace
{

constexpr double kPi = 3.14159265358979323846;

using Matrix3 = std::array<std::array<double, 3>, 3>;
using Vector3 = std::array<double, 3>;

struct PointXYZ
{
  float x;
  float y;
  float z;
};

Matrix3 transpose(const Matrix3 & matrix)
{
  Matrix3 result{};
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t column = 0; column < 3; ++column) {
      result[row][column] = matrix[column][row];
    }
  }
  return result;
}

Matrix3 multiply(const Matrix3 & left, const Matrix3 & right)
{
  Matrix3 result{};
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t column = 0; column < 3; ++column) {
      for (std::size_t index = 0; index < 3; ++index) {
        result[row][column] += left[row][index] * right[index][column];
      }
    }
  }
  return result;
}

Vector3 multiply(const Matrix3 & matrix, const Vector3 & vector)
{
  Vector3 result{};
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t column = 0; column < 3; ++column) {
      result[row] += matrix[row][column] * vector[column];
    }
  }
  return result;
}

Matrix3 rpy_matrix(double roll, double pitch, double yaw)
{
  const double cr = std::cos(roll);
  const double sr = std::sin(roll);
  const double cp = std::cos(pitch);
  const double sp = std::sin(pitch);
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);
  return {{
    {{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr}},
    {{sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr}},
    {{-sp, cp * sr, cp * cr}},
  }};
}

sensor_msgs::msg::PointField point_field(
  const std::string & name, std::uint32_t offset)
{
  sensor_msgs::msg::PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field.count = 1;
  return field;
}

}  // namespace

class Mid360NavBridge final : public rclcpp::Node
{
public:
  Mid360NavBridge()
  : Node("mid360_nav_bridge")
  {
    const auto body_x = declare_parameter<double>("body_to_base_x", 0.0);
    const auto body_y = declare_parameter<double>("body_to_base_y", 0.0);
    const auto body_z = declare_parameter<double>("body_to_base_z", 0.0);
    const auto body_roll = declare_parameter<double>("body_to_base_roll", 0.0);
    const auto body_pitch = declare_parameter<double>(
      "body_to_base_pitch", -0.3490658504);
    const auto body_yaw = declare_parameter<double>("body_to_base_yaw", 0.0);
    input_xyz_scale_ = declare_parameter<double>("input_xyz_scale", 1.0);
    publish_hz_ = declare_parameter<double>("publish_hz", 10.0);
    range_min_ = declare_parameter<double>("range_min", 0.20);
    range_max_ = declare_parameter<double>("range_max", 8.0);
    min_height_ = declare_parameter<double>("min_height", -0.45);
    max_height_ = declare_parameter<double>("max_height", 1.50);
    bins_ = declare_parameter<int>("scan_bins", 360);
    scan_memory_sec_ = declare_parameter<double>("scan_memory_sec", 0.5);
    if (!std::isfinite(input_xyz_scale_) || input_xyz_scale_ <= 0.0 ||
      !std::isfinite(publish_hz_) || publish_hz_ <= 0.0 || bins_ <= 0 ||
      !std::isfinite(scan_memory_sec_) || scan_memory_sec_ <= 0.0)
    {
      throw std::invalid_argument("MID360 bridge parameters are invalid");
    }

    // Config stores body<-base. Incoming points require base<-body.
    const Matrix3 base_from_body = transpose(
      rpy_matrix(body_roll, body_pitch, body_yaw));
    raw_rotation_ = base_from_body;
    const Vector3 body_translation{{body_x, body_y, body_z}};
    const Vector3 rotated_body_translation = multiply(
      base_from_body, body_translation);
    for (std::size_t index = 0; index < 3; ++index) {
      raw_translation_[index] = -rotated_body_translation[index];
    }

    cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/mid360/points_nav", 10);
    scan_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
      "/scan_mid360", 10);

    auto latest_qos = rclcpp::SensorDataQoS();
    latest_qos.keep_last(1);
    latest_qos.best_effort();
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/cloud_registered_body", latest_qos,
      std::bind(&Mid360NavBridge::on_cloud, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / publish_hz_);
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&Mid360NavBridge::publish_latest, this));
    RCLCPP_INFO(
      get_logger(),
      "MID360 C++ latest-frame bridge: body cloud -> points + scan @ %.1fHz",
      publish_hz_);
  }

private:
  void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    if (message->header.frame_id != "body") {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "ignore body cloud in frame '%s'", message->header.frame_id.c_str());
      return;
    }
    const std::size_t count =
      static_cast<std::size_t>(message->width) * message->height;
    std::vector<PointXYZ> filtered;
    filtered.reserve(count);
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*message, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*message, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*message, "z");
    const auto end = iter_x.end();
    for (; iter_x != end; ++iter_x, ++iter_y, ++iter_z) {
      const Vector3 input{{
        static_cast<double>(*iter_x) * input_xyz_scale_,
        static_cast<double>(*iter_y) * input_xyz_scale_,
        static_cast<double>(*iter_z) * input_xyz_scale_,
      }};
      const auto rotated = multiply(raw_rotation_, input);
      const double x = rotated[0] + raw_translation_[0];
      const double y = rotated[1] + raw_translation_[1];
      const double z = rotated[2] + raw_translation_[2];
      const double radius = std::hypot(x, y);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
        radius < range_min_ || radius > range_max_ ||
        z < min_height_ || z > max_height_)
      {
        continue;
      }
      const bool self_return =
        x >= -0.45 && x <= 0.45 && y >= -0.32 && y <= 0.32;
      if (!self_return) {
        filtered.push_back(PointXYZ{
          static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)});
      }
    }
    std::lock_guard<std::mutex> guard(latest_mutex_);
    latest_points_ = std::move(filtered);
    latest_ready_ = true;
  }

  void publish_latest()
  {
    std::vector<PointXYZ> points;
    {
      std::lock_guard<std::mutex> guard(latest_mutex_);
      if (!latest_ready_) {
        return;
      }
      points = latest_points_;
      latest_ready_ = false;
    }

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = "base_link";
    cloud.height = 1;
    cloud.width = static_cast<std::uint32_t>(points.size());
    cloud.fields = {
      point_field("x", 0), point_field("y", 4), point_field("z", 8)};
    cloud.is_bigendian = false;
    cloud.point_step = 12;
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.is_dense = true;
    cloud.data.resize(points.size() * cloud.point_step);
    for (std::size_t index = 0; index < points.size(); ++index) {
      std::memcpy(cloud.data.data() + index * 12, &points[index].x, 4);
      std::memcpy(cloud.data.data() + index * 12 + 4, &points[index].y, 4);
      std::memcpy(cloud.data.data() + index * 12 + 8, &points[index].z, 4);
    }
    cloud.header.stamp = now();
    cloud_publisher_->publish(cloud);

    std::vector<float> current(
      static_cast<std::size_t>(bins_), std::numeric_limits<float>::infinity());
    for (const auto & point : points) {
      const double radius = std::hypot(point.x, point.y);
      if (radius < range_min_ || radius > range_max_) {
        continue;
      }
      const double angle = std::atan2(point.y, point.x);
      auto bin = static_cast<int>(std::floor(
        (angle + kPi) * static_cast<double>(bins_) / (2.0 * kPi)));
      bin %= bins_;
      if (bin < 0) {
        bin += bins_;
      }
      current[static_cast<std::size_t>(bin)] = std::min(
        current[static_cast<std::size_t>(bin)], static_cast<float>(radius));
    }

    const auto steady_now = std::chrono::steady_clock::now();
    scan_history_.emplace_back(steady_now, std::move(current));
    const auto memory = std::chrono::duration<double>(scan_memory_sec_);
    while (!scan_history_.empty() &&
      steady_now - scan_history_.front().first > memory)
    {
      scan_history_.pop_front();
    }
    std::vector<float> merged(
      static_cast<std::size_t>(bins_), std::numeric_limits<float>::infinity());
    for (const auto & entry : scan_history_) {
      for (std::size_t index = 0; index < merged.size(); ++index) {
        merged[index] = std::min(merged[index], entry.second[index]);
      }
    }

    sensor_msgs::msg::LaserScan scan;
    scan.header.frame_id = "base_link";
    scan.header.stamp = now();
    scan.angle_min = static_cast<float>(-kPi);
    scan.angle_increment = static_cast<float>(2.0 * kPi / bins_);
    scan.angle_max = scan.angle_min +
      static_cast<float>(bins_ - 1) * scan.angle_increment;
    scan.scan_time = static_cast<float>(1.0 / publish_hz_);
    scan.time_increment = 0.0F;
    scan.range_min = static_cast<float>(range_min_);
    scan.range_max = static_cast<float>(range_max_);
    scan.ranges = std::move(merged);
    scan_publisher_->publish(scan);

    if (!first_publish_logged_) {
      first_publish_logged_ = true;
      RCLCPP_INFO(
        get_logger(), "first C++ MID360 nav frame published (%zu points)",
        points.size());
    }
  }

  Matrix3 raw_rotation_{};
  Vector3 raw_translation_{};
  double input_xyz_scale_{1.0};
  double publish_hz_{10.0};
  double range_min_{0.2};
  double range_max_{8.0};
  double min_height_{-0.45};
  double max_height_{1.5};
  int bins_{360};
  double scan_memory_sec_{0.5};

  std::mutex latest_mutex_;
  std::vector<PointXYZ> latest_points_;
  bool latest_ready_{false};
  bool first_publish_logged_{false};
  std::deque<std::pair<
    std::chrono::steady_clock::time_point, std::vector<float>>> scan_history_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
    cloud_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Mid360NavBridge>());
  rclcpp::shutdown();
  return 0;
}
