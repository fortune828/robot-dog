#include "sanitation_navigation_cpp/local_costmap_builder_component.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace sanitation_navigation_cpp
{
namespace
{

float readFloat32(const std::vector<uint8_t> & data, size_t offset)
{
  float value = std::numeric_limits<float>::quiet_NaN();
  std::memcpy(&value, data.data() + offset, sizeof(float));
  return value;
}

}  // namespace

LocalCostmapBuilderComponent::LocalCostmapBuilderComponent(const rclcpp::NodeOptions & options)
: Node("local_costmap_builder_node", options)
{
  params_.cloud_topic = declare_parameter<std::string>("cloud_topic", "/depth_anything/points_filtered");
  params_.grid_topic = declare_parameter<std::string>("grid_topic", "/local_occupancy_grid");
  params_.debug_image_topic =
    declare_parameter<std::string>("debug_image_topic", "/local_costmap_debug_image");
  params_.markers_topic = declare_parameter<std::string>("markers_topic", "/local_obstacle_markers");
  params_.frame_id = declare_parameter<std::string>("frame_id", "base_link");
  params_.x_min = declare_parameter<double>("x_min", 0.0);
  params_.x_max = declare_parameter<double>("x_max", 12.0);
  params_.y_min = declare_parameter<double>("y_min", -5.0);
  params_.y_max = declare_parameter<double>("y_max", 5.0);
  params_.resolution = declare_parameter<double>("resolution", 0.1);
  params_.min_obstacle_height = declare_parameter<double>("min_obstacle_height", 0.08);
  params_.max_obstacle_height = declare_parameter<double>("max_obstacle_height", 1.5);
  params_.min_points_per_cell = declare_parameter<int>("min_points_per_cell", 1);
  params_.robot_radius = declare_parameter<double>("robot_radius", 0.35);
  params_.safety_margin = declare_parameter<double>("safety_margin", 0.35);
  params_.inflation_radius = declare_parameter<double>("inflation_radius", 0.7);
  params_.log_interval = declare_parameter<int>("log_interval", 10);

  if (
    params_.resolution <= 0.0 || params_.x_max <= params_.x_min ||
    params_.y_max <= params_.y_min) {
    throw std::runtime_error("invalid local costmap bounds or resolution");
  }

  width_ = static_cast<int>(std::ceil((params_.x_max - params_.x_min) / params_.resolution));
  height_ = static_cast<int>(std::ceil((params_.y_max - params_.y_min) / params_.resolution));
  const double inflation_radius = params_.inflation_radius >= 0.0 ?
    params_.inflation_radius : params_.robot_radius + params_.safety_margin;
  inflation_cells_ = static_cast<int>(std::ceil(inflation_radius / params_.resolution));

  grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(params_.grid_topic, 10);
  debug_pub_ = create_publisher<sensor_msgs::msg::Image>(params_.debug_image_topic, 10);
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(params_.markers_topic, 10);
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    params_.cloud_topic, 10,
    std::bind(&LocalCostmapBuilderComponent::onCloud, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(), "C++ local costmap ready | %s -> %s | %dx%d",
    params_.cloud_topic.c_str(), params_.grid_topic.c_str(), width_, height_);
}

bool LocalCostmapBuilderComponent::readFieldOffsets(
  const sensor_msgs::msg::PointCloud2 & msg, FieldOffsets & offsets)
{
  bool has_x = false;
  bool has_y = false;
  bool has_z = false;
  for (const auto & field : msg.fields) {
    if (field.datatype != sensor_msgs::msg::PointField::FLOAT32) {
      continue;
    }
    if (field.name == "x") {
      offsets.x = field.offset;
      has_x = true;
    } else if (field.name == "y") {
      offsets.y = field.offset;
      has_y = true;
    } else if (field.name == "z") {
      offsets.z = field.offset;
      has_z = true;
    }
  }
  return has_x && has_y && has_z;
}

void LocalCostmapBuilderComponent::onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  if (!msg->header.frame_id.empty() && msg->header.frame_id != params_.frame_id) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Expected cloud frame %s, got %s; skipping",
      params_.frame_id.c_str(), msg->header.frame_id.c_str());
    return;
  }
  if (msg->is_bigendian || msg->point_step < 12) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Unsupported PointCloud2 layout; expected little-endian float32 XYZ");
    return;
  }
  FieldOffsets offsets;
  if (!readFieldOffsets(*msg, offsets)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Unsupported PointCloud2 fields; expected float32 x/y/z");
    return;
  }

  const size_t count = static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
  if (msg->data.size() < count * msg->point_step) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "PointCloud2 data is shorter than its declared dimensions");
    return;
  }

  std::vector<int> counts(static_cast<size_t>(width_) * static_cast<size_t>(height_), 0);
  size_t valid_count = 0;
  for (size_t i = 0; i < count; ++i) {
    const size_t base = i * msg->point_step;
    const float x = readFloat32(msg->data, base + offsets.x);
    const float y = readFloat32(msg->data, base + offsets.y);
    const float z = readFloat32(msg->data, base + offsets.z);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }
    if (z < params_.min_obstacle_height || z > params_.max_obstacle_height) {
      continue;
    }
    if (x < params_.x_min || x >= params_.x_max || y < params_.y_min || y >= params_.y_max) {
      continue;
    }
    const int col = static_cast<int>((x - params_.x_min) / params_.resolution);
    const int row = static_cast<int>((y - params_.y_min) / params_.resolution);
    if (row < 0 || row >= height_ || col < 0 || col >= width_) {
      continue;
    }
    ++counts[static_cast<size_t>(row) * static_cast<size_t>(width_) + static_cast<size_t>(col)];
    ++valid_count;
  }

  std::vector<uint8_t> occupied(counts.size(), 0);
  const int min_points = std::max(1, params_.min_points_per_cell);
  for (size_t i = 0; i < counts.size(); ++i) {
    occupied[i] = counts[i] >= min_points ? 1U : 0U;
  }

  std::vector<uint8_t> inflated;
  inflateOccupied(occupied, inflated);
  publishGrid(*msg, inflated);
  publishDebug(*msg, occupied, inflated);
  publishMarkers(*msg, occupied, inflated);

  ++seq_;
  if (params_.log_interval > 0 && seq_ % params_.log_interval == 0) {
    const auto occupied_count = std::count(occupied.begin(), occupied.end(), 1U);
    const auto inflated_count = std::count(inflated.begin(), inflated.end(), 1U);
    RCLCPP_INFO(
      get_logger(), "costmap frame#%d: valid_points=%zu occupied=%ld inflated=%ld",
      seq_, valid_count, occupied_count, inflated_count);
  }
}

void LocalCostmapBuilderComponent::inflateOccupied(
  const std::vector<uint8_t> & occupied, std::vector<uint8_t> & inflated)
{
  inflated = occupied;
  if (inflation_cells_ <= 0) {
    return;
  }
  for (int dy = -inflation_cells_; dy <= inflation_cells_; ++dy) {
    for (int dx = -inflation_cells_; dx <= inflation_cells_; ++dx) {
      if (dx * dx + dy * dy > inflation_cells_ * inflation_cells_) {
        continue;
      }
      for (int row = 0; row < height_; ++row) {
        const int src_row = row - dy;
        if (src_row < 0 || src_row >= height_) {
          continue;
        }
        for (int col = 0; col < width_; ++col) {
          const int src_col = col - dx;
          if (src_col < 0 || src_col >= width_) {
            continue;
          }
          const size_t src = static_cast<size_t>(src_row) * width_ + static_cast<size_t>(src_col);
          if (!occupied[src]) {
            continue;
          }
          const size_t dst = static_cast<size_t>(row) * width_ + static_cast<size_t>(col);
          inflated[dst] = 1U;
        }
      }
    }
  }
}

void LocalCostmapBuilderComponent::publishGrid(
  const sensor_msgs::msg::PointCloud2 & source, const std::vector<uint8_t> & inflated)
{
  nav_msgs::msg::OccupancyGrid msg;
  msg.header.stamp = source.header.stamp;
  msg.header.frame_id = params_.frame_id;
  msg.info.resolution = static_cast<float>(params_.resolution);
  msg.info.width = static_cast<uint32_t>(width_);
  msg.info.height = static_cast<uint32_t>(height_);
  msg.info.origin.position.x = params_.x_min;
  msg.info.origin.position.y = params_.y_min;
  msg.info.origin.orientation.w = 1.0;
  msg.data.resize(inflated.size());
  for (size_t i = 0; i < inflated.size(); ++i) {
    msg.data[i] = inflated[i] ? 100 : 0;
  }
  grid_pub_->publish(std::move(msg));
}

void LocalCostmapBuilderComponent::publishDebug(
  const sensor_msgs::msg::PointCloud2 & source, const std::vector<uint8_t> & occupied,
  const std::vector<uint8_t> & inflated)
{
  sensor_msgs::msg::Image msg;
  msg.header.stamp = source.header.stamp;
  msg.header.frame_id = params_.frame_id;
  msg.height = static_cast<uint32_t>(height_);
  msg.width = static_cast<uint32_t>(width_);
  msg.encoding = "rgb8";
  msg.step = static_cast<uint32_t>(width_ * 3);
  msg.data.assign(static_cast<size_t>(height_) * static_cast<size_t>(width_) * 3U, 0U);
  for (int row = 0; row < height_; ++row) {
    const int flipped_row = height_ - 1 - row;
    for (int col = 0; col < width_; ++col) {
      const size_t grid_index = static_cast<size_t>(row) * width_ + static_cast<size_t>(col);
      const size_t image_index =
        (static_cast<size_t>(flipped_row) * width_ + static_cast<size_t>(col)) * 3U;
      if (inflated[grid_index]) {
        msg.data[image_index + 0U] = 80U;
        msg.data[image_index + 1U] = 80U;
        msg.data[image_index + 2U] = 80U;
      }
      if (occupied[grid_index]) {
        msg.data[image_index + 0U] = 255U;
        msg.data[image_index + 1U] = 255U;
        msg.data[image_index + 2U] = 255U;
      }
    }
  }
  debug_pub_->publish(std::move(msg));
}

void LocalCostmapBuilderComponent::publishMarkers(
  const sensor_msgs::msg::PointCloud2 & source, const std::vector<uint8_t> & occupied,
  const std::vector<uint8_t> & inflated)
{
  visualization_msgs::msg::MarkerArray result;
  for (int layer = 0; layer < 2; ++layer) {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = source.header.stamp;
    marker.header.frame_id = params_.frame_id;
    marker.ns = "local_obstacles";
    marker.id = layer;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = params_.resolution;
    marker.scale.y = params_.resolution;
    marker.scale.z = layer == 0 ? 0.02 : 0.06;
    marker.color.a = 0.65;
    if (layer == 0) {
      marker.color.r = 1.0;
      marker.color.g = 0.7;
      marker.color.b = 0.0;
    } else {
      marker.color.r = 1.0;
      marker.color.g = 0.1;
      marker.color.b = 0.1;
    }
    for (int row = 0; row < height_; ++row) {
      for (int col = 0; col < width_; ++col) {
        const size_t index = static_cast<size_t>(row) * width_ + static_cast<size_t>(col);
        const bool include = layer == 0 ? (inflated[index] && !occupied[index]) : occupied[index];
        if (!include) {
          continue;
        }
        geometry_msgs::msg::Point point;
        point.x = params_.x_min + (static_cast<double>(col) + 0.5) * params_.resolution;
        point.y = params_.y_min + (static_cast<double>(row) + 0.5) * params_.resolution;
        point.z = marker.scale.z * 0.5;
        marker.points.push_back(point);
      }
    }
    result.markers.push_back(std::move(marker));
  }
  marker_pub_->publish(std::move(result));
}

}  // namespace sanitation_navigation_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(sanitation_navigation_cpp::LocalCostmapBuilderComponent)
