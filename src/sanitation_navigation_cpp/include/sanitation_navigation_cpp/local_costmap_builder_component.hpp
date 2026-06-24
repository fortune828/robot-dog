#ifndef SANITATION_NAVIGATION_CPP__LOCAL_COSTMAP_BUILDER_COMPONENT_HPP_
#define SANITATION_NAVIGATION_CPP__LOCAL_COSTMAP_BUILDER_COMPONENT_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace sanitation_navigation_cpp
{

class LocalCostmapBuilderComponent : public rclcpp::Node
{
public:
  explicit LocalCostmapBuilderComponent(const rclcpp::NodeOptions & options);

private:
  struct Params
  {
    std::string cloud_topic;
    std::string grid_topic;
    std::string debug_image_topic;
    std::string markers_topic;
    std::string frame_id;
    double x_min{};
    double x_max{};
    double y_min{};
    double y_max{};
    double resolution{};
    double min_obstacle_height{};
    double max_obstacle_height{};
    int min_points_per_cell{};
    double robot_radius{};
    double safety_margin{};
    double inflation_radius{};
    int log_interval{};
  };

  struct FieldOffsets
  {
    uint32_t x{};
    uint32_t y{};
    uint32_t z{};
  };

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  bool readFieldOffsets(const sensor_msgs::msg::PointCloud2 & msg, FieldOffsets & offsets);
  void inflateOccupied(const std::vector<uint8_t> & occupied, std::vector<uint8_t> & inflated);
  void publishGrid(
    const sensor_msgs::msg::PointCloud2 & source, const std::vector<uint8_t> & inflated);
  void publishDebug(
    const sensor_msgs::msg::PointCloud2 & source, const std::vector<uint8_t> & occupied,
    const std::vector<uint8_t> & inflated);
  void publishMarkers(
    const sensor_msgs::msg::PointCloud2 & source, const std::vector<uint8_t> & occupied,
    const std::vector<uint8_t> & inflated);

  Params params_{};
  int width_{};
  int height_{};
  int inflation_cells_{};
  int seq_{};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

}  // namespace sanitation_navigation_cpp

#endif  // SANITATION_NAVIGATION_CPP__LOCAL_COSTMAP_BUILDER_COMPONENT_HPP_
