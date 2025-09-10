#ifndef ROS2_ELEVATION_MAPPING_INCLUDE_ELEVATION_MAPPING_H_
#define ROS2_ELEVATION_MAPPING_INCLUDE_ELEVATION_MAPPING_H_

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include "grid_map_costmap_2d/grid_map_costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cmath>

namespace elevation_mapping {
class ElevationMapping {
  public:
    explicit ElevationMapping(rclcpp::Node::SharedPtr node, const std::string &ns = "",
                          std::vector<rclcpp::Parameter> parameters = {});
  private:
    rclcpp::Node::SharedPtr node_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr gridmap_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr feature_pub_;

    grid_map::GridMap gridmap_;
    nav2_costmap_2d::Costmap2D costmap_;
    grid_map::Costmap2DConverter<grid_map::GridMap, grid_map::Costmap2DTranslationTable<50, 100, 99, 80>> converter_; //todo: set proper params

    /*
     * Create an empty grid map with a specified geometry.
     */
    void init_gridmap();

    /*
     * Callback for the point cloud topic. Processes the point cloud and updates the grid map.
     */
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
};
} // namespace elevation_mapping

#endif  // ROS2_ELEVATION_MAPPING_INCLUDE_ELEVATION_MAPPING_H_