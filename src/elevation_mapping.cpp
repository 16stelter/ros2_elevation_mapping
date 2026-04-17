#include "ros2_elevation_mapping/elevation_mapping.hpp"

namespace elevation_mapping {

ElevationMapping::ElevationMapping(rclcpp::Node::SharedPtr node, 
                           const std::string &ns,
                           std::vector<rclcpp::Parameter> parameters)
: node_(node),
  tf_buffer_(node->get_clock()),
  tf_listener_(tf_buffer_),
  filterChain_("grid_map::GridMap")
{

  node_->declare_parameter("pointcloud_topic", "");
  node_->declare_parameter("map_topic", "");

  map_frame_ = node_->declare_parameter("map_frame", "");

  pointcloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    node_->get_parameter("pointcloud_topic").as_string(),
    rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
    std::bind(&ElevationMapping::pointcloud_callback, this, std::placeholders::_1)
  );
  gridmap_pub_ = node_->create_publisher<grid_map_msgs::msg::GridMap>(node_->get_parameter("map_topic").as_string(), 10);
  init_gridmap();
}

void ElevationMapping::init_gridmap()
{
  gridmap_ = grid_map::GridMap({"elevation", "variance"});
  gridmap_.setFrameId(map_frame_);
  gridmap_.setGeometry(grid_map::Length(5.0, 5.0), 0.05);
  gridmap_pub_->publish(grid_map::GridMapRosConverter::toMessage(gridmap_));
  alpha_ = 0.2; //todo: make parameter
  RCLCPP_INFO(node_->get_logger(), "Initialized grid map with size: (%i, %i)", gridmap_.getSize()(0), gridmap_.getSize()(1));
  if (filterChain_.configure(
      "filters", node_->get_node_logging_interface(),
      node_->get_node_parameters_interface()))
  {
    RCLCPP_INFO(node_->get_logger(), "Filter chain configured.");
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Could not configure the filter chain!");
    rclcpp::shutdown();
    return;
  }

}

void ElevationMapping::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  sensor_msgs::msg::PointCloud2 cloud_transformed;
  try {
    geometry_msgs::msg::TransformStamped transform =
    tf_buffer_.lookupTransform(map_frame_, msg->header.frame_id, msg->header.stamp,
                               rclcpp::Duration::from_seconds(0.1));
    tf2::doTransform(*msg, cloud_transformed, transform);
  } catch (tf2::TransformException &e) {
    RCLCPP_WARN(node_->get_logger(), "Transform failed: %s", e.what());
    return;
  }

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_transformed, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_transformed, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_transformed, "z");
  std::vector<double> dims(4, 0.0);
  std::map<std::pair<int, int>, std::vector<float>> cell_points;
  
  for(; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {

    grid_map::Position p(*iter_x, *iter_y);
    if (!std::isinf(p[0]) && !std::isinf(p[1])) {
      if (!gridmap_.isInside(p)) {
        dims[0] = std::min(dims[0], p[0]);
        dims[1] = std::min(dims[1], p[1]);
        dims[2] = std::max(dims[2], p[0]);
        dims[3] = std::max(dims[3], p[1]);
      }

      grid_map::Index i;
      if(gridmap_.getIndex(grid_map::Position(*iter_x, *iter_y), i)) {
        cell_points[{i[0], i[1]}].push_back(*iter_z);
      }
    }
  }

  if(std::any_of(dims.begin(), dims.end(), [](double val) { return val != 0.0; }))
  {
    dims = {dims[0] - 0.05, dims[1] - 0.05, dims[2] + 0.05, dims[3] + 0.05};
    grid_map::GridMap new_grid;
    grid_map::Length new_length = grid_map::Length((dims[2] - dims[0]), (dims[3] - dims[1]));
    grid_map::Position new_origin((dims[0] + dims[2]) / 2, (dims[1] + dims[3]) / 2);
    new_grid.setGeometry(new_length, 0.05, new_origin);
    gridmap_.extendToInclude(new_grid);
  }

  for(auto &c : cell_points)
  {
    auto i = c.first;
    auto &vs = c.second;
    if (vs.size() > 0) {
      float avg_z = std::accumulate(vs.begin(), vs.end(), 0.0f) / vs.size();
      grid_map::Index idx;
      idx[0] = i.first;
      idx[1] = i.second;
      if (!gridmap_.isValid(idx, "elevation")) {
        gridmap_.at("elevation", idx) = avg_z;
      }
      else {
        float last_z = gridmap_.at("elevation", idx);
        gridmap_.at("elevation", idx) = (1.0 - alpha_) * last_z + alpha_ * avg_z;
      }

    }
  }

  grid_map::GridMap filtered_gridmap;
  if (!filterChain_.update(gridmap_, filtered_gridmap)) {
    RCLCPP_ERROR(node_->get_logger(), "Could not update the grid map filter chain!");
    return;
  }

  auto message = grid_map::GridMapRosConverter::toMessage(filtered_gridmap);
  message->header.stamp = node_->now();
  gridmap_pub_->publish(*message);
}

} // namespace elevation_mapping

int main(int argc, char **argv) 
{
  // init node
  rclcpp::init(argc, argv);

  // Create ros node
  auto node = std::make_shared<rclcpp::Node>("elevation_map");

  // Create cpp node
  [[maybe_unused]] elevation_mapping::ElevationMapping elevation_mapping(node);

  // Create executor
  rclcpp::experimental::executors::EventsExecutor exec;
  exec.add_node(node);

  // Spin executor to process callbacks
  exec.spin();
  rclcpp::shutdown();
}