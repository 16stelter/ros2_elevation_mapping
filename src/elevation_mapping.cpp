#include "ros2_elevation_mapping/elevation_mapping.hpp"

namespace elevation_mapping {

ElevationMapping::ElevationMapping(rclcpp::Node::SharedPtr node, 
                           const std::string &ns,
                           std::vector<rclcpp::Parameter> parameters)
: node_(node),
  tf_buffer_(node->get_clock()),
  tf_listener_(tf_buffer_)
{
  pointcloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    "pointcloud",
    10,
    std::bind(&ElevationMapping::pointcloud_callback, this, std::placeholders::_1)
  );
  gridmap_pub_ = node_->create_publisher<grid_map_msgs::msg::GridMap>("grid_map", 10);
  init_gridmap();
}

void ElevationMapping::init_gridmap()
{
  gridmap_ = grid_map::GridMap({"elevation"});
  gridmap_.setFrameId("odom");
  gridmap_.setGeometry(grid_map::Length(5.0, 5.0), 0.05);
  gridmap_pub_->publish(grid_map::GridMapRosConverter::toMessage(gridmap_));
  feature_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("feature_map", 10);
  RCLCPP_INFO(node_->get_logger(), "Initialized grid map with size: (%i, %i)", gridmap_.getSize()(0), gridmap_.getSize()(1));
  converter_.initializeFromGridMap(gridmap_, costmap_);

}

void ElevationMapping::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  sensor_msgs::msg::PointCloud2 cloud_transformed;
  try {
    // create transform if it does not exist
    if (!tf_buffer_.canTransform("odom", "base_link", msg->header.stamp, rclcpp::Duration::from_seconds(0.1))) { //todo: parametrize frames
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = msg->header.stamp;
      transform.header.frame_id = "odom";
      transform.child_frame_id = "base_link";
      transform.transform.translation.x = 0.0;
      transform.transform.translation.y = 0.0;
      transform.transform.translation.z = 0.0;
      transform.transform.rotation.w = 1.0;
      transform.transform.rotation.x = 0.0;
      transform.transform.rotation.y = 0.0;
      transform.transform.rotation.z = 0.0;
      tf_buffer_.setTransform(transform, "default_authority");
    }
    geometry_msgs::msg::TransformStamped transform =
    tf_buffer_.lookupTransform("odom", msg->header.frame_id, msg->header.stamp,
                               rclcpp::Duration::from_seconds(0.1));
    tf2::doTransform(*msg, cloud_transformed, transform);
  } catch (tf2::TransformException &e) {
    RCLCPP_WARN(node_->get_logger(), "Transform failed: %s", e.what());
    return;
  }

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_transformed, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_transformed, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_transformed, "z");
  std::map<std::pair<int, int>, std::vector<float>> cell_points;
  
  for(; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    grid_map::Index i;
    if(gridmap_.getIndex(grid_map::Position(*iter_x, *iter_y), i)) {
      cell_points[{i[0], i[1]}].push_back(*iter_z);
    }
  }

  sensor_msgs::msg::PointCloud2 feature_cloud;
  feature_cloud.header = cloud_transformed.header;
  feature_cloud.height = 1;
  feature_cloud.is_dense = false;
  feature_cloud.is_bigendian = false;
  std::vector<std::array<float, 4>> features;
  for(auto &c : cell_points)
  {
    auto i = c.first;
    auto &vs = c.second;
    float avg_z = std::accumulate(vs.begin(), vs.end(), 0.0f) / vs.size();
    grid_map::Index idx;
    idx[0] = i.first;
    idx[1] = i.second;
    gridmap_.at("elevation", idx) = avg_z;
    float max_slope = 0.0;

    grid_map::Index neighbours[8] = {
      grid_map::Index{idx[0]-1, idx[1]-1},
      grid_map::Index{idx[0]-1, idx[1]},
      grid_map::Index{idx[0]-1, idx[1]+1},
      grid_map::Index{idx[0], idx[1]-1},
      grid_map::Index{idx[0], idx[1]+1},
      grid_map::Index{idx[0]+1, idx[1]-1},
      grid_map::Index{idx[0]+1, idx[1]},
      grid_map::Index{idx[0]+1, idx[1]+1}
    };
    for(auto &n : neighbours)
    {
      if (n[0] >= 0 && n[0] < static_cast<int>(gridmap_.getSize()(0)) &&
          n[1] >= 0 && n[1] < static_cast<int>(gridmap_.getSize()(1)))
      {
        float dz = avg_z - gridmap_.at("elevation", n);
        float distance = gridmap_.getResolution();
        if(n[0] != idx[0] && n[1] != idx[1])
        {
          distance *= std::sqrt(2.0);
        }
        float slope = std::atan(dz / distance) * 180.0 / M_PI;
        max_slope = std::max(max_slope, std::abs(slope));
      }
    }
    if (max_slope > 40.0) //todo: parametrize
    {
      grid_map::Position pos;
      gridmap_.getPosition(idx, pos);
      features.push_back({static_cast<float>(pos.x()), 
                    static_cast<float>(pos.y()), 
                    static_cast<float>(avg_z), 
                    static_cast<float>(max_slope)});
    }
  }
  
  sensor_msgs::PointCloud2Modifier modifier(feature_cloud);
  modifier.setPointCloud2Fields(4, 
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(features.size());
  sensor_msgs::PointCloud2Iterator<float> out_x(feature_cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> out_y(feature_cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> out_z(feature_cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> out_intensity(feature_cloud, "intensity");
  for(auto &f : features)
  {
    *out_x = f[0];
    *out_y = f[1];
    *out_z = f[2];
    *out_intensity = f[3];
    ++out_x; ++out_y; ++out_z; ++out_intensity;
  }
  auto message = grid_map::GridMapRosConverter::toMessage(gridmap_);
  gridmap_pub_->publish(*message);
  feature_pub_->publish(feature_cloud);
  converter_.setCostmap2DFromGridMap(gridmap_, "elevation", costmap_);
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