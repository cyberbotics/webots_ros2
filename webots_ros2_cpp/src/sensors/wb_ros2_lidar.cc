// wb_ros2_interface
#include <webots_ros2_cpp/sensors/wb_ros2_lidar.hpp>

namespace wb_ros2_interface {
namespace sensors {

WbRos2Lidar::WbRos2Lidar(webots::Lidar* lidar, 
    const std::shared_ptr<rclcpp::Node> node) :
  lidar_(lidar),
  WbRos2Sensor(node),
  clock_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME))
{
  if (auto nh = node_.lock()) {
    laser_pub_ = nh->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 
      rclcpp::SensorDataQoS());
    pc_pub_ = nh->create_publisher<sensor_msgs::msg::PointCloud>("/point_cloud", 
      rclcpp::SensorDataQoS());
  }
}

WbRos2Lidar::~WbRos2Lidar() {
}

void WbRos2Lidar::publish() {
  if (auto nh = node_.lock()) {
    if (lidar_->getNumberOfLayers() > 1) {
      pubPointCloud();
    }
    else if (lidar_->getNumberOfLayers() == 1) {
      pubLaserScan();
    }
    else
      return;
  }
}

void WbRos2Lidar::pubPointCloud() {
  const auto data = lidar_->getPointCloud();
  if (data) {
    auto msg = sensor_msgs::msg::PointCloud();
    msg.header.stamp = clock_->now();
    msg.header.frame_id = lidar_->getName();
    sensor_msgs::msg::ChannelFloat32 layer_channel;
    layer_channel.name = "layer";
    for (int i = 0; i < lidar_->getNumberOfPoints(); i++) {
      geometry_msgs::msg::Point32 point;
      point.x = data[i].x;
      point.y = data[i].y;
      point.z = data[i].z;
      msg.points.push_back(point);
      layer_channel.values.push_back(data[i].layer_id);
    }
    msg.channels.push_back(layer_channel);
    pc_pub_->publish(msg);
  }
}

void WbRos2Lidar::pubLaserScan() {
  const auto range_image = lidar_->getLayerRangeImage(0);
  if (range_image) {
    sensor_msgs::msg::LaserScan msg;
    msg.header.stamp = clock_->now();
    msg.header.frame_id = lidar_->getName();
    msg.angle_min = - lidar_->getFov() / 2.0;
    msg.angle_max = lidar_->getFov() / 2.0;
    msg.angle_increment = lidar_->getFov() / lidar_->getHorizontalResolution();
    msg.time_increment = (double)lidar_->getSamplingPeriod() / 
      (1000.0 * lidar_->getHorizontalResolution());
    msg.scan_time = (double)lidar_->getSamplingPeriod() / 1000.0;
    msg.range_min = lidar_->getMinRange();
    msg.range_max = lidar_->getMaxRange();
    
    auto n = lidar_->getHorizontalResolution();
    msg.ranges = std::move(std::vector<float>(range_image, range_image + n));
    std::reverse(msg.ranges.begin(), msg.ranges.end());
    laser_pub_->publish(msg);
  }
}

} // end namespace sensors
} // end namespace wb_ros2_interface
