// webots_ros2_cpp
#include <webots_ros2_cpp/sensors/wb_ros2_gps.hpp>

namespace wb_ros2_interface {
namespace sensors {

WbRos2GPS::WbRos2GPS(webots::GPS* gps, 
    const std::shared_ptr<rclcpp::Node> node) :
  gps_(gps),
  WbRos2Sensor(node),
  clock_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)),
  wsg84(false) 
{
  if (auto nh = node_.lock()) {
    if (gps_->getCoordinateSystem() == webots::GPS::WGS84) {
      wsg84 = true;
      gps_pub_ = nh->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/gps", 
        rclcpp::SensorDataQoS());
    }
    else {
      gps_pub2_ = nh->create_publisher<geometry_msgs::msg::PointStamped>(
        "/gps/gps", rclcpp::SensorDataQoS());
    }
    velocity_pub_ = nh->create_publisher<std_msgs::msg::Float32>(
      "/gps/velocity", rclcpp::SensorDataQoS());
  }
}

WbRos2GPS::~WbRos2GPS() {
}

void WbRos2GPS::publish() {
  std_msgs::msg::Float32 speed_msg;
  speed_msg.data = gps_->getSpeed();
  velocity_pub_->publish(speed_msg);
  if (wsg84)
    pubGPSNavSat();
  else
    pubGPSPointStamped();
}


void WbRos2GPS::pubGPSPointStamped() {
  geometry_msgs::msg::PointStamped msg;
  msg.header.stamp = clock_->now();
  msg.header.frame_id = gps_->getName();
  msg.point.x = gps_->getValues()[0];
  msg.point.y = gps_->getValues()[1];
  msg.point.z = gps_->getValues()[2];
  gps_pub2_->publish(msg);
}

void WbRos2GPS::pubGPSNavSat() {
  sensor_msgs::msg::NavSatFix msg;
  msg.header.stamp = clock_->now();
  msg.header.frame_id = gps_->getName();
  msg.latitude = gps_->getValues()[0];
  msg.longitude = gps_->getValues()[1];
  msg.altitude = gps_->getValues()[2];
  msg.position_covariance_type = 
    sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
  gps_pub_->publish(msg);
}
  
} // end namespace sensors
} // end namespace wb_ros2_interface
