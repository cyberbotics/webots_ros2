// wb_ros2_cpp
#include <webots_ros2_cpp/sensors/wb_ros2_imu.hpp>

namespace wb_ros2_interface {
namespace sensors {

WbRos2Imu::WbRos2Imu(webots::InertialUnit* imu, 
    const std::shared_ptr<rclcpp::Node> node) :
  imu_(imu),
  WbRos2Sensor(node),
  clock_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME))
{
  if (auto nh = node_.lock()) {
    imu_pub_ = nh->create_publisher<sensor_msgs::msg::Imu>("/imu", 
      rclcpp::SensorDataQoS());
  }
}

WbRos2Imu::~WbRos2Imu() {
}

void WbRos2Imu::pubImu() {
  sensor_msgs::msg::Imu value;
  value.header.stamp = clock_->now();
  value.header.frame_id = imu_->getName();

  // switch roll and pitch axes because the Webots and ROS coordinate systems 
  // are not equivalent
  tf2::Quaternion orientation(imu_->getQuaternion()[0], 
    imu_->getQuaternion()[1], imu_->getQuaternion()[2], 
    imu_->getQuaternion()[3]);
  tf2::Quaternion orientationRosFix(0.5, 0.5, 0.5, 0.5);
  orientation = orientation * orientationRosFix;
  value.orientation.x = orientation.getX();
  value.orientation.y = orientation.getY();
  value.orientation.z = orientation.getZ();
  value.orientation.w = orientation.getW();
  for (int i = 0; i < 9; ++i)  // means "covariance unknown"
    value.orientation_covariance[i] = 0;
  value.angular_velocity.x = 0.0;
  value.angular_velocity.y = 0.0;
  value.angular_velocity.z = 0.0;
  value.angular_velocity_covariance[0] = -1;  // means no angular_velocity information
  value.linear_acceleration.x = 0.0;
  value.linear_acceleration.y = 0.0;
  value.linear_acceleration.z = 0.0;
  value.linear_acceleration_covariance[0] = -1.0;  // means no linear_acceleration information
  imu_pub_->publish(value);
}

void WbRos2Imu::publish() {
  if (auto nh = node_.lock()) {
    pubImu();
  }
}

} // end namespace sensors
} // end namespace wb_ros2_interface
