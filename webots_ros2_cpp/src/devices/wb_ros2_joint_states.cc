// webots_ros2_cpp
#include <webots_ros2_cpp/sensors/wb_ros2_joint_states.hpp>

namespace wb_ros2_interface {
namespace sensors {

WbRos2JointState::WbRos2JointState(const std::shared_ptr<rclcpp::Node> node) :
    WbRos2Sensor(node),
    clock_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)) {
  if (auto nh = node_.lock()) {
    joints_pub_ = nh->create_publisher<sensor_msgs::msg::JointState>(
      "/joint_state", rclcpp::SensorDataQoS());
  }
}

WbRos2JointState::~WbRos2JointState() {
}

void WbRos2JointState::enable(int sampling_period) { 
  position_sensors_.back()->enable(sampling_period);
};

void WbRos2JointState::disable() {
  for (const auto p : position_sensors_)
    p->disable(); 
};

void WbRos2JointState::addPositionSensor(
  webots::PositionSensor* position_sensor, int sampling_period) 
{
  auto motor = position_sensor->getMotor();
  auto name = (motor) ? motor->getName() : position_sensor->getName();
  position_sensors_.emplace_back(position_sensor);
  enable(sampling_period);
  
  joint_msg_.name.push_back(name);
  joint_msg_.position.push_back(position_sensor->getValue());
  joint_msg_.velocity.push_back(0);
  joint_msg_.effort.push_back(0);
  joint_msg_.header.stamp = clock_->now();
}

void WbRos2JointState::publish() {
  if (auto nh = node_.lock()) {
    pubJoints();
  }
}

void WbRos2JointState::pubJoints() {
  auto t = clock_->now();
  auto t0 = joint_msg_.header.stamp;
  auto dt = (t - t0).seconds();
  for (size_t i = 0; i < position_sensors_.size(); ++i) {
    auto value = joint_msg_.position[i];
    joint_msg_.position[i] = position_sensors_[i]->getValue();
    if (dt > 0)
      joint_msg_.velocity[i] = (joint_msg_.position[i] - value) / dt;
  }
  joint_msg_.header.stamp = t;
  joint_msg_.header.frame_id = "joint_state";
  joints_pub_->publish(joint_msg_);
}
  
} // end namespace sensors
} // end namespace wb_ros2_interface
