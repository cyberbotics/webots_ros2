#ifndef ROS2_EMITTER_HPP
#define ROS2_EMITTER_HPP

#include <webots_ros2_driver/WebotsNode.hpp>
#include <webots_ros2_driver/plugins/Ros2SensorPlugin.hpp>

#include <webots_ros2_msgs/srv/set_string.hpp>

namespace webots_ros2_driver {
  class Ros2Emitter : public Ros2SensorPlugin {
  public:
    // Your plugin has to override step() and init() methods
    void step() override;
    void init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) override;

  private:
    void send_callback(const std::shared_ptr<webots_ros2_msgs::srv::SetString::Request> request,
                       std::shared_ptr<webots_ros2_msgs::srv::SetString::Response> response);
    // ROS2 service
    rclcpp::Service<webots_ros2_msgs::srv::SetString>::SharedPtr data_service_;
    // Device
    WbDeviceTag mEmitter;
    // Runtime vars
    std::string mDeviceName;
    int mDeviceChannel;
  };
} // namespace webots_ros2_driver

#endif
