#ifndef ROS2_RECEIVER_HPP
#define ROS2_RECEIVER_HPP

#include <webots_ros2_driver/WebotsNode.hpp>
#include <webots_ros2_driver/plugins/Ros2SensorPlugin.hpp>

#include <webots_ros2_msgs/msg/string_stamped.hpp>

namespace webots_ros2_driver {
  class Ros2Receiver : public Ros2SensorPlugin {
  public:
    // Your plugin has to override step() and init() methods
    void step() override;
    void init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) override;

  private:
    bool mustPublish();
    void publishData();
    // ROS2 topics
    rclcpp::Publisher<webots_ros2_msgs::msg::StringStamped>::SharedPtr mDataPublisher;
    webots_ros2_msgs::msg::StringStamped mDataMessage;
    // Device
    WbDeviceTag mReceiver;
    // Runtime vars
    std::string mDeviceName;
    int mDeviceChannel;
    bool mIsEnabled;
  };
} // namespace webots_ros2_driver

#endif
