#ifndef ROS2_COMPASS_HPP
#define ROS2_COMPASS_HPP

#include <webots_ros2_driver/WebotsNode.hpp>
#include <webots_ros2_driver/plugins/Ros2SensorPlugin.hpp>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <webots_ros2_msgs/msg/float_stamped.hpp>

namespace webots_ros2_driver {
  class Ros2Compass : public Ros2SensorPlugin {
  public:
    void step() override;
    void init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) override;

  private:
    bool mustPublish();
    void publishData();

    // ROS2 topics
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr mVectorPublisher;
    geometry_msgs::msg::Vector3Stamped mVectorMessage;
    rclcpp::Publisher<webots_ros2_msgs::msg::FloatStamped>::SharedPtr mFloatPublisher;
    webots_ros2_msgs::msg::FloatStamped mFloatMessage;

    // Device
    WbDeviceTag mCompass;
    // Runtime vars
    std::string mDeviceName;
    bool mIsEnabled;
  };

}  // namespace webots_ros2_driver

#endif
