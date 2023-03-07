#ifndef ROS2_RECEIVER_HPP
#define ROS2_RECEIVER_HPP

#include <webots_ros2_driver/WebotsNode.hpp>
#include <webots_ros2_driver/plugins/Ros2SensorPlugin.hpp>

#include <webots_ros2_msgs/msg/string_stamped.hpp>
#include <webots_ros2_msgs/srv/get_float.hpp>
#include <webots_ros2_msgs/srv/get_int.hpp>
#include <webots_ros2_msgs/srv/receiver_get_emitter_direction.hpp>
#include <webots_ros2_msgs/srv/set_int.hpp>

namespace webots_ros2_driver {
  class Ros2Receiver : public Ros2SensorPlugin {
  public:
    // Your plugin has to override step() and init() methods
    void step() override;
    void init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) override;

  private:
    bool mustPublish();
    void publishData();

    void enable_callback(const std::shared_ptr<webots_ros2_msgs::srv::SetInt::Request> request,
                         std::shared_ptr<webots_ros2_msgs::srv::SetInt::Response> response);
    void get_emitter_direction_callback(
      const std::shared_ptr<webots_ros2_msgs::srv::ReceiverGetEmitterDirection::Request> request,
      std::shared_ptr<webots_ros2_msgs::srv::ReceiverGetEmitterDirection::Response> response);
    void get_sampling_period_callback(const std::shared_ptr<webots_ros2_msgs::srv::GetInt::Request> request,
                                      std::shared_ptr<webots_ros2_msgs::srv::GetInt::Response> response);
    void get_signal_strength_callback(const std::shared_ptr<webots_ros2_msgs::srv::GetFloat::Request> request,
                                      std::shared_ptr<webots_ros2_msgs::srv::GetFloat::Response> response);
    // ROS2 topics
    rclcpp::Publisher<webots_ros2_msgs::msg::StringStamped>::SharedPtr mDataPublisher;
    webots_ros2_msgs::msg::StringStamped mDataMessage;

    rclcpp::Service<webots_ros2_msgs::srv::SetInt>::SharedPtr enable_service_;
    rclcpp::Service<webots_ros2_msgs::srv::ReceiverGetEmitterDirection>::SharedPtr get_emitter_direction_service_;
    rclcpp::Service<webots_ros2_msgs::srv::GetInt>::SharedPtr get_sampling_period_service_;
    rclcpp::Service<webots_ros2_msgs::srv::GetFloat>::SharedPtr get_signal_strength_service_;
    // Device
    WbDeviceTag mReceiver;
    // Runtime vars
    std::string mDeviceName;
    int mDeviceChannel;
    bool mIsEnabled;
  };

}  // namespace webots_ros2_driver

#endif
