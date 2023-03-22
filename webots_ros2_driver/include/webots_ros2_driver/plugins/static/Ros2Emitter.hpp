#ifndef ROS2_EMITTER_HPP
#define ROS2_EMITTER_HPP

#include <webots_ros2_driver/WebotsNode.hpp>
#include <webots_ros2_driver/plugins/Ros2SensorPlugin.hpp>

#include <webots_ros2_msgs/srv/get_float.hpp>
#include <webots_ros2_msgs/srv/get_int.hpp>
#include <webots_ros2_msgs/srv/set_float.hpp>
#include <webots_ros2_msgs/srv/set_int.hpp>
#include <webots_ros2_msgs/srv/set_string.hpp>

namespace webots_ros2_driver {
  class Ros2Emitter : public Ros2SensorPlugin {
  public:
    void step() override;
    void init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) override;

  private:
    void get_buffer_size_callback(const std::shared_ptr<webots_ros2_msgs::srv::GetInt::Request> request,
                                  std::shared_ptr<webots_ros2_msgs::srv::GetInt::Response> response);
    void get_channel_callback(const std::shared_ptr<webots_ros2_msgs::srv::GetInt::Request> request,
                              std::shared_ptr<webots_ros2_msgs::srv::GetInt::Response> response);
    void get_range_callback(const std::shared_ptr<webots_ros2_msgs::srv::GetFloat::Request> request,
                            std::shared_ptr<webots_ros2_msgs::srv::GetFloat::Response> response);
    void send_callback(const std::shared_ptr<webots_ros2_msgs::srv::SetString::Request> request,
                       std::shared_ptr<webots_ros2_msgs::srv::SetString::Response> response);
    void set_channel_callback(const std::shared_ptr<webots_ros2_msgs::srv::SetInt::Request> request,
                              std::shared_ptr<webots_ros2_msgs::srv::SetInt::Response> response);
    void set_range_callback(const std::shared_ptr<webots_ros2_msgs::srv::SetFloat::Request> request,
                            std::shared_ptr<webots_ros2_msgs::srv::SetFloat::Response> response);
    // ROS2 service
    rclcpp::Service<webots_ros2_msgs::srv::GetInt>::SharedPtr mGetBufferSizeService;
    rclcpp::Service<webots_ros2_msgs::srv::GetInt>::SharedPtr mGetChannelService;
    rclcpp::Service<webots_ros2_msgs::srv::GetFloat>::SharedPtr mGetRangeService;
    rclcpp::Service<webots_ros2_msgs::srv::SetString>::SharedPtr mSendService;
    rclcpp::Service<webots_ros2_msgs::srv::SetInt>::SharedPtr mSetChannelService;
    rclcpp::Service<webots_ros2_msgs::srv::SetFloat>::SharedPtr mSetRangeService;
    // Device
    WbDeviceTag mEmitter;
    // Runtime vars
    std::string mDeviceName;
    int mDeviceChannel;
  };

}  // namespace webots_ros2_driver

#endif
