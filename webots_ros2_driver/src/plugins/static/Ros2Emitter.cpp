#include "webots_ros2_driver/plugins/static/Ros2Emitter.hpp"

#include <webots/emitter.h>
#include <webots/robot.h>

using std::placeholders::_1;
using std::placeholders::_2;

namespace webots_ros2_driver {
  void Ros2Emitter::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) {
    Ros2SensorPlugin::init(node, parameters);
    // This parameter is read when loading the URDF file
    mDeviceName = parameters.count("name") ? parameters["name"] : "emitter";
    mDeviceChannel = parameters.count("channel") ? atoi(parameters["channel"].c_str()) : -1;

    mEmitter = wb_robot_get_device(mDeviceName.c_str());

    assert(mEmitter != 0);

    wb_emitter_set_channel(mEmitter, mDeviceChannel);

    // Initialize services, publishers and subcriptions
    get_buffer_size_service_ = mNode->create_service<webots_ros2_msgs::srv::GetInt>(
      mTopicName + "/get_buffer_size", std::bind(&Ros2Emitter::get_buffer_size_callback, this, _1, _2));
    get_channel_service_ = mNode->create_service<webots_ros2_msgs::srv::GetInt>(
      mTopicName + "/get_channel", std::bind(&Ros2Emitter::get_channel_callback, this, _1, _2));
    get_range_service_ = mNode->create_service<webots_ros2_msgs::srv::GetFloat>(
      mTopicName + "/get_range", std::bind(&Ros2Emitter::get_range_callback, this, _1, _2));
    send_service_ = mNode->create_service<webots_ros2_msgs::srv::SetString>(
      mTopicName + "/send", std::bind(&Ros2Emitter::send_callback, this, _1, _2));
    set_channel_service_ = mNode->create_service<webots_ros2_msgs::srv::SetInt>(
      mTopicName + "/set_channel", std::bind(&Ros2Emitter::set_channel_callback, this, _1, _2));
    set_range_service_ = mNode->create_service<webots_ros2_msgs::srv::SetFloat>(
      mTopicName + "/set_range", std::bind(&Ros2Emitter::set_range_callback, this, _1, _2));
    RCLCPP_DEBUG(rclcpp::get_logger(mDeviceName), "Emitter initialized!");
  }
  void Ros2Emitter::get_buffer_size_callback(const std::shared_ptr<webots_ros2_msgs::srv::GetInt::Request> request,
                                             std::shared_ptr<webots_ros2_msgs::srv::GetInt::Response> response) {
    response->value = request->ask ? wb_emitter_get_buffer_size(mEmitter) : -1;
  }
  void Ros2Emitter::get_channel_callback(const std::shared_ptr<webots_ros2_msgs::srv::GetInt::Request> request,
                                         std::shared_ptr<webots_ros2_msgs::srv::GetInt::Response> response) {
    response->value = request->ask ? wb_emitter_get_channel(mEmitter) : -1;
  }
  void Ros2Emitter::get_range_callback(const std::shared_ptr<webots_ros2_msgs::srv::GetFloat::Request> request,
                                       std::shared_ptr<webots_ros2_msgs::srv::GetFloat::Response> response) {
    response->value = request->ask ? wb_emitter_get_range(mEmitter) : -1;
  }
  void Ros2Emitter::send_callback(const std::shared_ptr<webots_ros2_msgs::srv::SetString::Request> request,
                                  std::shared_ptr<webots_ros2_msgs::srv::SetString::Response> response) {
    std::string message = request->value;
    auto umessage = reinterpret_cast<unsigned char *>(const_cast<char *>(message.c_str()));
    response->success = wb_emitter_send(mEmitter, umessage, message.length() + 1);
  }
  void Ros2Emitter::set_channel_callback(const std::shared_ptr<webots_ros2_msgs::srv::SetInt::Request> request,
                                         std::shared_ptr<webots_ros2_msgs::srv::SetInt::Response> response) {
    wb_emitter_set_channel(mEmitter, request->value);
    response->success = true;
  }
  void Ros2Emitter::set_range_callback(const std::shared_ptr<webots_ros2_msgs::srv::SetFloat::Request> request,
                                       std::shared_ptr<webots_ros2_msgs::srv::SetFloat::Response> response) {
    wb_emitter_set_range(mEmitter, request->value);
    response->success = true;
  }
  void Ros2Emitter::step() {
    if (!preStep())
      return;
  }
}  // namespace webots_ros2_driver
