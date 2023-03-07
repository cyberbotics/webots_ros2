#include "webots_ros2_driver/plugins/static/Ros2Receiver.hpp"
#include <webots_ros2_driver/utils/Utils.hpp>

#include <webots/receiver.h>
#include <webots/robot.h>

using std::placeholders::_1;
using std::placeholders::_2;

namespace webots_ros2_driver {
  void Ros2Receiver::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) {
    Ros2SensorPlugin::init(node, parameters);
    // This parameter is read when loading the URDF file
    mDeviceName = parameters.count("name") ? parameters["name"] : "emitter";
    mDeviceChannel = parameters.count("channel") ? atoi(parameters["channel"].c_str()) : -1;

    mReceiver = wb_robot_get_device(mDeviceName.c_str());
    mIsEnabled = false;

    assert(mReceiver != 0);

    wb_receiver_set_channel(mReceiver, mDeviceChannel);

    // Data publisher
    mDataPublisher =
      mNode->create_publisher<webots_ros2_msgs::msg::StringStamped>(mTopicName + "/data", rclcpp::SensorDataQoS().reliable());
    // Services
    enable_service_ = mNode->create_service<webots_ros2_msgs::srv::SetInt>(
      mTopicName + "/enable", std::bind(&Ros2Receiver::enable_callback, this, _1, _2));
    get_emitter_direction_service_ = mNode->create_service<webots_ros2_msgs::srv::ReceiverGetEmitterDirection>(
      mTopicName + "/get_emitter_direction", std::bind(&Ros2Receiver::get_emitter_direction_callback, this, _1, _2));
    get_sampling_period_service_ = mNode->create_service<webots_ros2_msgs::srv::GetInt>(
      mTopicName + "/get_sampling_period", std::bind(&Ros2Receiver::get_sampling_period_callback, this, _1, _2));
    get_signal_strength_service_ = mNode->create_service<webots_ros2_msgs::srv::GetFloat>(
      mTopicName + "/get_signal_strength", std::bind(&Ros2Receiver::get_signal_strength_callback, this, _1, _2));
    RCLCPP_DEBUG(rclcpp::get_logger(mDeviceName), (mDeviceName + " initialized!").c_str());

    // Calculate timestep
    if (mAlwaysOn) {
      wb_receiver_enable(mReceiver, mPublishTimestepSyncedMs);
      RCLCPP_DEBUG(rclcpp::get_logger(mDeviceName), (mDeviceName + " activated!").c_str());
      mIsEnabled = true;
    }
  }
  void Ros2Receiver::step() {
    if (!preStep())
      return;

    if (mIsEnabled)
      publishData();

    if (mAlwaysOn)
      return;

    mustPublish();
  }
  bool Ros2Receiver::mustPublish() {
    // Enable/Disable sensor
    const bool shouldBeEnabled = mDataPublisher->get_subscription_count() > 0;
    if (shouldBeEnabled != mIsEnabled) {
      if (shouldBeEnabled)
        wb_receiver_enable(mReceiver, mPublishTimestepSyncedMs);
      else
        wb_receiver_disable(mReceiver);
      mIsEnabled = shouldBeEnabled;
    }
    return mIsEnabled;
  }
  void Ros2Receiver::publishData() {
    // If there is any packet publish the data
    if (wb_receiver_get_queue_length(mReceiver) > 0) {
      const int length_buffer = wb_receiver_get_data_size(mReceiver);
      const void *received_data = wb_receiver_get_data(mReceiver);
      // cast to char*
      char *publishing_data_char = (char *)realloc(const_cast<void *>(received_data), length_buffer);
      // publish
      mDataMessage.data = std::string(publishing_data_char);
      mDataMessage.header.stamp = mNode->get_clock()->now();
      mDataPublisher->publish(mDataMessage);
      wb_receiver_next_packet(mReceiver);
    }
  }
  void Ros2Receiver::enable_callback(const std::shared_ptr<webots_ros2_msgs::srv::SetInt::Request> request,
                                     std::shared_ptr<webots_ros2_msgs::srv::SetInt::Response> response) {
    if (request->value) {
      wb_receiver_enable(mReceiver, mPublishTimestepSyncedMs);
      mAlwaysOn = true;
      mIsEnabled = true;
    } else {
      wb_receiver_disable(mReceiver);
      mAlwaysOn = false;
      mIsEnabled = false;
    }
    response->success = true;
  }
  void Ros2Receiver::get_emitter_direction_callback(const std::shared_ptr<webots_ros2_msgs::srv::ReceiverGetEmitterDirection::Request> request,
                                                    std::shared_ptr<webots_ros2_msgs::srv::ReceiverGetEmitterDirection::Response> response) {
    std::vector<double> value {0, 0, 0};

    if (request->ask) {
      const double * emitter_dir = wb_receiver_get_emitter_direction(mReceiver);
      value.at(0) = emitter_dir[0];
      value.at(1) = emitter_dir[1];
      value.at(2) = emitter_dir[2];
    }

    response->direction = value;
  }
  void Ros2Receiver::get_sampling_period_callback(const std::shared_ptr<webots_ros2_msgs::srv::GetInt::Request> request,
                                                  std::shared_ptr<webots_ros2_msgs::srv::GetInt::Response> response) {
    response->value = request->ask ? wb_receiver_get_sampling_period(mReceiver) : 0;
  }
  void Ros2Receiver::get_signal_strength_callback(const std::shared_ptr<webots_ros2_msgs::srv::GetFloat::Request> request,
                                                  std::shared_ptr<webots_ros2_msgs::srv::GetFloat::Response> response) {
    response->value = request->ask ? wb_receiver_get_signal_strength(mReceiver) : 0;
  }
}  // namespace webots_ros2_driver
