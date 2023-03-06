#include "webots_ros2_driver/plugins/static/Ros2Receiver.hpp"
#include <webots_ros2_driver/utils/Utils.hpp>

#include <webots/receiver.h>
#include <webots/robot.h>

// #include <cstdio>
// #include <string>

//   TODO: implement all other node functions!

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
    RCLCPP_INFO(rclcpp::get_logger(mDeviceName), (mDeviceName + " initialized!").c_str());

    // Calculate timestep
    if (mAlwaysOn) {
      wb_receiver_enable(mReceiver, mPublishTimestepSyncedMs);
      RCLCPP_INFO(rclcpp::get_logger(mDeviceName), (mDeviceName + " activated!").c_str());
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
      RCLCPP_INFO(rclcpp::get_logger(mDeviceName), "data captured");
      const int length_buffer = wb_receiver_get_data_size(mReceiver);
      const void *received_data = wb_receiver_get_data(mReceiver);
      // cast to char*
      char *publishing_data_char = (char *)realloc(const_cast<void *>(received_data), length_buffer);
      // publish
      mDataMessage.data = std::string(publishing_data_char);
      mDataMessage.header.stamp = mNode->get_clock()->now();
      mDataPublisher->publish(mDataMessage);
      // release pointer
      wb_receiver_next_packet(mReceiver);
    }
  }
}  // namespace webots_ros2_driver
