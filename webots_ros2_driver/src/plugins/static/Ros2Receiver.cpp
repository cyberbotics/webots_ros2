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
    mDataPublisher = node->create_publisher<webots_ros2_msgs::msg::StringStamped>(mTopicName + "/data",
                                                                                  rclcpp::SensorDataQoS().reliable());
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
    if (shouldBeEnabled != mIsEnabled)
    {
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
//       cast for manipulation      
//       void *received_data_void = const_cast<void *>(received_data);  // remove const
//       char *publishing_data = static_cast<char *>(received_data_void); // cast type
      char *publishing_data = (char *)malloc((size_t)length_buffer);
//       char* publishing_data;
//       const std::string *publishing_data = static_cast<const std::string*>(received_data);
//       char publishing_data[length_buffer];
//       void *received_data_void = const_cast<void *>(received_data);
//       publishing_data = static_cast<char *>(received_data_void);
//       const char *publishing_data = static_cast<const char *>(received_data);
      memcpy(publishing_data, received_data, length_buffer);

      RCLCPP_INFO(rclcpp::get_logger(mDeviceName), "cast2");
      RCLCPP_INFO(rclcpp::get_logger(mDeviceName), std::to_string(strlen(publishing_data)).c_str());
//       RCLCPP_INFO(rclcpp::get_logger(mDeviceName), publishing_data);
//       RCLCPP_INFO(rclcpp::get_logger(mDeviceName), "cast");
      RCLCPP_INFO(rclcpp::get_logger(mDeviceName), std::to_string(length_buffer).c_str());
      RCLCPP_INFO(rclcpp::get_logger(mDeviceName), std::string(publishing_data).c_str());
//       mDataMessage.data = publishing_data;
//       std::string message = std::to_string(*publishing_data);
//       mDataMessage.data = message;
//       std::string message = std::to_string(*publishing_data);
      mDataMessage.header.stamp = mNode->get_clock()->now();
//       mDataMessage.data = std::string(publishing_data);
      mDataMessage.data = publishing_data;
//       RCLCPP_INFO(rclcpp::get_logger(mDeviceName), "cast3");
//       RCLCPP_INFO(rclcpp::get_logger(mDeviceName), message.c_str());
      
      mDataPublisher->publish(mDataMessage);
      wb_receiver_next_packet(mReceiver);
//       delete received_data;
      RCLCPP_INFO(rclcpp::get_logger(mDeviceName), "Data pblished1");
      RCLCPP_INFO(rclcpp::get_logger(mDeviceName), mDataMessage.data.c_str());
//       delete publishing_data;
//       RCLCPP_INFO(rclcpp::get_logger(mDeviceName), "Data pblished");
    }
  }
}  // end namespace webots_ros2_driver
