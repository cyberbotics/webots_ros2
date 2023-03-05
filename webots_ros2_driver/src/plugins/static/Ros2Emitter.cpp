#include "webots_ros2_driver/plugins/static/Ros2Emitter.hpp"

#include <webots/emitter.h>
#include <webots/robot.h>

using std::placeholders::_1;
using std::placeholders::_2;


// TODO: implement
// set_channel
// get_channel
// set_range
// get_range
// get_buffer_size

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
    data_service_ = mNode->create_service<webots_ros2_msgs::srv::SetString>(mTopicName + "/send",
                                                                           std::bind(&Ros2Emitter::send_callback, this, _1, _2));
    RCLCPP_INFO(rclcpp::get_logger(mDeviceName), "Emitter initialized!");
  }
  void Ros2Emitter::send_callback(const std::shared_ptr<webots_ros2_msgs::srv::SetString::Request> request,
                                  std::shared_ptr<webots_ros2_msgs::srv::SetString::Response> response) {
    std::string message = request->value;
    auto umessage = reinterpret_cast<unsigned char*>(const_cast<char *>(message.c_str()));
    int ok = wb_emitter_send(mEmitter, umessage, message.length() + 1);
    response->success = ok;
  }
  void Ros2Emitter::step() {
    if (!preStep())
      return;
  }
}  // namespace webots_ros2_driver
