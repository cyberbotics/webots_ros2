#include "webots_ros2_driver/plugins/static/Ros2Compass.hpp"

#include <webots/compass.h>
#include <webots/robot.h>

using std::placeholders::_1;
using std::placeholders::_2;

namespace webots_ros2_driver {
  void Ros2Compass::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) {
    Ros2SensorPlugin::init(node, parameters);
    // This parameter is read when loading the URDF file
    mDeviceName = parameters.count("name") ? parameters["name"] : "compass";

    mCompass = wb_robot_get_device(mDeviceName.c_str());
    mIsEnabled = false;

    assert(mCompass != 0);

    // Data publisher
    mDataPublisher =
      mNode->create_publisher<sensor_msgs::msg::MagneticField>(mTopicName + "/values", rclcpp::SensorDataQoS().reliable());
    // Services
    enable_service_ = mNode->create_service<webots_ros2_msgs::srv::SetInt>(
      mTopicName + "/enable", std::bind(&Ros2Compass::enable_callback, this, _1, _2));
    get_sampling_period_service_ = mNode->create_service<webots_ros2_msgs::srv::GetInt>(
      mTopicName + "/get_sampling_period", std::bind(&Ros2Compass::get_sampling_period_callback, this, _1, _2));
    get_lookup_table_service_ = mNode->create_service<webots_ros2_msgs::srv::GetFloatArray>(
      mTopicName + "/get_lookup_table", std::bind(&Ros2Compass::get_lookup_table_callback, this, _1, _2));
    RCLCPP_DEBUG(rclcpp::get_logger(mDeviceName), (mDeviceName + " initialized!").c_str());

    // Calculate timestep
    if (mAlwaysOn) {
      wb_compass_enable(mCompass, mPublishTimestepSyncedMs);
      RCLCPP_DEBUG(rclcpp::get_logger(mDeviceName), (mDeviceName + " activated!").c_str());
      mIsEnabled = true;
    }
  }
  void Ros2Compass::step() {
    if (!preStep())
      return;

    if (mIsEnabled)
      publishData();

    if (mAlwaysOn)
      return;

    mustPublish();
  }
  bool Ros2Compass::mustPublish() {
    // Enable/Disable sensor
    const bool shouldBeEnabled = mDataPublisher->get_subscription_count() > 0;
    if (shouldBeEnabled != mIsEnabled) {
      if (shouldBeEnabled)
        wb_compass_enable(mCompass, mPublishTimestepSyncedMs);
      else
        wb_compass_disable(mCompass);
      mIsEnabled = shouldBeEnabled;
    }
    return mIsEnabled;
  }
  void Ros2Compass::publishData() {
    const double *compass_values = wb_compass_get_values(mCompass);
    mDataMessage.header.stamp = mNode->get_clock()->now();
    mDataMessage.magnetic_field.x = compass_values[0];
    mDataMessage.magnetic_field.y = compass_values[1];
    mDataMessage.magnetic_field.z = compass_values[2];
    mDataPublisher->publish(mDataMessage);
  }
  void Ros2Compass::enable_callback(const std::shared_ptr<webots_ros2_msgs::srv::SetInt::Request> request,
                                    std::shared_ptr<webots_ros2_msgs::srv::SetInt::Response> response) {
    if (request->value) {
      wb_compass_enable(mCompass, mPublishTimestepSyncedMs);
      mAlwaysOn = true;
      mIsEnabled = true;
    } else {
      wb_compass_disable(mCompass);
      mAlwaysOn = false;
      mIsEnabled = false;
    }
    response->success = true;
  }
  void Ros2Compass::get_sampling_period_callback(const std::shared_ptr<webots_ros2_msgs::srv::GetInt::Request> request,
                                                 std::shared_ptr<webots_ros2_msgs::srv::GetInt::Response> response) {
    response->value = request->ask ? wb_compass_get_sampling_period(mCompass) : 0;
  }
  void Ros2Compass::get_lookup_table_callback(const std::shared_ptr<webots_ros2_msgs::srv::GetFloatArray::Request> request,
                                              std::shared_ptr<webots_ros2_msgs::srv::GetFloatArray::Response> response) {
    std::vector<double> values{0, 0, 0};
    if (request->ask) {
      const double *lookup_table = wb_compass_get_lookup_table(mCompass);
      values.at(0) = lookup_table[0];
      values.at(1) = lookup_table[1];
      values.at(2) = lookup_table[2];
    }
    response->values = values;
  }
}  // namespace webots_ros2_driver
