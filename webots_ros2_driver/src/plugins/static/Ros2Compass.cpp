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
    mVectorPublisher = mNode->create_publisher<geometry_msgs::msg::Vector3Stamped>(mTopicName + "/north_vector",
                                                                                   rclcpp::SensorDataQoS().reliable());
    mFloatPublisher =
      mNode->create_publisher<webots_ros2_msgs::msg::FloatStamped>(mTopicName + "/bearing", rclcpp::SensorDataQoS().reliable());

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
    const bool shouldBeEnabled =
      (mVectorPublisher->get_subscription_count() > 0 || mFloatPublisher->get_subscription_count() > 0);
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
    const double *north_vector = wb_compass_get_values(mCompass);
    mVectorMessage.header.stamp = mNode->get_clock()->now();
    mVectorMessage.vector.x = north_vector[0];
    mVectorMessage.vector.y = north_vector[1];
    mVectorMessage.vector.z = north_vector[2];
    mVectorPublisher->publish(mVectorMessage);

    // north degree
    const double rad = atan2(north_vector[1], north_vector[0]);
    double bearing = (rad - 1.5708) / M_PI * 180.0;
    if (bearing < 0.0)
      bearing += 360.0;

    mFloatMessage.header.stamp = mVectorMessage.header.stamp;
    mFloatMessage.data = bearing;
    mFloatPublisher->publish(mFloatMessage);
  }
}  // namespace webots_ros2_driver
