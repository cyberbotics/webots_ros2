#include <webots_ros2_cpp/devices/Ros2GPS.hpp>

#include <webots_ros2_cpp/utils/Utils.hpp>


namespace webots_ros2
{

  Ros2GPS::Ros2GPS(webots_ros2::WebotsNode *node, std::map<std::string, std::string> &parameters) : mNode(node), mIsEnabled(false), mLastUpdate(0)
  {
    mGPS = mNode->robot()->getGPS(parameters["name"]);

    // Parameters
    mTopicName = parameters.count("topicName") ? parameters["topicName"] : "/" + getFixedNameString(mGPS->getName());
    mPublishTimestep = parameters.count("updateRate") ? 1.0 / atof(parameters["updateRate"].c_str()) : 0;
    mAlwaysOn = parameters.count("alwaysOn") ? (parameters["alwaysOn"] == "true") : false;
    mFrameName = parameters.count("frameName") ? parameters["frameName"] : getFixedNameString(mGPS->getName());

    // Calcualte timestep
    mPublishTimestepSyncedMs = getDeviceTimestepMsFromPublishTimestep(mPublishTimestep, mNode->robot()->getBasicTimeStep());

    if (mGPS->getCoordinateSystem() == webots::GPS::WGS84)
    {
      mGPSPublisher = mNode->create_publisher<sensor_msgs::msg::NavSatFix>(mTopicName, rclcpp::SensorDataQoS().reliable());
      mGPSMessage.header.frame_id = mFrameName;
      mGPSMessage.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
      mGPSMessage.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    }
    else
    {
      mPointPublisher = mNode->create_publisher<geometry_msgs::msg::PointStamped>(mTopicName, rclcpp::SensorDataQoS().reliable());
      mPointMessage.header.frame_id = mFrameName;
    }
    mVelocityPublisher = mNode->create_publisher<std_msgs::msg::Float32>(mTopicName + "/velocity", rclcpp::SensorDataQoS().reliable());
  }

  void Ros2GPS::step()
  {
    // Update only if needed
    if (mNode->robot()->getTime() - mLastUpdate < mPublishTimestep)
      return;
    mLastUpdate = mNode->robot()->getTime();

    // Enable/Disable sensor
    const bool shouldBeEnabled = mAlwaysOn ||
                                 (mPointPublisher != nullptr && mPointPublisher->get_subscription_count() > 0) ||
                                 (mGPSPublisher != nullptr && mGPSPublisher->get_subscription_count() > 0) ||
                                 mVelocityPublisher->get_subscription_count() > 0;
    if (shouldBeEnabled != mIsEnabled)
    {
      if (shouldBeEnabled)
        mGPS->enable(mPublishTimestepSyncedMs);
      else
        mGPS->disable();
      mIsEnabled = shouldBeEnabled;
    }

    // Publish data
    if (mPointPublisher)
      pubishPoint();
    if (mGPSPublisher)
      publishGPS();

    // Publish velocity
    std_msgs::msg::Float32 mSpeedMessage;
    mSpeedMessage.data = mGPS->getSpeed();
    mVelocityPublisher->publish(mSpeedMessage);
  }

  void Ros2GPS::pubishPoint()
  {
    mPointMessage.header.stamp = rclcpp::Clock().now();
    const double *values = mGPS->getValues();
    mPointMessage.point.x = values[0];
    mPointMessage.point.y = values[1];
    mPointMessage.point.z = values[2];
    mPointPublisher->publish(mPointMessage);
  }

  void Ros2GPS::publishGPS()
  {
    mGPSMessage.header.stamp = rclcpp::Clock().now();
    const double *values = mGPS->getValues();
    mGPSMessage.latitude = values[0];
    mGPSMessage.longitude = values[1];
    mGPSMessage.altitude = values[2];
    mGPSPublisher->publish(mGPSMessage);
  }
}
