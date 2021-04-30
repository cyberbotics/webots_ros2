#include <webots_ros2_cpp/devices/Ros2Lidar.hpp>

#include <sensor_msgs/msg/point_field.hpp>

namespace webots_ros2
{
  Ros2Lidar::Ros2Lidar(webots_ros2::WebotsNode *node, std::map<std::string, std::string> &parameters) : mNode(node)
  {
    mLidar = mNode->robot()->getLidar(parameters["name"]);

    // Parameters
    mTopicName = parameters.count("topicName") ? parameters["topicName"] : "/" + mLidar->getName();
    mPublishTimestep = parameters.count("updateRate") ? 1.0 / atof(parameters["updateRate"].c_str()) : 0;
    mAlwaysOn = parameters.count("alwaysOn") ? (parameters["alwaysOn"] == "true") : false;
    mFrameName = parameters.count("frameName") ? parameters["frameName"] : mLidar->getName();

    // Calcualte timestep
    mPublishTimestepSyncedMs = mNode->robot()->getBasicTimeStep();
    while (mPublishTimestepSyncedMs / 1000.0 <= mPublishTimestep)
      mPublishTimestepSyncedMs *= 2;

    // Initialize publishers
    if (mLidar->getNumberOfLayers() == 1)
    {
      mLaserPublisher = mNode->create_publisher<sensor_msgs::msg::LaserScan>(mTopicName, rclcpp::SensorDataQoS().reliable());
    }
    mPointCloudPublisher = mNode->create_publisher<sensor_msgs::msg::PointCloud2>(mTopicName + "/point_cloud", rclcpp::SensorDataQoS().reliable());
    mLidar->enablePointCloud();

    mLastUpdate = mNode->robot()->getTime();
    mIsEnabled = false;
  }

  void Ros2Lidar::step()
  {
    // Update only if needed
    if (mNode->robot()->getTime() - mLastUpdate < mPublishTimestep)
      return;
    mLastUpdate = mNode->robot()->getTime();

    // Enable/Disable sensor
    const bool shouldBeEnabled = mAlwaysOn || mLaserPublisher->get_subscription_count() > 0 || mPointCloudPublisher->get_subscription_count() > 0;
    if (shouldBeEnabled != mIsEnabled)
    {
      if (shouldBeEnabled)
        mLidar->enable(mPublishTimestepSyncedMs);
      else
        mLidar->disable();
      mIsEnabled = shouldBeEnabled;
    }

    // Publish data
    if (mLaserPublisher != nullptr && (mLaserPublisher->get_subscription_count() > 0 || mAlwaysOn))
      pubLaserScan();
    if (mPointCloudPublisher->get_subscription_count() > 0 || mAlwaysOn)
      pubPointCloud();
  }

  void Ros2Lidar::pubPointCloud()
  {
    const auto data = mLidar->getPointCloud();
    if (data)
    {
      auto message = sensor_msgs::msg::PointCloud2();
      message.header.stamp = rclcpp::Clock().now();
      message.header.frame_id = mFrameName;
      message.height = 1;
      message.width = mLidar->getNumberOfPoints();
      message.point_step = 20;
      message.row_step = 20 * mLidar->getNumberOfPoints();
      message.is_dense = false;
      message.fields.resize(3);
      message.fields[0].name = "x";
      message.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
      message.fields[0].count = 1;
      message.fields[0].offset = 0;
      message.fields[1].name = "y";
      message.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
      message.fields[1].count = 1;
      message.fields[1].offset = 4;
      message.fields[2].name = "z";
      message.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
      message.fields[2].count = 1;
      message.fields[2].offset = 8;
      message.is_bigendian = false;
      message.data.resize(message.row_step * message.height);
      memcpy(message.data.data(), data, message.row_step * message.height);
      mPointCloudPublisher->publish(message);
    }
  }

  void Ros2Lidar::pubLaserScan()
  {
    const auto rangeImage = mLidar->getLayerRangeImage(0);
    if (rangeImage)
    {
      const int resolution = mLidar->getHorizontalResolution();

      sensor_msgs::msg::LaserScan message;
      message.header.stamp = rclcpp::Clock().now();
      message.header.frame_id = mFrameName;
      message.angle_min = -mLidar->getFov() / 2.0;
      message.angle_max = mLidar->getFov() / 2.0;
      message.angle_increment = mLidar->getFov() / resolution;
      message.time_increment = (double)mLidar->getSamplingPeriod() / (1000.0 * resolution);
      message.scan_time = (double)mLidar->getSamplingPeriod() / 1000.0;
      message.range_min = mLidar->getMinRange();
      message.range_max = mLidar->getMaxRange();

      message.ranges = std::vector<float>(rangeImage, rangeImage + resolution);
      mLaserPublisher->publish(message);
    }
  }

} // end namespace webots_ros2
