#include <webots_ros2_cpp/devices/Ros2Lidar.hpp>

namespace webots_ros2
{
  Ros2Lidar::Ros2Lidar(webots_ros2::WebotsNode *node, std::map<std::string, std::string> &parameters) : mNode(node)
  {
    mLidar = mNode->robot()->getLidar(parameters["name"]);

    // Parameters
    mTopicName = parameters.count("topicName") ? parameters["topicName"] : "/" + mLidar->getName();
    mPublishTimestep = parameters.count("updateRate") ? 1.0 / atof(parameters["updateRate"].c_str()) : 0;
    mAlwaysOn = parameters.count("alwaysOn") ? (parameters["alwaysOn"] == "true") : false;

    // Calcualte timestep
    mPublishTimestepSyncedMs = mNode->robot()->getBasicTimeStep();
    while (mPublishTimestepSyncedMs / 1000.0 <= mPublishTimestep)
      mPublishTimestepSyncedMs *= 2;

    // Initialize publishers
    if (mLidar->getNumberOfLayers() == 1)
      mLaserPublisher = mNode->create_publisher<sensor_msgs::msg::LaserScan>(mTopicName, rclcpp::SensorDataQoS().reliable());
    mPointCloudPublisher = mNode->create_publisher<sensor_msgs::msg::PointCloud>(mTopicName + "/point_cloud", rclcpp::SensorDataQoS().reliable());

    mLastUpdate = mNode->robot()->getTime();
  }

  void Ros2Lidar::step()
  {
    if (mNode->robot()->getTime() - mLastUpdate < mPublishTimestep)
      return;
    mLastUpdate = mNode->robot()->getTime();

    // Enable/Disable sensor
    if (mAlwaysOn || mLaserPublisher->get_subscription_count() > 0 || mPointCloudPublisher->get_subscription_count() > 0)
      mLidar->enable(mPublishTimestepSyncedMs);
    else
      mLidar->disable();

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
      // TODO: We need PointCloud2
      /*
      auto message = sensor_msgs::msg::PointCloud();
      // message.header.stamp = clock_->now();
      message.header.frame_id = mLidar->getName();
      sensor_msgs::msg::ChannelFloat32 layer_channel;
      layer_channel.name = "layer";
      for (int i = 0; i < mLidar->getNumberOfPoints(); i++)
      {
        geometry_msgs::msg::Point32 point;
        point.x = data[i].x;
        point.y = data[i].y;
        point.z = data[i].z;
        message.points.push_back(point);
        layer_channel.values.push_back(data[i].layer_id);
      }
      message.channels.push_back(layer_channel);
      mPointCloudPublisher->publish(message);
      */
    }
  }

  void Ros2Lidar::pubLaserScan()
  {
    const auto rangeImage = mLidar->getLayerRangeImage(0);
    if (rangeImage)
    {
      sensor_msgs::msg::LaserScan message;
      // message.header.stamp = clock_->now();
      message.header.frame_id = mLidar->getName();
      message.angle_min = -mLidar->getFov() / 2.0;
      message.angle_max = mLidar->getFov() / 2.0;
      message.angle_increment = mLidar->getFov() / mLidar->getHorizontalResolution();
      message.time_increment = (double)mLidar->getSamplingPeriod() /
                               (1000.0 * mLidar->getHorizontalResolution());
      message.scan_time = (double)mLidar->getSamplingPeriod() / 1000.0;
      message.range_min = mLidar->getMinRange();
      message.range_max = mLidar->getMaxRange();

      auto n = mLidar->getHorizontalResolution();
      message.ranges = std::move(std::vector<float>(rangeImage, rangeImage + n));
      std::reverse(message.ranges.begin(), message.ranges.end());
      mLaserPublisher->publish(message);
    }
  }

} // end namespace webots_ros2
