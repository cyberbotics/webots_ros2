#include <webots_ros2_cpp/devices/Ros2Lidar.hpp>

namespace webots_ros2
{
  Ros2Lidar::Ros2Lidar(webots_ros2::WebotsNode *node, std::map<std::string, std::string> &parameters) : mNode(node)
  {
    mLidar = mNode->robot()->getLidar("lidar");

    // Parameters
    mTopicName = parameters.count("topicName") ? parameters["topicName"] : "/" + mLidar->getName();

    // Initialize publishers
    mLaserPublisher = mNode->create_publisher<sensor_msgs::msg::LaserScan>(mTopicName,  rclcpp::SensorDataQoS().reliable());
    mPointCloudPublisher = mNode->create_publisher<sensor_msgs::msg::PointCloud>(mTopicName + "/point_cloud", rclcpp::SensorDataQoS().reliable());
  }

  void Ros2Lidar::step(int size)
  {
    if (mLidar->getNumberOfLayers() > 1)
      pubPointCloud();
    if (mLidar->getNumberOfLayers() == 1)
      pubLaserScan();
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
