#ifndef ROS2_COMPASS_HPP
#define ROS2_COMPASS_HPP

#include <webots_ros2_driver/WebotsNode.hpp>
#include <webots_ros2_driver/plugins/Ros2SensorPlugin.hpp>

#include <sensor_msgs/msg/magnetic_field.hpp>
#include <webots_ros2_msgs/srv/get_float_array.hpp>
#include <webots_ros2_msgs/srv/get_int.hpp>
#include <webots_ros2_msgs/srv/set_int.hpp>

namespace webots_ros2_driver {
  class Ros2Compass : public Ros2SensorPlugin {
  public:
    void step() override;
    void init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) override;

  private:
    bool mustPublish();
    void publishData();

    void enable_callback(const std::shared_ptr<webots_ros2_msgs::srv::SetInt::Request> request,
                         std::shared_ptr<webots_ros2_msgs::srv::SetInt::Response> response);
    void get_sampling_period_callback(const std::shared_ptr<webots_ros2_msgs::srv::GetInt::Request> request,
                                      std::shared_ptr<webots_ros2_msgs::srv::GetInt::Response> response);
    void get_lookup_table_callback(const std::shared_ptr<webots_ros2_msgs::srv::GetFloatArray::Request> request,
                                   std::shared_ptr<webots_ros2_msgs::srv::GetFloatArray::Response> response);
    // ROS2 topics
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mDataPublisher;
    sensor_msgs::msg::MagneticField mDataMessage;

    rclcpp::Service<webots_ros2_msgs::srv::SetInt>::SharedPtr mEnableService;
    rclcpp::Service<webots_ros2_msgs::srv::GetInt>::SharedPtr mGetSamplingPeriodService;
    rclcpp::Service<webots_ros2_msgs::srv::GetFloatArray>::SharedPtr mGetLookupTableService;
    // Device
    WbDeviceTag mCompass;
    // Runtime vars
    std::string mDeviceName;
    bool mIsEnabled;
  };

}  // namespace webots_ros2_driver

#endif
