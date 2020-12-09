// export LD_LIBRARY_PATH=${WEBOTS_HOME}/lib/controller

#include <chrono>
#include <functional>
#include <memory>
#include <string>

// Webots dependencies
#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>

// ROS2 dependencies
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"


class DriverNode : public rclcpp::Node
{
public:
  DriverNode() : Node("webots_driver_node"), mTimeStep(64)
  {
    // For Pioneer 3-DX
    mRobot = new webots::Robot();
    mDistanceSensor = mRobot->getDistanceSensor("so4");
    mDistanceSensor->enable(mTimeStep);

    mPublisher = this->create_publisher<sensor_msgs::msg::Range>("/range", 1);
    mTimer = this->create_wall_timer(std::chrono::milliseconds(mTimeStep), std::bind(&DriverNode::timerCallback, this));
  }

private:
  void timerCallback()
  {
    mRobot->step(mTimeStep);

    auto message = sensor_msgs::msg::Range();
    message.range = mDistanceSensor->getValue();
    mPublisher->publish(message);
  }

  webots::Robot *mRobot;
  webots::DistanceSensor *mDistanceSensor;
  rclcpp::TimerBase::SharedPtr mTimer;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr mPublisher;
  int mTimeStep;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DriverNode>());
  rclcpp::shutdown();
  return 0;
}
