#include <webots_ros2_cpp/devices/Ros2Camera.hpp>

namespace webots_ros2
{

  Ros2Camera::Ros2Camera(webots_ros2::WebotsNode *node, std::map<std::string, std::string> &parameters) : mNode(node), mIsEnabled(false)
  {
    mCamera = mNode->robot()->getCamera(parameters["name"]);
    mTopicName = parameters.count("topicName") ? parameters["topicName"] : "/" + mCamera->getName();
    mPublishTimestep = parameters.count("updateRate") ? 1.0 / atof(parameters["updateRate"].c_str()) : 0;
    mAlwaysOn = parameters.count("alwaysOn") ? (parameters["alwaysOn"] == "true") : false;
    mFrameName = parameters.count("frameName") ? parameters["frameName"] : mCamera->getName();

    // Calcualte timestep
    mPublishTimestepSyncedMs = mNode->robot()->getBasicTimeStep();
    while (mPublishTimestepSyncedMs / 1000.0 <= mPublishTimestep)
      mPublishTimestepSyncedMs *= 2;

    // Image publisher
    mImagePublisher = mNode->create_publisher<sensor_msgs::msg::Image>(mTopicName, rclcpp::SensorDataQoS().reliable());
    mImageMessage.header.frame_id = mFrameName;
    mImageMessage.height = mCamera->getHeight();
    mImageMessage.width = mCamera->getWidth();
    mImageMessage.is_bigendian = false;
    mImageMessage.step = sizeof(unsigned char) * 4 * mCamera->getWidth();
    mImageMessage.data.resize(4 * mCamera->getWidth() * mCamera->getHeight());
    mImageMessage.encoding = sensor_msgs::image_encodings::BGRA8;

    /*
    rmw_qos_profile_t qos_profile;
    qos_profile.depth = 1;
    qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    qos_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    rclcpp::QoSInitialization qos_init(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1);
    rclcpp::QoS camera_info_qos(qos_init, qos_profile);
    camera_info_pub_ = nh->create_publisher<sensor_msgs::msg::CameraInfo>(
      "/camera_info", camera_info_qos);

    recog_pub_ = nh->create_publisher<vision_msgs::msg::Detection2DArray>(
      "/recognitions", rclcpp::SensorDataQoS());

    wb_recog_pub_ = nh->create_publisher<
      webots_ros2_msgs::msg::WbCameraRecognitionObjects>("/recognitions/webots", 
      rclcpp::SensorDataQoS());
  */
  }

  void Ros2Camera::step()
  {
    // Update only if needed
    if (mNode->robot()->getTime() - mLastUpdate < mPublishTimestep)
      return;
    mLastUpdate = mNode->robot()->getTime();

    // Enable/Disable sensor
    const bool shouldBeEnabled = mAlwaysOn || mImagePublisher->get_subscription_count() > 0;
    if (shouldBeEnabled != mIsEnabled)
    {
      if (shouldBeEnabled)
        mCamera->enable(mPublishTimestepSyncedMs);
      else
        mCamera->disable();
      mIsEnabled = shouldBeEnabled;
    }

    // Publish data
    if (shouldBeEnabled)
      publishImage();
  }

  /*
  void Ros2Camera::createCameraInfoMsg()
  {
    camera_info_.header.stamp = clock_->now();
    camera_info_.height = camera_->getHeight();
    camera_info_.width = camera_->getWidth();
    camera_info_.distortion_model = "plumb_bob";
    auto focal_length = (camera_->getFocalLength() == 0) ? 570.34 : camera_->getFocalLength();
    camera_info_.d = {0.0, 0.0, 0.0, 0.0, 0.0};
    camera_info_.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    camera_info_.k = {
        focal_length, 0.0, (double)camera_->getWidth() / 2,
        0.0, focal_length, (double)camera_->getHeight() / 2,
        0.0, 0.0, 1.0};
    camera_info_.p = {
        focal_length, 0.0, (double)camera_->getWidth() / 2, 0.0,
        0.0, focal_length, (double)camera_->getHeight() / 2, 0.0,
        0.0, 0.0, 1.0, 0.0};
  }
*/

  void Ros2Camera::publishImage()
  {
    auto image = mCamera->getImage();
    if (image)
    {
      mImageMessage.header.stamp = rclcpp::Clock().now();
      memcpy(mImageMessage.data.data(), image, mImageMessage.data.size());
      mImagePublisher->publish(mImageMessage);
    }
  }

  /*
  void Ros2Camera::pubRecognition()
  {
    if (camera_->getRecognitionNumberOfObjects() == 0)
      return;

    auto objects = camera_->getRecognitionObjects();

    // Recognition Data
    vision_msgs::msg::Detection2DArray reco_msg;
    webots_ros2_msgs::msg::WbCameraRecognitionObjects reco_msg_webots;
    reco_msg.header.stamp = clock_->now();
    reco_msg_webots.header.stamp = clock_->now();
    reco_msg.header.frame_id = camera_->getName();
    reco_msg_webots.header.frame_id = camera_->getName();
    for (size_t i = 0; i < camera_->getRecognitionNumberOfObjects(); ++i)
    {
      // Getting Object Info
      geometry_msgs::msg::Point position;
      geometry_msgs::msg::Quaternion orientation;
      position.x = objects[i].position[0];
      position.y = objects[i].position[1];
      position.z = objects[i].position[2];
      axisAngleToQuaternion(objects[i].orientation, orientation);

      // Object Info -> Detection2D
      vision_msgs::msg::Detection2D detection;
      vision_msgs::msg::ObjectHypothesisWithPose hyp;
      hyp.id = std::string(objects[i].model);
      hyp.pose.pose.position = position;
      hyp.pose.pose.orientation = orientation;
      detection.results.push_back(hyp);
      detection.bbox.center.x = objects[i].position_on_image[0];
      detection.bbox.center.y = objects[i].position_on_image[1];
      detection.bbox.size_x = objects[i].size_on_image[0];
      detection.bbox.size_y = objects[i].size_on_image[1];
      reco_msg.detections.push_back(detection);

      // Object Info -> WbCameraRecognitionObject
      webots_ros2_msgs::msg::WbCameraRecognitionObject reco_webots_obj;
      reco_webots_obj.id = objects[i].id;
      reco_webots_obj.model = std::string(objects[i].model);
      reco_webots_obj.pose.pose.position = position;
      reco_webots_obj.pose.pose.orientation = orientation;
      reco_webots_obj.bbox.center.x = objects[i].position_on_image[0];
      reco_webots_obj.bbox.center.y = objects[i].position_on_image[1];
      reco_webots_obj.bbox.size_x = objects[i].size_on_image[0];
      reco_webots_obj.bbox.size_y = objects[i].size_on_image[1];
      for (size_t j = 0; j < objects[i].number_of_colors; ++j)
      {
        std_msgs::msg::ColorRGBA color;
        color.r = objects[i].colors[3 * j];
        color.g = objects[i].colors[3 * j + 1];
        color.b = objects[i].colors[3 * j + 2];
        reco_webots_obj.colors.push_back(color);
      }
      reco_msg_webots.objects.push_back(reco_webots_obj);
    }
    wb_recog_pub_->publish(reco_msg_webots);
    recog_pub_->publish(reco_msg);
  }

  void Ros2Camera::publish()
  {
    if (auto nh = node_.lock())
    {
      pubImage();
      if (camera_->hasRecognition())
      {
        pubRecognition();
      }
    }
  }
*/
}
