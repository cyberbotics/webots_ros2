// wb_ros2_interface
#include <webots_ros2_cpp/sensors/wb_ros2_camera.hpp>

namespace wb_ros2_interface {
namespace sensors {

WbRos2Camera::WbRos2Camera(webots::Camera* camera, 
    const std::shared_ptr<rclcpp::Node> node) :
  camera_(camera),
  WbRos2Sensor(node),
  clock_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME))
{
  if (auto nh = node_.lock()) {
    image_pub_ = nh->create_publisher<sensor_msgs::msg::Image>("/image_raw", 
      rclcpp::SensorDataQoS());
    
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
  }
}

WbRos2Camera::~WbRos2Camera() {
}

void WbRos2Camera::enable(int sampling_period) { 
  camera_->enable(sampling_period); 
  if (camera_->hasRecognition()) {
    camera_->recognitionEnable(sampling_period);
  }
  else {
    recog_pub_.reset();
    wb_recog_pub_.reset();
  }
};

void WbRos2Camera::createCameraInfoMsg() {
  camera_info_.header.stamp = clock_->now();
  camera_info_.height = camera_->getHeight();
  camera_info_.width = camera_->getWidth();
  camera_info_.distortion_model = "plumb_bob";
  auto focal_length = (camera_->getFocalLength() == 0) ? 570.34 : 
    camera_->getFocalLength();
  camera_info_.d = {0.0, 0.0, 0.0, 0.0, 0.0};
  camera_info_.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  camera_info_.k = {
    focal_length, 0.0, (double)camera_->getWidth() / 2,
    0.0, focal_length, (double)camera_->getHeight() / 2,
    0.0, 0.0, 1.0
  };
  camera_info_.p = {
    focal_length, 0.0, (double)camera_->getWidth() / 2, 0.0,
    0.0, focal_length, (double)camera_->getHeight() / 2, 0.0,
    0.0, 0.0, 1.0, 0.0
  };
}

void WbRos2Camera::pubImage() {
  sensor_msgs::msg::Image image_msg;
  image_msg.header.stamp = clock_->now();
  image_msg.header.frame_id = camera_->getName();
  image_msg.height = camera_->getHeight();
  image_msg.width = camera_->getWidth();
  image_msg.is_bigendian = false;
  image_msg.step = sizeof(unsigned char) * 4 * camera_->getWidth();

  auto image = camera_->getImage();
  auto n = sizeof(unsigned char) * 4 * camera_->getWidth() * 
    camera_->getHeight();
  image_msg.data = std::move(std::vector<unsigned char>(image, image + n));
  
  image_msg.encoding = sensor_msgs::image_encodings::BGRA8;
  image_pub_->publish(image_msg);

  camera_info_.header.stamp = clock_->now();
  camera_info_pub_->publish(camera_info_);
}

void WbRos2Camera::pubRecognition() {
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
  for (size_t i = 0; i < camera_->getRecognitionNumberOfObjects(); ++i) {
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
    for (size_t j = 0; j < objects[i].number_of_colors; ++j) {
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

void WbRos2Camera::publish() {
  if (auto nh = node_.lock()) {
    pubImage();
    if (camera_->hasRecognition()) {
      pubRecognition();
    }
  }
}

} // end namespace sensors
} // end namespace wb_ros2_interface
