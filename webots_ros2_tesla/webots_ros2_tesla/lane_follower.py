# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""ROS2 Tesla driver."""

import cv2
import numpy as np
import rclpy
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDrive
from rclpy.qos import qos_profile_sensor_data, QoSReliabilityPolicy
from rclpy.node import Node


CONTROL_COEFFICIENT = 0.0005


class LaneFollower(Node):
    def __init__(self):
        super().__init__('lane_follower')

        # ROS interface
        self.__ackermann_publisher = self.create_publisher(AckermannDrive, 'cmd_ackermann', 1)

        qos_camera_data = qos_profile_sensor_data
        qos_camera_data.reliability = QoSReliabilityPolicy.RELIABLE
        self.create_subscription(Image, 'vehicle/camera/image_color', self.__on_camera_image, qos_camera_data)

    def __on_camera_image(self, message):
        img = message.data
        img = np.frombuffer(img, dtype=np.uint8).reshape((message.height, message.width, 4))
        img = img[160:190, :]

        # Segment the image by color in HSV color space
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2RGB)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(img, np.array([50, 110, 150]), np.array([120, 255, 255]))

        # Find the largest segmented contour
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        command_message = AckermannDrive()
        command_message.speed = 50.0
        command_message.steering_angle = 0.0

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            largest_contour_center = cv2.moments(largest_contour)

            if largest_contour_center['m00'] != 0:
                center_x = int(largest_contour_center['m10'] / largest_contour_center['m00'])
                # Find error (the lane distance from the target distance)
                error = center_x - 190
                command_message.steering_angle = error*CONTROL_COEFFICIENT

        self.__ackermann_publisher.publish(command_message)


def main(args=None):
    rclpy.init(args=args)
    follower = LaneFollower()
    rclpy.spin(follower)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
