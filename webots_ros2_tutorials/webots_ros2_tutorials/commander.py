# Copyright 1996-2020 Soft_illusion.
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

import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge


class Image_processor(Node):
    def __init__(self):
        super().__init__('Image_processor')
        # Publish cmd vel
        self.pubs_cmdvel = self.create_publisher(Twist, 'cmd_vel', 1)
        time.sleep(5)

        # vehicle parameters
        self.speed = 0.1
        self.angle_correction = -10.0

        # Initialize parameters
        self.delta = 0
        self.cmd = Twist()
        self.stop = False
        self.count = 0
        self.number_of_pixels = 512
        self.reach_threshold = self.number_of_pixels*0.4  # value is in pixels 40% of image
        self.reached = False

        self.camera_subscriber = self.create_subscription(Image, 'camera/image_raw', self.Image_processing_callback, 1)
        self.bridge = CvBridge()
        self.processedImage_publish = self.create_publisher(Image, 'camera/processed_image', 1)
        self.translation_x = 0

    def lineFollowingModule(self):
        # Constant velocity
        self.cmd.linear.x = self.speed

        # # Correction parameters
        self.cmd.angular.z = self.angle_correction*self.translation_x

        if self.stop:
            self.cmd.linear.x = 0.01
            self.cmd.angular.z = 1.2

        if self.reached:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0

        # Publish cmd vel
        self.pubs_cmdvel.publish(self.cmd)

    def Image_processing_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        height = msg.height
        width = msg.width
        self.number_of_pixels = height
        matrix_coefficients = np.mat([[1.0, 0.0, height/2], [0.0, 1.0, width / 2.0], [0.0, 0.0, 1.0]])
        distortion_coefficients = np.mat([0.0, 0.0, 0.0, 0.0])

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Change grayscale
        aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)  # Use 5x5 dictionary to find markers
        parameters = aruco.DetectorParameters_create()  # Marker detection parameters
        # lists of ids and the corners beloning to each id
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict,
                                              parameters=parameters,
                                              cameraMatrix=matrix_coefficients,
                                              distCoeff=distortion_coefficients)
        print(ids, ": ID of AR-tag")
        if np.all(ids is not None):  # If there are markers found by detector
            for i in range(0, len(ids)):  # Iterate in markers
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], 0.1, matrix_coefficients, distortion_coefficients)
                aruco.drawDetectedMarkers(frame, corners)  # Draw A square around the markers
                aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.1)  # Draw Axis
                if abs(corners[0][0][0][0]-corners[0][0][2][0]) > self.reach_threshold:
                    self.reached = True
                    self.get_logger().info('Reached the goal')
                else:
                    self.reached = False
                self.translation_x = tvec[0][0][0]
                self.stop = False
        else:
            self.translation_x = 0
            self.stop = True
        self.processedImage_publish.publish(self.bridge.cv2_to_imgmsg(frame, "rgb8"))
        self.lineFollowingModule()


def main(args=None):

    rclpy.init(args=args)

    commander = Image_processor()
    rclpy.spin(commander)

    commander.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
