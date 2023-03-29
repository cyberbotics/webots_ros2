#!/usr/bin/env python

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

import time
import os
import unittest
import rclpy
from rosgraph_msgs.msg import Clock


DEFAULT_TIMEOUT = 120.0
DEFAULT_CLOCK_TIMEOUT = 15 * DEFAULT_TIMEOUT


class TestWebots(unittest.TestCase):
    def wait_for_messages(self,
                          node,
                          message_type,
                          topic,
                          timeout=DEFAULT_TIMEOUT,
                          condition=None,
                          qos=1,
                          messages_to_receive=1):
        received_messages = []

        def on_message(message):
            if condition is None or condition(message):
                received_messages.append(message)

        subscription = node.create_subscription(
            message_type,
            topic,
            on_message,
            qos
        )
        try:
            end_time = time.time() + timeout
            while time.time() < end_time:
                rclpy.spin_once(node, timeout_sec=0.1)
                if len(received_messages) >= messages_to_receive:
                    break

            self.assertGreaterEqual(len(received_messages), messages_to_receive, 'Not enough messages have been received')
        finally:
            node.destroy_subscription(subscription)

    def wait_for_clock(self, node, timeout=DEFAULT_CLOCK_TIMEOUT, messages_to_receive=5):
        self.wait_for_messages(node, Clock, 'clock', timeout=timeout, messages_to_receive=messages_to_receive)


def initialize_webots_test():
    """Ensure there is only one instance of Webots running."""
    if 'WEBOTS_OFFSCREEN' in os.environ:
        os.system('killall -9 webots-bin')
