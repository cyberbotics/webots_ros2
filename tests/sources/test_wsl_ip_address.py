#!/usr/bin/env python

# Copyright 1996-2026 Cyberbotics Ltd.
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

"""Unit tests for WSL host IP address detection."""

import os
import sys
import unittest
from unittest.mock import mock_open, patch


ROOT_FOLDER = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
DRIVER_FOLDER = os.path.join(ROOT_FOLDER, 'webots_ros2_driver')
sys.path.insert(0, DRIVER_FOLDER)

from webots_ros2_driver import utils  # noqa: E402


class TestWslIpAddress(unittest.TestCase):
    """Test default gateway selection and resolver fallback."""

    @patch('webots_ros2_driver.utils.open', new_callable=mock_open, read_data='nameserver 10.255.255.254\n')
    @patch('webots_ros2_driver.utils.subprocess.run')
    def test_default_gateway_takes_precedence_over_resolver(self, run, open_file):
        """Use the Windows gateway even when a VPN changes resolv.conf."""
        run.return_value.stdout = 'default via 172.27.64.1 dev eth0 proto kernel\n'

        self.assertEqual(utils.get_wsl_ip_address(), '172.27.64.1')
        open_file.assert_not_called()

    @patch('webots_ros2_driver.utils.open', new_callable=mock_open, read_data='nameserver 10.0.0.53\n')
    @patch('webots_ros2_driver.utils.subprocess.run')
    def test_resolver_is_fallback_when_gateway_is_unavailable(self, run, _open_file):
        """Retain resolver fallback for environments without a default route."""
        run.return_value.stdout = '10.0.0.0/24 dev eth0 scope link\n'

        self.assertEqual(utils.get_wsl_ip_address(), '10.0.0.53')

    @patch('webots_ros2_driver.utils.open', side_effect=OSError)
    @patch('webots_ros2_driver.utils.subprocess.run', side_effect=OSError)
    def test_localhost_is_last_resort(self, _run, _open_file):
        """Return localhost when neither route nor resolver can be read."""
        self.assertEqual(utils.get_wsl_ip_address(), '127.0.0.1')


if __name__ == '__main__':
    unittest.main()
