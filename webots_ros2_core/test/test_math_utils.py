# Copyright 1996-2021 Cyberbotics Ltd.
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

import unittest
from webots_ros2_core.math.interpolation import interpolate_lookup_table


TABLE_DESC = [
    0,     1000,  0,
    0.1,   1000,  0.1,
    0.2,    400,  0.1,
    0.3,     50,  0.1,
    0.37,    30,  0
]

TABLE_ASC = [
    0.37,    30,  0,
    0.3,     50,  0.1,
    0.2,    400,  0.1,
    0.1,   1000,  0.1,
    0,     1000,  0
]


class TestMathUtils(unittest.TestCase):
    def test_lookup_desc_interpolation(self):
        self.assertAlmostEqual(interpolate_lookup_table(900, TABLE_DESC), 0.117, places=3)
        self.assertAlmostEqual(interpolate_lookup_table(31, TABLE_DESC), 0.3665, places=3)

    def test_lookup_desc_edge(self):
        self.assertAlmostEqual(interpolate_lookup_table(1000, TABLE_DESC), 0.05, places=3)
        self.assertAlmostEqual(interpolate_lookup_table(30, TABLE_DESC), 0.37, places=3)

    def test_lookup_desc_extrapolation(self):
        self.assertAlmostEqual(interpolate_lookup_table(2000, TABLE_DESC), 0, places=3)
        self.assertAlmostEqual(interpolate_lookup_table(0, TABLE_DESC), 0.475, places=3)

    def test_lookup_asc_interpolation(self):
        self.assertAlmostEqual(interpolate_lookup_table(900, TABLE_ASC), 0.117, places=3)
        self.assertAlmostEqual(interpolate_lookup_table(31, TABLE_ASC), 0.3665, places=3)

    def test_lookup_asc_edge(self):
        self.assertAlmostEqual(interpolate_lookup_table(1000, TABLE_ASC), 0.05, places=3)
        self.assertAlmostEqual(interpolate_lookup_table(30, TABLE_ASC), 0.37, places=3)

    def test_lookup_asc_extrapolation(self):
        self.assertAlmostEqual(interpolate_lookup_table(2000, TABLE_ASC), 0, places=3)
        self.assertAlmostEqual(interpolate_lookup_table(0, TABLE_ASC), 0.475, places=3)


if __name__ == '__main__':
    unittest.main()
