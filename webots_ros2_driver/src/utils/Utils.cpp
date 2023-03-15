// Copyright 1996-2023 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "webots_ros2_driver/utils/Utils.hpp"

#include <algorithm>

namespace webots_ros2_driver {
  int getDeviceTimestepMsFromPublishTimestep(double publishTimestep, int basicTimestepMs) {
    int result = basicTimestepMs;
    while (result / 1000.0 < publishTimestep - basicTimestepMs / 2000.0)
      result += basicTimestepMs;
    return result;
  }

  std::string getFixedNameString(const std::string &name) {
    std::string fixedName = name;
    std::replace(fixedName.begin(), fixedName.end(), '-', '_');
    std::replace(fixedName.begin(), fixedName.end(), '.', '_');
    std::replace(fixedName.begin(), fixedName.end(), ' ', '_');
    std::replace(fixedName.begin(), fixedName.end(), ')', '_');
    std::replace(fixedName.begin(), fixedName.end(), '(', '_');
    return fixedName;
  }

}  // namespace webots_ros2_driver
