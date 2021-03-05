// Copyright 1996-2020 Cyberbotics Ltd.
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

#pragma once

// webots
#include <webots/Device.hpp>

// std
#include <memory>

namespace wb_ros2_interface {

class WbRos2Device {
public:
  virtual ~WbRos2Device(){};
  WbRos2Device(const std::shared_ptr<webots::Device> device) :
    device_(device)
  {};

  virtual std::string deviceName() { return device_->getName(); };
  virtual std::shared_ptr<webots::Device> device() { return device_; }

private:
  std::shared_ptr<webots::Device> device_;
};

} // end namespace wb_ros2_interface
