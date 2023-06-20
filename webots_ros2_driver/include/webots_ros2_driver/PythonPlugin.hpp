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

#ifndef PYTHON_PLUGIN_HPP
#define PYTHON_PLUGIN_HPP

#include <Python.h>
#include <string>
#include <unordered_map>

#include <webots_ros2_driver/PluginInterface.hpp>
#include <webots_ros2_driver/WebotsNode.hpp>

namespace webots_ros2_driver {
  class PythonPlugin : public PluginInterface {
  public:
    void init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) override;
    void step() override;
    void stop();

    static std::shared_ptr<PythonPlugin> createFromType(const std::string &type);

  private:
    PythonPlugin(PyObject *pyPlugin);

    PyObject *mPyPlugin;
    PyObject *getPyWebotsNodeInstance();
  };
}  // namespace webots_ros2_driver

#endif
