#ifndef UTILS_HPP
#define UTILS_HPP

#include <string>

namespace webots_ros2
{
    int getDeviceTimestepMsFromPublishTimestep(double publishTimestep, int basicTimestepMs);
    std::string getFixedNameString(const std::string &name);
}

#endif
