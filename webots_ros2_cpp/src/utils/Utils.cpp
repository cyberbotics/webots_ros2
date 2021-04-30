#include "webots_ros2_cpp/utils/Utils.hpp"

namespace webots_ros2
{
    int getDeviceTimestepMsFromPublishTimestep(double publishTimestep, int basicTimestepMs)
    {
        int result = basicTimestepMs;
        while (result / 1000.0 < publishTimestep - basicTimestepMs / 2000.0)
            result += basicTimestepMs;
        return result;
    }
}
