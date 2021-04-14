#include <webots/Supervisor.hpp>

namespace webots_ros2
{
    class WebotsRos2Plugin
    {
    public:
        // WebotsNode initializes the Robot/Supervisor class.
        // Parameters are passed from the WebotsNode (in the initial phase) or some other source (e.g. URDF).
        WebotsRos2Plugin(const WebotsNode &node, const  const std::map<std::string, std::string> &parameters);

        // This method is called on each timestep.
        // Never call `robot.step()` in this method.
        virtual void step(int size) = 0;
    };
}