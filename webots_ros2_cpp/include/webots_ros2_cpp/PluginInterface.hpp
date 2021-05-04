#ifndef PLUGIN_INTERFACE
#define PLUGIN_INTERFACE

#include <map>
#include <webots/Supervisor.hpp>

namespace webots_ros2
{
    class PluginInterface
    {
    public:
        // WebotsNode initializes the Robot/Supervisor class.
        // Parameters are passed from the WebotsNode (in the initial phase) or some other source (e.g. URDF).
        // PluginInterface(const WebotsNode &node, const std::map<std::string, std::string> &parameters);

        // This method is called on each timestep.
        // Never call `robot.step()` in this method.
        virtual void step() = 0;
        // virtual void init(webots_ros2::WebotsNode* node, const std::map<std::string, std::string> &parameters) = 0;
    };
}

#endif
