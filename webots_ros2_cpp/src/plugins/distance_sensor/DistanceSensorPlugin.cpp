#include <webots_ros2_cpp/PluginInterface.hpp>
#include <webots/Supervisor.hpp>

#include <string>
#include <map>
#include <iostream>

namespace webots_ros2
{
    class DistanceSensorPlugin : public PluginInterface
    {
    public:
        DistanceSensor(const webots::WebotsNode *node, const std::map<std::string, std::string> &parameters)
        {
            mNode = node;
            mName = parameters["name"];
        };
        virtual void step(int size) override
        {
            std::cout << "step()\n";
        }

    private:
        std::string mName;
        std::shared_ptr<webots::Supervisor> mNode;
    };
}

extern "C" webots_ros2::PluginInterface* create_plugin()
{
    return new DistanceSensorPlugin();
}
