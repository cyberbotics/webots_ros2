#include <webots_ros2_cpp/PluginInterface.hpp>
#include <webots_ros2_cpp/WebotsNode.hpp>
#include <webots/Supervisor.hpp>

#include <string>
#include <map>
#include <iostream>
#include <memory>

namespace webots_ros2
{
    class DistanceSensorPlugin : public PluginInterface
    {
    public:
        DistanceSensorPlugin(webots_ros2::WebotsNode* node, const std::map<std::string, std::string> &parameters)
        {
            mNode = node;
            mName = parameters.at("name");
        };
        virtual void step() override
        {
            std::cout << "step()\n";
        }


    private:
        std::string mName;
        webots_ros2::WebotsNode* mNode;
    };
}

extern "C" std::shared_ptr<webots_ros2::PluginInterface> create_plugin(webots_ros2::WebotsNode* node, const std::map<std::string, std::string> &parameters)
{
    return new webots_ros2::DistanceSensorPlugin(node, parameters);
}
