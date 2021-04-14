#include <webots_ros2_cpp/WebotsRos2Plugin.hpp>
#include <webots/Supervisor.hpp>

#include <string>
#include <map>

namespace webots_ros2
{
    class DistanceSensor : public WebotsPlugin
    {
    public:
        WebotsPlugin(const webots::Supervisor *robot, const std::map<std::string, std::string> &parameters)
        {
            mRobot = robot;
            mName = parameters["name"];
        };
        virtual void step(int size) override
        {

        }

    private:
        std::string mName;
        std::shared_ptr<webots::Supervisor> mRobot;
    };
}
