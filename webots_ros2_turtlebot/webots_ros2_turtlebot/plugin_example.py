from webots_ros2_driver.controller import Node


class PluginExample:
    def init(self, webots_node, properties):
        print('The init() method is called')
        print('  - properties:', properties)

        print('  - basic timestep:', int(webots_node.robot.getBasicTimeStep()))
        print('  - robot name:', webots_node.robot.getName())
        print('  - is robot?', webots_node.robot.getType() == Node.ROBOT)
        print()

    def step(self):
        print('The step() method is called')
