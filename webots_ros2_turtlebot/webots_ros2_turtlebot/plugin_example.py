from webots_ros2_driver.controller import Node


class PluginExample:
    def init(self, webots_node, properties):
        print('The init() method is called')
        print('Properties:', properties)

        print('Basic timestep:', int(webots_node.robot.getBasicTimeStep()))
        print('Robot name:', webots_node.robot.getName())
        print('Is robot?', webots_node.robot.getType() == Node.ROBOT)
        print()

    def step(self):
        print('The step() method is called')
