from webots_ros2_driver.webots.controller import Node
from std_msgs.msg import Float32
import rclpy
import rclpy.node


class PluginExample:
    def init(self, webots_node, properties):
        print('The init() method is called')
        print('  - properties:', properties)

        print('  - basic timestep:', int(webots_node.robot.getBasicTimeStep()))
        print('  - robot name:', webots_node.robot.getName())
        print('  - is robot?', webots_node.robot.getType() == Node.ROBOT)
        print()

        self.__robot = webots_node.robot

        # Unfortunately, we cannot get an instance of the parent ROS node.
        # However, we can create a new one.
        rclpy.init(args=None)
        self.__node = rclpy.node.Node('plugin_node_example')
        print('Node created')
        self.__publisher = self.__node.create_publisher(Float32, 'custom_time', 1)
        print('Publisher created')

    def step(self):
        self.__publisher.publish(Float32(data=self.__robot.getTime()))
        print('The step() method is called')
