import numpy
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


# python3 simple_mapper.py --ros-args --param use_sim_time:=true
# https://answers.ros.org/question/286221/create-2d-occupancy-grid-map-by-laser-data/

class SimpleMapper(Node):
    def __init__(self, name, args=None):
        super().__init__(name)

        self.create_timer(1, self.main_loop)
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', 1)
        self.tf_publisher = TransformBroadcaster(self)

    def main_loop(self):
        now = self.get_clock().now()

        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = "map"
        tf.child_frame_id = "odom"
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        self.tf_publisher.sendTransform(tf)

        msg = OccupancyGrid()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = 0.01
        msg.info.width = 100
        msg.info.height = 100
        msg.info.origin.position.x = -(msg.info.width * msg.info.resolution / 2)
        msg.info.origin.position.y = -(msg.info.height * msg.info.resolution / 2)
        msg.data = [0]*(msg.info.width * msg.info.height)
        self.map_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    epuck_controller = SimpleMapper('epuck_simple_mapper', args=args)
    rclpy.spin(epuck_controller)
    epuck_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
