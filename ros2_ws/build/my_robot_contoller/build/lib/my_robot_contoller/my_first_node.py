#!/usr/bin/env python3
import rclpy

from rclpy.node import Node
class MyNode(Node):
    def __init__(self):
        super().__init__('my_first_node')
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('Hello, I am Rishi Raj!')
def main(args=None):
    rclpy.init(args=args)
    # Create the node
    Node=MyNode()
    rclpy.spin(Node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()
