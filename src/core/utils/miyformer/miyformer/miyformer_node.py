#!/usr/bin/env python3
"""
Miyformer ROS2 Node - A simple demonstration node with PyTorch integration
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# init mixformer path
import os
import sys
mixformer_path = os.path.join(os.path.dirname(__file__), 'mixformer')
if mixformer_path not in sys.path:
    sys.path.append(mixformer_path)

from lib.test.evaluation import Tracker


class MiyformerNode(Node):
    """
    A simple ROS2 node that demonstrates PyTorch integration
    """

    def __init__(self):
        super().__init__("miyformer_node")
        
        # Create camera topic subscriber
        # self.subscription = self.create_subscription(

        # Create a publisher
        self.publisher_ = self.create_publisher(String, "miyformer_topic", 10)

        # Create a timer to publish messages periodically
        timer_period = 2.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # # Initialize PyTorch if available
        # self.setup_pytorch()

        self.get_logger().info("Miyformer node has been started!")



    def timer_callback(self):
        """Timer callback to publish messages"""
        msg = String()

        msg.data = f"Hello World from Miyformer!"

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')


def main(args=None):
    """Main function to run the node"""
    rclpy.init(args=args)

    node = MiyformerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
