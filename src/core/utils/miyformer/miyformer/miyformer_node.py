#!/usr/bin/env python3
"""
Miyformer ROS2 Node - A simple demonstration node with PyTorch integration
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import os
import sys
mixformer_path = os.path.join(os.path.dirname(__file__), 'mixformer')
if mixformer_path not in sys.path:
    sys.path.append(mixformer_path)

from lib.test.evaluation import Tracker

try:
    import torch

    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False


class MiyformerNode(Node):
    """
    A simple ROS2 node that demonstrates PyTorch integration
    """

    def __init__(self):
        super().__init__("miyformer_node")

        # Create a publisher
        self.publisher_ = self.create_publisher(String, "miyformer_topic", 10)

        # Create a timer to publish messages periodically
        timer_period = 2.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize PyTorch if available
        self.setup_pytorch()

        self.get_logger().info("Miyformer node has been started!")

    def setup_pytorch(self):
        """Initialize PyTorch and display information"""
        if TORCH_AVAILABLE:
            self.get_logger().info("PyTorch is available!")
            self.get_logger().info(f"PyTorch version: {torch.__version__}")

            # Create a simple tensor
            self.tensor = torch.tensor([1.0, 2.0, 3.0])
            self.get_logger().info(f"Created tensor: {self.tensor}")

            # Check if CUDA is available (though we're using CPU)
            if torch.cuda.is_available():
                self.get_logger().info("CUDA is available but using CPU version")
            else:
                self.get_logger().info("Using CPU version of PyTorch")
        else:
            self.get_logger().warn("PyTorch is not available! Please install it.")
            self.tensor = None

    def timer_callback(self):
        """Timer callback to publish messages"""
        msg = String()

        if TORCH_AVAILABLE and self.tensor is not None:
            # Perform a simple operation with PyTorch
            tensor_sum = torch.sum(self.tensor).item()
            msg.data = f"Hello World from Miyformer! PyTorch tensor sum: {tensor_sum}"
        else:
            msg.data = f"Hello World from Miyformer! (PyTorch not available)"

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
