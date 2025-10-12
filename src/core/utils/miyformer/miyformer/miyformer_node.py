#!/usr/bin/env python3
"""
Miyformer ROS2 Node - A simple demonstration node with PyTorch integration
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo, Image

import cv2 as cv
import numpy as np

# init mixformer path
import os
import sys


def ros_image_to_cv2(msg: Image):
    """
    手动将 ROS Image 消息转换为 OpenCV Mat 格式
    """
    # 获取图像数据
    height = msg.height
    width = msg.width
    encoding = msg.encoding
    data = msg.data

    # 根据编码格式确定数据类型和通道数
    if encoding == "mono8" or encoding == "8UC1":
        dtype = np.uint8
        channels = 1
    elif encoding == "mono16" or encoding == "16UC1":
        dtype = np.uint16
        channels = 1
    elif encoding == "bgr8":
        dtype = np.uint8
        channels = 3
    elif encoding == "rgb8":
        dtype = np.uint8
        channels = 3
    elif encoding == "bgra8":
        dtype = np.uint8
        channels = 4
    elif encoding == "rgba8":
        dtype = np.uint8
        channels = 4
    elif encoding == "32FC1":
        dtype = np.float32
        channels = 1
    else:
        raise ValueError(f"Unsupported encoding: {encoding}")

    # 将字节数据转换为 numpy 数组
    img_array = np.frombuffer(data, dtype=dtype)

    # 重塑为图像矩阵
    if channels == 1:
        cv_image = img_array.reshape((height, width))
    else:
        cv_image = img_array.reshape((height, width, channels))

    # 如果是 RGB 格式，需要转换为 BGR（OpenCV 默认格式）
    if encoding == "rgb8":
        cv_image = cv.cvtColor(cv_image, cv.COLOR_RGB2BGR)
    elif encoding == "rgba8":
        cv_image = cv.cvtColor(cv_image, cv.COLOR_RGBA2BGR)

    return cv_image


def cv2_to_ros_image(cv_image: cv.Mat, encoding="bgr8"):
    """
    将 OpenCV Mat 转换为 ROS Image 消息
    """
    msg = Image()

    if len(cv_image.shape) == 2:  # 灰度图
        height, width = cv_image.shape
        channels = 1
    else:  # 彩色图
        height, width, channels = cv_image.shape

    msg.height = height
    msg.width = width
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = width * channels * cv_image.dtype.itemsize

    # 如果需要从 BGR 转换为 RGB
    if encoding == "rgb8" and channels == 3:
        cv_image = cv.cvtColor(cv_image, cv.COLOR_BGR2RGB)

    msg.data = cv_image.tobytes()

    return msg


mixformer_path = os.path.join(os.path.dirname(__file__), "mixformer")
if mixformer_path not in sys.path:
    sys.path.append(mixformer_path)

from lib.test.evaluation import TrackerOnFrame


class MiyformerNode(Node):
    """
    A simple ROS2 node that demonstrates PyTorch integration
    """

    def __init__(self):
        super().__init__("miyformer_node")

        # Create camera topic subscriber
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "camera_info",
            self.camera_info_callback,
            qos_profile_sensor_data,
        )
        self.camera_image_sub = self.create_subscription(
            Image, "image_raw", self.camera_image_callback, qos_profile_sensor_data
        )

        # Create a publisher
        self.publisher_ = self.create_publisher(String, "miyformer_topic", 10)
        self.dbg_img_pub = self.create_publisher(Image, "dbg_image", 10)

        self.mixformer_tracker = TrackerOnFrame(
            "mixformer2_vit_online",
            "224_depth4_mlp1_score",
            tracker_params={
                "model": "models/mixformerv2_small.pth.tar",
                "update_interval": 25,
                "online_size": 1,
                "search_area_scale": 4.5,
                "max_score_decay": 1.0,
                "vis_attn": 0,
            },
        )

        self.get_logger().info("Miyformer node has been started!")

    def camera_info_callback(self, msg):
        """Callback function for camera info subscriber"""
        # self.get_logger().info(f"Received camera info: {msg}")

    def camera_image_callback(self, msg: Image):
        """Callback function for camera image subscriber"""
        # temporary var
        DEBUG = True
        
        start_time = self.get_clock().now()
        dgb_img, results, tracking = self.mixformer_tracker.run(
            ros_image_to_cv2(msg), debug=True
        )
        infer_time = (self.get_clock().now() - start_time).nanoseconds / 1e6

        if DEBUG:
            pub_time = 0.0
            start_time = self.get_clock().now()
            msg = cv2_to_ros_image(dgb_img, encoding="bgr8")
            pub_time = (self.get_clock().now() - start_time).nanoseconds / 1e6

            self.dbg_img_pub.publish(msg)

        self.get_logger().info(
            f"Received camera image: {msg.width}x{msg.height}, infer_time: {infer_time:.2f} ms, pub_time: {pub_time:.2f} ms, tracking: {tracking}, results: {results[-1]}"
        )


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
