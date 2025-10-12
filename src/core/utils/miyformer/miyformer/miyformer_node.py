#!/usr/bin/env python3
"""
Miyformer ROS2 Node - A simple demonstration node with PyTorch integration
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo, Image
from auto_aim_interfaces.msg import Target
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped

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
        self.target_pub = self.create_publisher(Target, "tracker/target", 3)
        self.dbg_img_pub = self.create_publisher(Image, "dbg_image", 10)

        # Initialize TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

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

        self.camera_info = None

        self.get_logger().info("Miyformer node has been started!")

    def camera_info_callback(self, msg):
        """Callback function for camera info subscriber"""
        # self.get_logger().info(f"Received camera info: {msg}")
        self.camera_info = msg

    def solve_pnp(self, camera_info, target_point):
        """Solve PnP to get the 3D position of the target point"""
        fx = camera_info.k[0]
        fy = camera_info.k[4]
        cx = camera_info.k[2]
        cy = camera_info.k[5]

        # Assuming a fixed depth for simplicity
        Z = 1.0  # meters
        X = (target_point[0] - cx) * Z / fx
        Y = (target_point[1] - cy) * Z / fy

        return np.array([X, Y, Z])

    def camera_image_callback(self, msg: Image):
        """Callback function for camera image subscriber"""
        # temporary var
        DEBUG = True

        target_msg = Target()

        start_time = self.get_clock().now()
        dgb_img, results, tracking = self.mixformer_tracker.run(
            ros_image_to_cv2(msg), debug=True
        )
        infer_time = (self.get_clock().now() - start_time).nanoseconds / 1e6

        if DEBUG:
            pub_time = 0.0
            start_time = self.get_clock().now()
            msg_out = cv2_to_ros_image(dgb_img, encoding="bgr8")
            pub_time = (self.get_clock().now() - start_time).nanoseconds / 1e6

            self.dbg_img_pub.publish(msg_out)
            # self.get_logger().info(
            #     f"Received camera image: {msg_out.width}x{msg_out.height}, infer_time: {infer_time:.2f} ms, pub_time: {pub_time:.2f} ms, tracking: {tracking}, results: {results[-1] if results else 'None'}"
            # )

        target_box: list = results[-1]
        if sum(target_box) > 0 and tracking:
            target_point = (
                int(target_box[0] + target_box[2] / 2),
                int(target_box[1] + target_box[3] / 2),
            )

            if self.camera_info is not None:
                position_3d_cam = self.solve_pnp(self.camera_info, target_point)

                # 通过TF变换将以相机为坐标系的位置转换为机器人的gimbal_sub_yaw_odom坐标系
                try:
                    # 创建相机坐标系下的点
                    point_cam = PointStamped()
                    point_cam.header.stamp = self.get_clock().now().to_msg()
                    point_cam.header.frame_id = "industrial_camera"  # 相机光学坐标系
                    point_cam.point.x = float(position_3d_cam[2])
                    point_cam.point.y = float(position_3d_cam[0])
                    point_cam.point.z = float(position_3d_cam[1])

                    # 转换到目标坐标系
                    transform = self.tf_buffer.lookup_transform(
                        "gimbal_sub_yaw_odom",  # 目标坐标系
                        "industrial_camera",  # 源坐标系
                        rclpy.time.Time(),  # 最新的变换
                        timeout=rclpy.duration.Duration(seconds=1.0),
                    )

                    # 应用变换
                    point_gimbal = tf2_geometry_msgs.do_transform_point(
                        point_cam, transform
                    )

                    # 发布目标消息

                    target_msg.header.stamp = self.get_clock().now().to_msg()
                    target_msg.header.frame_id = "gimbal_sub_yaw_odom"
                    target_msg.position.x = point_gimbal.point.x
                    target_msg.position.y = point_gimbal.point.y
                    target_msg.position.z = point_gimbal.point.z
                    target_msg.tracking = tracking
                    target_msg.distance_valid = False

                    self.target_pub.publish(target_msg)

                    if DEBUG:
                        self.get_logger().info(
                            f"3D Position (camera): {position_3d_cam}"
                        )
                        self.get_logger().info(
                            f"3D Position (gimbal): [{point_gimbal.point.x:.3f}, {point_gimbal.point.y:.3f}, {point_gimbal.point.z:.3f}]"
                        )

                except (
                    tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException,
                ) as e:
                    self.get_logger().warn(f"TF transform failed: {e}")
                    if DEBUG:
                        self.get_logger().info(
                            f"3D Position (camera only): {position_3d_cam}"
                        )
        else:
            target_msg.header.stamp = self.get_clock().now().to_msg()
            target_msg.header.frame_id = "gimbal_sub_yaw_odom"
            target_msg.tracking = False
            target_msg.distance_valid = False

            self.target_pub.publish(target_msg)


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
