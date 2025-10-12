import os
import sys
from ament_index_python.packages import get_package_share_directory

sys.path.append(
    os.path.join(get_package_share_directory("rm_vision_bringup"), "launch")
)


def generate_launch_description():
    from common import node_params, launch_params
    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer, Node
    from launch.actions import Shutdown
    from launch import LaunchDescription

    def get_camera_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name="camera_node",
            parameters=[node_params],
            extra_arguments=[{"use_intra_process_comms": True}],
        )

    detectors = {
        "miyformer": None,
        "common": ComposableNode(
            package="armor_detector",
            plugin="rm_auto_aim::ArmorDetectorNode",
            name="armor_detector",
            parameters=[node_params],
            extra_arguments=[{"use_intra_process_comms": True}],
        ),
    }
    trackers = {
        "miyformer": None,
        "common": Node(
            package="armor_tracker",
            executable="armor_tracker_node",
            output="both",
            namespace="",
            emulate_tty=True,
            parameters=[node_params],
            ros_arguments=[
                "--log-level",
                "armor_tracker:=" + launch_params["tracker_log_level"],
            ],
            remappings=[
                ("/tf", "tf"),
                ("/tf_static", "tf_static"),
            ],
        ),
    }

    def get_camera_detector_container(camera_node):
        nodes = [
            camera_node,
            ComposableNode(
                package="rmoss_projectile_motion",
                plugin="rmoss_projectile_motion::ProjectileSolverNode",
                name="rmoss_projectile_motion",
                # parameters=[],
            ),
        ]

        detector_node = detectors.get(launch_params["work_mode"], detectors["common"])
        if detector_node != None:
            nodes.append(detector_node)

        return ComposableNodeContainer(
            name="camera_detector_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=nodes,
            output="both",
            emulate_tty=True,
            ros_arguments=[
                "--ros-args",
                "--log-level",
                "armor_detector:=" + launch_params["detector_log_level"],
            ],
            on_exit=Shutdown(),
        )

    hik_camera_node = get_camera_node(
        "hik_camera_ros2_driver", "hik_camera_ros2_driver::HikCameraRos2DriverNode"
    )
    mv_camera_node = get_camera_node(
        "mindvision_camera", "mindvision_camera::MVCameraNode"
    )

    tracker_node = trackers.get(launch_params["work_mode"], trackers["common"])

    # miyformer_node = N

    if launch_params["camera"] == "hik":
        cam_detector = get_camera_detector_container(hik_camera_node)
    elif launch_params["camera"] == "mv":
        cam_detector = get_camera_detector_container(mv_camera_node)

    launch_nodes = [
        cam_detector,
    ]

    if tracker_node != None:
        launch_nodes.append(tracker_node)

    return LaunchDescription(launch_nodes)
