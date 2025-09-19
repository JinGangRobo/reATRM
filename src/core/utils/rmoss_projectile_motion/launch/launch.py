import os
import sys
from ament_index_python.packages import get_package_share_directory

sys.path.append(
    os.path.join(get_package_share_directory("rmoss_projectile_motion"), "launch")
)


def generate_launch_description():
    from launch_ros.actions import Node
    from launch import LaunchDescription

    detector_node = Node(
        package="rmoss_projectile_motion",
        executable="projectile_solver_node",
        output="screen",
        parameters=[],
        arguments=[],
    )

    return LaunchDescription(
        [
            # robot_state_publisher,
            detector_node,
        ]
    )
