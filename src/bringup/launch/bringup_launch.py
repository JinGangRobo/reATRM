# Copyright 2025 Lihan Chen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import (
    IfCondition,
    LaunchConfigurationEquals,
    LaunchConfigurationNotEquals,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
    AnyLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml
from launch.actions import TimerAction, Shutdown


node_params = os.path.join(
    get_package_share_directory("atrm_bringup"), "config", "node_params.yaml"
)


def generate_launch_description():
    # Get the launch directory

    core_communicate_node = Node(
        package="rm_serial_driver",
        executable="rm_serial_driver_node",
        name="serial_driver",
        namespace="red_standard_robot1",
        output="both",
        emulate_tty=True,
        parameters=[node_params],
        on_exit=Shutdown(),
        ros_arguments=["--ros-args", "--log-level", "serial_driver:=INFO"],
    )

    delay_core_communicate_node = TimerAction(
        period=1.5,
        actions=[core_communicate_node],
    )

    debug_fox_node = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                os.path.join(get_package_share_directory("foxglove_bridge"), "launch"),
                "foxglove_bridge_launch.xml",
            )
        ),
        launch_arguments={"port": "8765", "max_qos_depth": "500"}.items(),
    )
    core_description_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                os.path.join(
                    get_package_share_directory("pb2025_robot_description"), "launch"
                ),
                "robot_description_launch.py",
            )
        ),
        launch_arguments={
            "namespace": "red_standard_robot1",
            "enable_joint_pub": "True",
            "use_rviz": "False",
        }.items(),
    )

    gui_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        # namespace="red_standard_robot1",
        output="screen",
    )

    task_vision_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                os.path.join(
                    get_package_share_directory("rm_vision_bringup"), "launch"
                ),
                "vision_bringup.launch.py",
            )
        ),
        launch_arguments={}.items(),
    )

    # Specify the actions
    bringup_cmd_group = GroupAction(
        [
            PushRosNamespace(namespace="red_standard_robot1"),
            SetRemap("/tf", "tf"),
            SetRemap("/tf_static", "tf_static"),
            # CORE UP
            delay_core_communicate_node,
            # gui_publisher_node,
            core_description_node,
            # DEBUG UP
            debug_fox_node,
            # TASK UP
            task_vision_node,
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)

    return ld
