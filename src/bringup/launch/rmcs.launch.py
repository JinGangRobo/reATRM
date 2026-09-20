from typing import List, Optional
import os

from launch import (
    LaunchContext,
    LaunchDescription,
    LaunchDescriptionEntity,
)
from launch.actions import LogInfo,DeclareLaunchArgument,IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

class MyLaunchDescriptionEntity(LaunchDescriptionEntity):
    def visit(
        self, context: "LaunchContext"
    ) -> Optional[List["LaunchDescriptionEntity"]]:
        entities = []

        robot_config = LaunchConfiguration("robot").perform(context)
        if robot_config.startswith("auto."):
            is_automatic = True
            robot_name = robot_config[5:]
        else:
            is_automatic = False
            robot_name = robot_config

        entities.append(
            LogInfo(
                msg=f"Starting RMCS on robot '{robot_config}'{'(automatic)' if is_automatic else ''} -> {robot_name}.yaml"
            )
        )

        enable_vision_entities = ["dual-sentry", "dual-infantry", "mec-hero", "mini-infantry"]

        entities.append(
            Node(
                package="rmcs_executor",
                executable="rmcs_executor",
                parameters=[
                    os.path.join(
                        FindPackageShare("atrm_bringup").perform(context),
                        "config",
                        robot_name + ".yaml",
                    ),
                ],
                respawn=True,
                respawn_delay=1.0,
                output="log",  # stdout and stderr are logged to launch log file and stderr to the screen.
            )
        )
        demo_launch_path = os.path.join(
            FindPackageShare("arm_moveit_config").perform(context),
            "launch",
            "demo.launch.py"
        )
        # TODO: Better way to identify robots needs vision capabilities
        if robot_name in enable_vision_entities:
            entities.append(
                Node(
                    package="rmcs_auto_aim_v2",
                    executable="rmcs_auto_aim_v2_runtime",
                    respawn=True,
                    respawn_delay=1.0,
                    output="screen",
                )
            )
        else:
            entities.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(demo_launch_path),
                launch_arguments={
                    "use_rviz": LaunchConfiguration("use_rviz")
                }.items(),
            )
        )


        if is_automatic:
            pass

        return entities


def generate_launch_description():
    ld = LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            MyLaunchDescriptionEntity()
        ]
    )

    return ld
