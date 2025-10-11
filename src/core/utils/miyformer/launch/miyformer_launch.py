#!/usr/bin/env python3
"""
Launch file for Miyformer node
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for miyformer node"""
    
    miyformer_node = Node(
        package='miyformer',
        executable='miyformer_node',
        name='miyformer_node',
        output='screen',
        parameters=[
            # Add any parameters here if needed
        ]
    )
    
    return LaunchDescription([
        miyformer_node
    ])