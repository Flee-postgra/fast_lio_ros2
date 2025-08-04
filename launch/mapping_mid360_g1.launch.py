#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Whether to start rviz'
    )
    
    # Get package directories
    fast_lio_dir = get_package_share_directory('fast_lio')
    config_file = os.path.join(fast_lio_dir, 'config', 'mid360_g1.yaml')
    rviz_config = os.path.join(fast_lio_dir, 'rviz_cfg', 'loam_livox.rviz')
    
    # Fast LIO node
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fast_lio_node',
        output='screen',
        parameters=[config_file],
        arguments=[]
    )
    
    # RViz node (conditional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    return LaunchDescription([
        rviz_arg,
        fast_lio_node,
        rviz_node
    ]) 