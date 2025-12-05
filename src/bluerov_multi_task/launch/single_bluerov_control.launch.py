#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Declare arguments that will be passed from the main 'two_bluerovs' file
    # This allows this single file to adapt to bluerov1 OR bluerov2
    args = [
        DeclareLaunchArgument('robot_name', default_value='bluerov'),
        DeclareLaunchArgument('fcu_url', default_value='udp://127.0.0.1:14551@14555'),
        DeclareLaunchArgument('tgt_system', default_value='1'),
        # These coordinates help if you add localization later
        DeclareLaunchArgument('init_x', default_value='0.0'),
        DeclareLaunchArgument('init_y', default_value='0.0'),
    ]

    # 2. MAVROS Node (The Bridge to the Flight Controller)
    # Since we use 'PushRosNamespace' in the main file, this node 
    # will automatically start as /bluerov1/mavros/... or /bluerov2/mavros/...
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        # We remap the node name to be simple
        name='mavros', 
        parameters=[{
            'fcu_url': LaunchConfiguration('fcu_url'),
            'gcs_url': '', 
            'tgt_system': LaunchConfiguration('tgt_system'),
            'tgt_component': 1,
            'fcu_protocol': 'v2.0',
            'system_id': 1,
            'component_id': 1,
        }]
    )

    return LaunchDescription(args + [mavros_node])
