import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_depth = LaunchConfiguration('use_depth')
    return LaunchDescription([
        # Declarar el argumento 'use_depth' con valor por defecto 'true'
        DeclareLaunchArgument(
            'use_depth',
            default_value='true',
            description='Flag to indicate whether to use depth information.'
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory('bluerov2_cirtesu_core'), 'config', 'ekf_config.yaml')],
        ),
        Node(
            condition=IfCondition(use_depth),
            package='bluerov2_cirtesu_core',
            executable='depth_odom_publisher',
            name='depth_odom_publisher',
            output='screen',
        )
    ])

