import os

import ament_index_python

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

# Path to the launch files and directories that we will use
_MICROSTRAIN_LAUNCH_FILE = os.path.join(ament_index_python.packages.get_package_share_directory('microstrain_inertial_driver'), 'launch', 'microstrain_launch.py')
_CV7_PARAMS_FILE = os.path.join(ament_index_python.packages.get_package_share_directory('bluerov2_cirtesu_core'), 'config', 'cv7.yml')

def generate_launch_description():
    return LaunchDescription([
        # Microstrain node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_MICROSTRAIN_LAUNCH_FILE),
            launch_arguments={
                'configure': 'true',
                'activate': 'true',
                'params_file': _CV7_PARAMS_FILE,
                'namespace': '/',
            }.items()
        ),

        # Publish a static transform for where the CV7 is mounted on base_link.
        # Unless the CV7 is mounted exactly at base_link, you should change this to be accurate to your setup
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                "--x", "0",
                "--y", "-0.0083",
                "--z", ".043",
                "--roll", "0.0",
                "--pitch", "0",
                "--yaw", "-1.5708",
                "--frame-id", "zed_camera_link",
                "--child-frame-id", "cv7_link"
            ]
        ),
        
        # Removed the RViz node
    ])

