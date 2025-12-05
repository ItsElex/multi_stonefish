import os
from launch import LaunchDescription
from launch.actions import GroupAction, DeclareLaunchArgument, LogInfo
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition 


def generate_launch_description():
    # Rutas a los archivos de lanzamiento que se van a incluir
    zed_file = os.path.join(get_package_share_directory('zed_wrapper'), 'launch', 'zed_camera.launch.py')
    ahrs_file = os.path.join(get_package_share_directory('bluerov2_cirtesu_core'), 'launch', 'ahrs.launch.py')
    robot_description = os.path.join(get_package_share_directory('bluerov2_cirtesu_core'), 'launch', 'robot_description.launch.py')
    localization_file = os.path.join(get_package_share_directory('bluerov2_cirtesu_core'), 'launch', 'localization.launch.py')

    # Utilizar rviz
    use_rviz = LaunchConfiguration('use_rviz')

    description_file = os.path.join(
        get_package_share_directory('blue_description'), 
        'description', 
        'bluerov2_heavy', 
        'config.xacro'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',  # Default to true if not provided
            description='Flag to enable RViz'
        ),
        
        # Log info to confirm RViz launch
        LogInfo(
            condition=IfCondition(use_rviz),
            msg="Launching RViz..."
        ),
        #lanzar rviz
        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(get_package_share_directory('bluerov2_cirtesu_core'), 'config', 'bluerov.rviz')]
        ),
        
        # Lanzar robot descriptions
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_description),
            launch_arguments={'use_tank': 'true',}.items()  # Aquí puedes añadir argumentos si es necesario
        ),
        
        # Lanzar AHRS
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ahrs_file),
            launch_arguments={}.items()  # Aquí puedes añadir argumentos si es necesario
        ),
        #lanzar localization filter
        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource(localization_file),
        #    launch_arguments={}.items()  # Aquí puedes añadir argumentos si es necesario
        #),

        # Incluir el segundo archivo de lanzamiento
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(zed_file),
            launch_arguments={
                'camera_model': 'zed2',
                'pos_tracking_enabled': 'true',
                'publish_tf': 'false',
                'publish_map_tf': 'false',
                'map_frame': 'map',
                'odometry_frame': 'odom', 
            }.items()
        ),
        
    ])

