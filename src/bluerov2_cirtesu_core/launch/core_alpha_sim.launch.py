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
    ahrs_file = os.path.join(get_package_share_directory('bluerov2_cirtesu_core'), 'launch', 'ahrs.launch.py')
    robot_description = os.path.join(get_package_share_directory('bluerov2_cirtesu_core'), 'launch', 'robot_description_alpha.launch.py')
    alpha = os.path.join(get_package_share_directory('bluerov2_cirtesu_core'), 'launch', 'alpha.launch.py')

    # Utilizar rviz
    use_rviz = LaunchConfiguration('use_rviz')
    # Utilizar point_cloud
    use_point_cloud = LaunchConfiguration('use_point_cloud')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_point_cloud',
            default_value='false',  # Default to true if not provided
            description='Flag to enable point cloud'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',  # Default to true if not provided
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
            arguments=['-d', os.path.join(get_package_share_directory('bluerov2_cirtesu_core'), 'config', 'bluerov_sim.rviz')]
        ),
        
        # Lanzar robot descriptions
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_description),
            launch_arguments={'use_tank': 'true',}.items()  # Aquí puedes añadir argumentos si es necesario
        ),

        # Lanzar alpha_driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_description),
            launch_arguments={'use_tank': 'true',}.items()  # Aquí puedes añadir argumentos si es necesario
        ),

        #Crear point clouds
        Node(
            condition=IfCondition(use_point_cloud),
            package='depth_image_proc',
            executable='point_cloud_xyzrgb_node',
            name='point_cloud_xyzrgb',
            output='screen',
            remappings=[
                ('/rgb/image_rect_color', '/bluerov/image_color'),
                ('/depth_registered/image_rect', '/bluerov/depth_camera/image_depth'),
                ('/rgb/camera_info', '/bluerov/depth_camera/camera_info'),
            ]
        ),

        # lanzar alpha
        IncludeLaunchDescription(
           PythonLaunchDescriptionSource(alpha),
           launch_arguments={}.items()  # Aquí puedes añadir argumentos si es necesario
        ),

        
    ])

