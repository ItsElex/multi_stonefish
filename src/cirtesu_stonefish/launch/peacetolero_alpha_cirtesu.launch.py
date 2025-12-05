import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Declare launch arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='peacetolero',
        description='Name of the robot'
    )

    # Group action with namespace
    namespace_action = GroupAction(
        actions=[
            # Include the simulator launch file
            IncludeLaunchDescription(
                PathJoinSubstitution([
                    FindPackageShare('stonefish_ros2'), 'launch', 'stonefish_simulator.launch.py'
                ]),
                launch_arguments={
                    'simulation_data': PathJoinSubstitution([
                        FindPackageShare('cirtesu_stonefish'), 'data'
                    ]),
                    'scenario_desc': PathJoinSubstitution([
                        FindPackageShare('cirtesu_stonefish'), 'scenarios', 'cirtesu','peacetolero_cirtesu_tank.scn'
                    ]),
                    'simulation_rate': '50.0',
                    'window_res_x': '1200',
                    'window_res_y': '800',
                    'rendering_quality': 'high'
                }.items()
            ),
        ]
    )

    # Obtener la ruta del archivo .xacro
    description_file_cirtesu = os.path.join(
        get_package_share_directory('bluerov2_cirtesu_core'),
        'urdf',
        'cirtesu',
        'cirtesu.urdf.xacro'
    )

    # Procesar el archivo xacro para generar el robot_description
    robot_description_cirtesu = os.popen(f'xacro {description_file_cirtesu}').read().strip()
        # core_sim = os.path.join(get_package_share_directory('bluerov2_cirtesu_core'), 'launch', 'core_sim.launch.py')
        # mav2ros = os.path.join(get_package_share_directory('bluerov_mav2ros'), 'launch', 'bluerov_mav2ros.launch.py')

    return LaunchDescription([
        robot_name_arg,
        namespace_action
    ,
        # Node(
        #     package='joy',
        #     executable='joy_node',
        #     output='screen',
        # ),
        # Node(
        #     package='cirtesu_stonefish',
        #     executable='bluerov2_logitechF310teleop.py',
        #     output='screen',
        # ),
            Node(
            package='cirtesu_stonefish',
            executable='odom2tf.py',
            output='screen',
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                "--x", "0",
                "--y", "0",
                "--z", "0.0",
                "--roll", "0.0",
                "--pitch", "3.1416",
                "--yaw", "0",
                "--frame-id", "world_ned",
                "--child-frame-id", "cirtesu_tank"
            ]
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            name='robot_state_publisher_cirtesu',
            remappings=[('/robot_description', '/cirtesu/robot_description')],
            parameters=[
                {'robot_description': robot_description_cirtesu}
            ]
        ),

        # Launch bluerov_mav2ros
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(mav2ros),
        #     launch_arguments={}.items()  # Aquí puedes añadir argumentos si es necesario
        # ),
        # Launch moveit_planning
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         PathJoinSubstitution([
        #             FindPackageShare("peacetolero_control"),
        #             "launch",
        #             "moveit_planning.launch.py"
        #         ])
        #     ),
        #     launch_arguments={}.items()  # Aquí puedes añadir argumentos si es necesario
        # ),
 
        ])


