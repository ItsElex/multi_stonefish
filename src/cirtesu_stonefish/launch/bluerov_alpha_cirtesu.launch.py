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
        default_value='bluerov',
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
                        FindPackageShare('cirtesu_stonefish'), 'scenarios', 'cirtesu','bluerov2_heavy_cirtesu_tank.scn'
                    ]),
                    'simulation_rate': '50.0',
                    'window_res_x': '1200',
                    'window_res_y': '800',
                    'rendering_quality': 'high'
                }.items()
            ),
        ]
    )
    core_sim = os.path.join(get_package_share_directory('bluerov2_cirtesu_core'), 'launch', 'core_alpha_sim.launch.py')
    mav2ros = os.path.join(get_package_share_directory('bluerov_mav2ros'), 'launch', 'bluerov_mav2ros.launch.py')

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
            package='cirtesu_stonefish',
            namespace='bluerov',
            executable='ardusim_patch.py',
            name='ardusim_patch',
            output='screen',
            emulate_tty='true',
        ),

        # Launch core_sim
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(core_sim),
            launch_arguments={}.items()  # Aquí puedes añadir argumentos si es necesario
        ),

        # Launch bluerov_mav2ros
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mav2ros),
            launch_arguments={}.items()  # Aquí puedes añadir argumentos si es necesario
        ),
 
        ])


