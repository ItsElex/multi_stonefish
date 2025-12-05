from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    robot_name = LaunchConfiguration('robot_name', default='bluerov')

    # Stonefish simulator launch with BlueROV scenario
    stonefish_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('stonefish_ros2'), 'launch', 'stonefish_simulator.launch.py'
        ]),
        launch_arguments={
            'simulation_data': PathJoinSubstitution([
                FindPackageShare('cirtesu_stonefish'), 'data'
            ]),
            'scenario_desc': PathJoinSubstitution([
                FindPackageShare('cirtesu_stonefish'), 'scenarios', 'cirtesu', 'bluerov2_heavy_cirtesu_tank.scn'
            ]),
            'simulation_rate': '100.0',
            'window_res_x': '1200',
            'window_res_y': '800',
            'rendering_quality': 'high'
        }.items()
    )

    # Core BlueROV simulation launch
    core_sim_path = os.path.join(get_package_share_directory('bluerov2_cirtesu_core'), 'launch', 'core_sim.launch.py')
    core_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(core_sim_path),
        launch_arguments={}.items()
    )

    # bluerov_mav2ros nodes launching under namespace
    param_file_path = os.path.join(get_package_share_directory('bluerov_mav2ros'), 'param', 'stonefish_bluerov.params')

    mav2ros_nodes_group = GroupAction(actions=[
        Node(
            package='bluerov_mav2ros',
            executable='node.py',
            name='bluerov_mav2ros',
            namespace=robot_name,
            output='screen',
            parameters=[param_file_path]
        ),
        Node(
            package='bluerov_mav2ros',
            executable='cmd_vel_map.py',
            name='cmd_vel_map',
            namespace=robot_name,
            output='screen',
            parameters=[param_file_path]
        ),
    ])

    # Launch ArduSub SITL simulator process with params
    sitl_exec = ExecuteProcess(
        cmd=[
            'sim_vehicle.py',
            '-v', 'ArduSub',
            '--model', 'JSON',
            '--map',
            '-L', 'CIRTESU',
            '-m',
            '--streamrate=-1',
            f"--add-param-file={param_file_path}"
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='bluerov', description='Name of the robot'),

        stonefish_launch,
        core_sim,
        mav2ros_nodes_group,
        sitl_exec,
    ])

