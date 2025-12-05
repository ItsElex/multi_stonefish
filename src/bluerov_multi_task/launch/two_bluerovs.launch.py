#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import PushRosNamespace, Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_bluerov = FindPackageShare('bluerov_multi_task')
    pkg_stonefish = FindPackageShare('stonefish_ros2')
    pkg_data = FindPackageShare('cirtesu_stonefish')

    # 1. Stonefish Simulator (The Tiny World)
    stonefish_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_stonefish, 'launch', 'stonefish_simulator.launch.py'])
        ]),
        launch_arguments={
            'simulation_data': PathJoinSubstitution([pkg_data, 'data']),
            'scenario_desc': PathJoinSubstitution([pkg_bluerov, 'scenarios', 'two_bluerovs.scn']),
            'simulation_rate': '100.0',
            'window_res_x': '1200',
            'window_res_y': '900',
        }.items()
    )

    # Path to the generic MAVROS/Connection file
    single_control_path = PathJoinSubstitution([pkg_bluerov, 'launch', 'single_bluerov_control.launch.py'])

    # --- ROBOT 1: The LEADER (Joystick Controlled) ---
    bluerov1_stack = GroupAction([
        PushRosNamespace('bluerov1'),
        
        # A. Connection to SITL (Port 14551)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(single_control_path),
            launch_arguments={
                'robot_name': 'bluerov1', 
                'fcu_url': 'udp://127.0.0.1:14551@14555'
            }.items()
        ),

        # B. Teleop Node (Joy -> /bluerov1/mavros/setpoint_velocity/cmd_vel_unstamped)
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[{
                'scale_linear': 0.8, 
                'scale_angular': 0.5,
                'require_enable_button': False 
            }],
            remappings=[
                ('/cmd_vel', '/bluerov1/mavros/setpoint_velocity/cmd_vel_unstamped'),
                ('/joy', '/joy') 
            ]
        )
    ])

    # --- ROBOT 2: The FOLLOWER (Passive / Waiting) ---
    bluerov2_stack = GroupAction([
        PushRosNamespace('bluerov2'),

        # Connection to SITL (Port 14561)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(single_control_path),
            launch_arguments={
                'robot_name': 'bluerov2', 
                'fcu_url': 'udp://127.0.0.1:14561@14565'
            }.items()
        ),
    ])

    # --- GLOBAL NODES ---
    
    # 1. Joy Node (Driver)
    joy_driver = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'coalesce_interval': 0.02}] 
    )

    # 2. Joystick Manager (Buttons -> Arming/Modes)
    # UPDATED: Now pointing to 'joystick.py'
    joy_manager = Node(
        package='bluerov_multi_task',
        executable='joystick', # <--- UPDATED HERE
        name='xbox_manager',
        output='screen'
    )

    return LaunchDescription([
        stonefish_sim,
        joy_driver,     
        joy_manager,    
        bluerov1_stack, 
        bluerov2_stack
    ])
