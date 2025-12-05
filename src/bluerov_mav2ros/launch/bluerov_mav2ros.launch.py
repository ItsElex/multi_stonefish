from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import LogInfo
import os
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    param_file_path = os.path.join(
        get_package_share_directory('bluerov_mav2ros'),
        'param',
        'stonefish_bluerov.params'
    )
    
    # Verifica si el archivo existe
    if not os.path.isfile(param_file_path):
        raise FileNotFoundError(f"Parameter file not found: {param_file_path}")
    return LaunchDescription([
        LogInfo(msg=f"Param file path: {param_file_path}"),
        Node(
            package='bluerov_mav2ros',
            executable='node.py',
            name='bluerov_mav2ros',
            namespace='bluerov',
            output='screen'
        ),
        Node(
            package='bluerov_mav2ros',
            executable='cmd_vel_map.py',
            name='cmd_vel_map',
            namespace='bluerov',
            output='screen'
        ),
        # Lanzar el simulador de ArduPilot con el archivo de parámetros
        ExecuteProcess(
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
    ])

