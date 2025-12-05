from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description() -> LaunchDescription:
    """
    Launch description to start only the controller_manager node.
    """
    # Declare launch arguments
    args = [
        DeclareLaunchArgument(
            "description_package",
            default_value="alpha_description",
            description="The description package with the Alpha URDF files."
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="alpha_stonefish.config.xacro",
            description="The URDF/XACRO description file with the Alpha."
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="alpha_controllers_stonefish.yaml",
            description="The YAML file defining the ros2_control controllers."
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="The namespace of the launched nodes."
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Start the driver using fake hardware mirroring command to its states."
        ),
    ]

    # Initialize launch configurations
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    controllers_file = LaunchConfiguration("controllers_file")
    namespace = LaunchConfiguration("namespace")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    # Generate robot description using xacro
    robot_description = {
        "robot_description": Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare(description_package),
                        "xacro",
                        description_file,
                    ]
                ),
                " ",
                "use_fake_hardware:=",
                use_fake_hardware,
            ]
        )
    }

    # Declare ROS 2 node: controller_manager
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        namespace=namespace,
        parameters=[
            robot_description,
            PathJoinSubstitution(
                [
                    FindPackageShare(description_package),
                    "config",
                    controllers_file,
                ]
            ),
        ],
    )

    rqt_node = Node(
            package="rqt_gui",
            executable="rqt_gui",
            name="rqt",
            remappings=[
                ("robot_description", "/bluerov/robot_description"),
            ],
        )

    return LaunchDescription(args + [control_node]+ [rqt_node])
