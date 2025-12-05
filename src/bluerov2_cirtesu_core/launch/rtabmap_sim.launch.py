import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        # Incluir el launch de rtabmap con parámetros directos
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('rtabmap_launch'),
                    'launch',
                    'rtabmap.launch.py'
                )
            ),
            launch_arguments={
                'stereo':                'false',  # Use stereo input instead of RGB-D
                'localization':          'false',  # Launch in localization mode
                'rtabmap_viz':           'true',   # Launch RTAB-Map UI
                'rviz':                  'false',  # Launch RVIZ
                'use_sim_time':          'false',  # Use simulation clock
                'log_level':             'info',   # ROS logging level
                'cfg':                   '',       # Path of RTAB-Map config file
                'gui_cfg':               '~/.ros/rtabmap_gui.ini',  # Config path for rtabmap_viz
                # 'rviz_cfg':              '/home/bluerov/bluerov_ws/src/bluerov2_cirtesu_core/config/rtabmap.rviz',  # Config path for RVIZ
                'frame_id':              'bluerov/base_link',  # Fixed frame ID of the robot
                'odom_frame_id':         'world_ned',  # TF for odometry
                'map_frame_id':          'world_ned',  # Output map frame ID
                'map_topic':             'map',  # Map topic name
                'publish_tf_map':        'false',  # Publish TF between map and odometry
                'namespace':             'rtabmap',  # Namespace for nodes
                'database_path':         '~/.ros/rtabmap.db',  # Database path
                'topic_queue_size':      '1',  # Queue size for topic subscribers
                'queue_size':            '10',  # Backward compatibility for sync_queue_size
                'qos':                   '1',  # QoS for sensor data
                'wait_for_transform':    '0.2',  # Wait for TF transform
                'rtabmap_args':          '',  # Backward compatibility for args
                'launch_prefix':         '',  # Debugging prefix for nodes
                'output':                'screen',  # Node output (screen or log)
                'initial_pose':          '',  # Initial pose (in localization mode)
                'output_goal_topic':     '/goal_pose',  # Output goal topic (for nav2)
                'use_action_for_goal':   'false',  # Connect to nav2 action server
                'ground_truth_frame_id': '',  # Frame ID for ground truth
                'ground_truth_base_frame_id': '',  # Base frame ID for ground truth
                'approx_sync':           'true',  # Approximate sync for input topics
                'approx_sync_max_interval': '0.0',  # Max interval for approx sync
                'rgb_topic':             '/bluerov/image_color',  # RGB topic
                'depth_topic':           '/bluerov/depth_camera/image_depth',  # Depth topic
                'camera_info_topic':     '/bluerov/depth_camera/camera_info',  # Camera info topic
                'stereo_namespace':      '/stereo_camera',  # Stereo namespace
                'left_image_topic':      '/stereo_camera/left/image_rect_color',  # Left image topic
                'right_image_topic':     '/stereo_camera/right/image_rect',  # Right image topic (grayscale)
                'left_camera_info_topic': '/stereo_camera/left/camera_info',  # Left camera info topic
                'right_camera_info_topic': '/stereo_camera/right/camera_info',  # Right camera info topic
                'rgbd_sync':             'false',  # Pre-sync RGB-D topics
                'approx_rgbd_sync':      'true',  # Approximate sync for RGB-D
                'subscribe_rgbd':        'false',  # Subscribe to RGB-D topic
                'rgbd_topic':            'rgbd_image',  # RGB-D topic name
                'depth_scale':           '1.0',  # Depth scale
                'compressed':            'false',  # Subscribe to compressed image topics
                'rgb_image_transport':   'compressed',  # Image transport for RGB
                'depth_image_transport': 'compressedDepth',  # Image transport for depth
                'subscribe_scan':        'false',  # Subscribe to 2D scan
                'scan_topic':            '/scan',  # Scan topic
                'subscribe_scan_cloud':  'false',  # Subscribe to 3D scan cloud
                'scan_cloud_topic':      '/points',  # Scan cloud topic
                'scan_normal_k':         '0',  # Normal estimation for scan points
                'visual_odometry':       'false',  # Launch visual odometry node
                'icp_odometry':          'false',  # Launch ICP odometry node
                'odom_topic':            '/bluerov/navigator/odometry',  # Odometry topic
                #'odom_topic':            '/zed/microstrain_inertial_driver/odom',  # Odometry topic
                'vo_frame_id':           'world_ned',  # Frame ID for visual/ICP odometry
                'publish_tf_odom':       'false',  # Publish TF for odometry
                'odom_tf_angular_variance': '0.01',  # Angular variance for TF odometry
                'odom_tf_linear_variance': '0.001',  # Linear variance for TF odometry
                'odom_args':             '',  # Additional odometry arguments
                'odom_sensor_sync':      'false',  # Sensor sync for odometry
                'odom_guess_frame_id':   '',  # Frame ID for odometry guess
                'odom_guess_min_translation': '0.0',  # Min translation for odometry guess
                'odom_guess_min_rotation': '0.0',  # Min rotation for odometry guess
                #'imu_topic':             '/zed/microstrain_inertial_driver/imu/data',  # IMU topic
                'imu_topic':             'bluerov/imu/data',  # IMU topic
                'wait_imu_to_init':      'false',  # Wait for IMU data to initialize
                'subscribe_user_data':   'false',  # Subscribe to user data
                'user_data_topic':       '/user_data',  # User data topic
                'user_data_async_topic': '/user_data_async',  # Async user data topic
                'gps_topic':             '/gps/fix',  # GPS topic
                'tag_topic':             '/detections',  # AprilTag topic
                'tag_linear_variance':   '0.0001',  # Linear variance for tags
                'tag_angular_variance':  '9999.0',  # Angular variance for tags
                'fiducial_topic':        '/fiducial_transforms'  # Aruco tag topic
            }.items(),
        ),
    ])
