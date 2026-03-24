import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Model path
    detection_share = get_package_share_directory('motherv2_detection')
    default_model = os.path.join(detection_share, 'models', 'efficientdet_lite0.tflite')

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('camera_device', default_value='0'),
        DeclareLaunchArgument('camera_width', default_value='640'),
        DeclareLaunchArgument('camera_height', default_value='360'),
        DeclareLaunchArgument('camera_orientation', default_value='0'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyAMA0'),
        DeclareLaunchArgument('serial_baud', default_value='115200'),
        DeclareLaunchArgument('min_speed', default_value='130'),
        DeclareLaunchArgument('max_speed', default_value='150'),
        DeclareLaunchArgument('model_path', default_value=default_model),
        DeclareLaunchArgument('conf_threshold', default_value='0.35'),
        DeclareLaunchArgument('target_bbox_ratio', default_value='0.25'),
        DeclareLaunchArgument('web_port', default_value='8080'),
        DeclareLaunchArgument('debug_class', default_value='-1'),

        # Camera Node (camera_ros - libcamera backend for IMX219)
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera_node',
            parameters=[{
                'camera': LaunchConfiguration('camera_device'),
                'width': LaunchConfiguration('camera_width'),
                'height': LaunchConfiguration('camera_height'),
                'orientation': LaunchConfiguration('camera_orientation'),
                'format': 'BGR888',
            }],
            remappings=[
                ('~/image_raw', '/motherv2/image_raw'),
            ],
            output='screen',
        ),

        # Detection Node
        Node(
            package='motherv2_detection',
            executable='detection_node',
            name='detection_node',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'conf_threshold': LaunchConfiguration('conf_threshold'),
                'debug_class': LaunchConfiguration('debug_class'),
                'stream_width': 640,
            }],
            output='screen',
        ),

        # Follower Node
        Node(
            package='motherv2_follower',
            executable='follower_node',
            name='follower_node',
            parameters=[{
                'image_width': LaunchConfiguration('camera_width'),
                'image_height': LaunchConfiguration('camera_height'),
                'max_speed': LaunchConfiguration('max_speed'),
                'target_bbox_ratio': LaunchConfiguration('target_bbox_ratio'),
            }],
            output='screen',
        ),

        # Serial Node
        Node(
            package='motherv2_serial',
            executable='serial_node',
            name='serial_node',
            parameters=[{
                'port': LaunchConfiguration('serial_port'),
                'baud': LaunchConfiguration('serial_baud'),
                'min_speed': LaunchConfiguration('min_speed'),
                'max_speed': LaunchConfiguration('max_speed'),
            }],
            output='screen',
        ),

        # Web Node
        Node(
            package='motherv2_web',
            executable='web_node',
            name='web_node',
            parameters=[{
                'port': LaunchConfiguration('web_port'),
            }],
            output='screen',
        ),
    ])
