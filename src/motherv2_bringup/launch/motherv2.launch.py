import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    detection_share = get_package_share_directory('motherv2_detection')
    default_model = os.path.join(detection_share, 'models', 'efficientdet_lite0.tflite')

    bringup_share = get_package_share_directory('motherv2_bringup')
    slam_launch_path = os.path.join(bringup_share, 'launch', 'slam.launch.py')

    # API_URL 환경변수가 설정되어 있을 때만 api_node 실행
    # 미설정 시 follower는 _follow_enabled=True 기본값으로 항상 팔로우
    api_url = os.environ.get('API_URL', '').strip()

    args = [
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
        DeclareLaunchArgument('target_bbox_ratio', default_value='0.80'),
        DeclareLaunchArgument('web_port', default_value='8080'),
        DeclareLaunchArgument('debug_class', default_value='-1'),
        DeclareLaunchArgument('mqtt_broker', default_value='broker.emqx.io'),
        DeclareLaunchArgument('mqtt_topic', default_value='bssm/relay'),
        DeclareLaunchArgument('api_poll_interval', default_value='1.0'),
        # ── SLAM 관련 인자 ─────────────────────────────────────────────────
        # use_slam:=true 이면 hector_mapping + slam_localization_node 함께 실행
        DeclareLaunchArgument(
            'use_slam', default_value='true',
            description='true: SLAM 활성화 (라이다 드라이버 별도 실행 필요)'),
        DeclareLaunchArgument(
            'camera_hfov_deg', default_value='62.2',
            description='카메라 수평 화각 (도)'),
        DeclareLaunchArgument(
            'target_depth_m', default_value='0.8',
            description='SLAM 기반 목표 추적 거리 (미터)'),
        DeclareLaunchArgument(
            'lidar_angle_offset_rad', default_value='0.0',
            description='카메라-라이다 수평 각도 오프셋 (라디안)'),
        DeclareLaunchArgument(
            'target_class_id', default_value='39',
            description='추적 대상 COCO class_id'),
    ]

    nodes = [
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
                # SLAM 연동: use_slam=true 이면 라이다 깊이 기반 거리 제어 활성화
                'use_slam_depth': LaunchConfiguration('use_slam'),
                'use_slam_search': LaunchConfiguration('use_slam'),
                'target_depth_m': LaunchConfiguration('target_depth_m'),
                'search_enabled': True,
                'follow_default': not api_url,  # API 있으면 False로 시작, 없으면 True
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

        # MQTT Node (ESP32 릴레이 등 외부 하드웨어 제어)
        Node(
            package='motherv2_mqtt',
            executable='mqtt_node',
            name='mqtt_node',
            parameters=[{
                'mqtt_broker': LaunchConfiguration('mqtt_broker'),
                'mqtt_topic': LaunchConfiguration('mqtt_topic'),
            }],
            output='screen',
        ),

        # ── SLAM (use_slam:=true 일 때만 실행) ────────────────────────────
        # hector_mapping + slam_localization_node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_path),
            launch_arguments={
                'camera_hfov_deg': LaunchConfiguration('camera_hfov_deg'),
                'image_width': LaunchConfiguration('camera_width'),
                'lidar_angle_offset_rad':
                    LaunchConfiguration('lidar_angle_offset_rad'),
                'target_class_id': LaunchConfiguration('target_class_id'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('use_slam')),
        ),
    ]

    # API_URL이 설정된 경우에만 api_node 추가
    if api_url:
        nodes.append(
            Node(
                package='motherv2_web',
                executable='api_node',
                name='api_node',
                parameters=[{
                    'poll_interval': LaunchConfiguration('api_poll_interval'),
                }],
                output='screen',
            )
        )

    return LaunchDescription(args + nodes)
