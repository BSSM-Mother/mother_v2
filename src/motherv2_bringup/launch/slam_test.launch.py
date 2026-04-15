"""
slam_test.launch.py
──────────────────────────────────────────────────────────────────────────────
SLAM 단독 테스트용 런치 파일

실행 노드:
  1) sllidar_node         — echo_lidar 라이다 드라이버 (/scan 퍼블리시)
  2) camera_node          — 카메라 영상 (/motherv2/image_raw)
  3) detection_node       — 객체 감지 (/motherv2/detections)
  4) slam_toolbox         — SLAM 맵 생성 (/map, map TF, odom 불필요)
  5) slam_localization_node — 라이다 깊이 융합 + 궤적 예측
  6) web_node             — 웹 대시보드 (SLAM 맵 실시간 시각화)

사용법:
  ros2 launch motherv2_bringup slam_test.launch.py
  ros2 launch motherv2_bringup slam_test.launch.py lidar_port:=/dev/ttyAMA2
  ros2 launch motherv2_bringup slam_test.launch.py web_port:=8080 use_slam_depth:=false
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    detection_share = get_package_share_directory('motherv2_detection')
    default_model = os.path.join(detection_share, 'models', 'efficientdet_lite0.tflite')

    bringup_share = get_package_share_directory('motherv2_bringup')
    slam_launch_path = os.path.join(bringup_share, 'launch', 'slam.launch.py')

    return LaunchDescription([
        # ── 인자 ────────────────────────────────────────────────────────────
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyAMA2',
                              description='라이다 시리얼 포트'),
        DeclareLaunchArgument('camera_device', default_value='0'),
        DeclareLaunchArgument('camera_width', default_value='640'),
        DeclareLaunchArgument('camera_height', default_value='360'),
        DeclareLaunchArgument('camera_orientation', default_value='0'),
        DeclareLaunchArgument('model_path', default_value=default_model),
        DeclareLaunchArgument('conf_threshold', default_value='0.35'),
        DeclareLaunchArgument('debug_class', default_value='-1'),
        DeclareLaunchArgument('web_port', default_value='8080'),
        DeclareLaunchArgument('camera_hfov_deg', default_value='62.2'),
        DeclareLaunchArgument('target_depth_m', default_value='0.8'),
        DeclareLaunchArgument('lidar_angle_offset_rad', default_value='0.0'),
        DeclareLaunchArgument('target_class_id', default_value='0'),  # 0=person

        # ── 1) 라이다 드라이버 ───────────────────────────────────────────────
        Node(
            package='echo_lidar',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': LaunchConfiguration('lidar_port'),
                'serial_baudrate': 460800,
                'frame_id': 'base_link',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Standard',
                'scan_frequency': 10.0,
            }],
            output='screen',
        ),

        # ── 2) 카메라 ───────────────────────────────────────────────────────
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

        # ── 3) 객체 감지 ─────────────────────────────────────────────────────
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

        # ── 4+5) hector_mapping + slam_localization ──────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_path),
            launch_arguments={
                'camera_hfov_deg': LaunchConfiguration('camera_hfov_deg'),
                'image_width': LaunchConfiguration('camera_width'),
                'lidar_angle_offset_rad': LaunchConfiguration('lidar_angle_offset_rad'),
                'target_class_id': LaunchConfiguration('target_class_id'),
            }.items(),
        ),

        # ── 6) 웹 대시보드 ───────────────────────────────────────────────────
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
