"""
slam.launch.py
──────────────────────────────────────────────────────────────────────────────
slam_toolbox + 객체 위치 추정 노드 런치 파일

시작 노드:
  1) async_slam_toolbox_node — /scan → map TF + /pose 퍼블리시
  2) slam_localization_node  — 라이다 깊이 융합 + 궤적 예측 + 장소 인덱싱

사용법:
  # SLAM만 단독 실행 (다른 노드와 결합)
  ros2 launch motherv2_bringup slam.launch.py

  # 파라미터 오버라이드 예시
  ros2 launch motherv2_bringup slam.launch.py \
      lidar_frame:=laser \
      camera_hfov_deg:=62.2 \
      target_depth_m:=0.8

필수 조건:
  - /scan 토픽이 퍼블리시되어야 함 (라이다 드라이버 별도 실행)
  - slam_toolbox 설치 필요:
      sudo apt install ros-jazzy-slam-toolbox
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_share = get_package_share_directory('motherv2_bringup')
    slam_params_file = os.path.join(bringup_share, 'config', 'slam_params.yaml')

    return LaunchDescription([
        # ── 런치 인자 ──────────────────────────────────────────────────────
        # LiDAR 설정
        DeclareLaunchArgument(
            'lidar_frame', default_value='base_scan',
            description='LiDAR TF 프레임 ID (/scan header.frame_id)'),
        DeclareLaunchArgument(
            'base_frame', default_value='base_link',
            description='로봇 기준 프레임'),
        DeclareLaunchArgument(
            'odom_frame', default_value='base_link',
            description='Odometry 프레임 — odom 없이 쓸 경우 base_link와 동일하게 설정'),
        DeclareLaunchArgument(
            'map_frame', default_value='map',
            description='맵 프레임'),

        # slam_toolbox 맵 설정
        DeclareLaunchArgument(
            'map_resolution', default_value='0.05',
            description='맵 해상도 (미터/셀)'),

        # 카메라 / 라이다 융합 파라미터
        DeclareLaunchArgument(
            'camera_hfov_deg', default_value='62.2',
            description='카메라 수평 화각 (도). IMX219=62.2°'),
        DeclareLaunchArgument(
            'image_width', default_value='640',
            description='카메라 이미지 가로 해상도'),
        DeclareLaunchArgument(
            'lidar_angle_offset_rad', default_value='0.0',
            description='카메라-라이다 수평 각도 오프셋 (라디안)'),
        DeclareLaunchArgument(
            'target_depth_m', default_value='0.8',
            description='라이다 기반 목표 추적 거리 (미터)'),
        DeclareLaunchArgument(
            'target_class_id', default_value='0',
            description='추적 대상 COCO class_id (0=person)'),
        DeclareLaunchArgument(
            'lost_timeout_s', default_value='1.5',
            description='객체 로스트 판정 시간 (초)'),
        DeclareLaunchArgument(
            'prediction_horizon_s', default_value='2.0',
            description='Dead-reckoning 예측 시간 (초)'),
        DeclareLaunchArgument(
            'location_index_path',
            default_value='/root/.ros/motherv2/location_index.json',
            description='장소 인덱스 JSON 저장 경로'),

        # ── odom 없이 동작하기 위한 identity TF ──────────────────────────────
        # slam_toolbox가 odom→base_link TF를 요구하므로 static identity 발행
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            output='screen',
        ),

        # ── slam_toolbox 노드 ─────────────────────────────────────────────
        # /scan → map TF, /pose, /map 퍼블리시 (오도메트리 없이 동작)
        # 파라미터는 config/slam_params.yaml 로 관리 (inline dict 방식보다 안정적)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            emulate_tty=True,
            parameters=[slam_params_file],
        ),

        # ── slam_toolbox lifecycle 확인 ───────────────────────────────────
        # autostart:true 파라미터로 자동 activate 됨 (slam_params.yaml 참고)
        # 이 루프는 활성화 여부만 확인하고, 실패 시 수동 전환을 시도
        TimerAction(period=5.0, actions=[
            ExecuteProcess(
                cmd=[
                    'bash', '-c',
                    'for i in 1 2 3 4 5 6 7 8 9 10; do '
                    '  state=$(ros2 lifecycle get /slam_toolbox 2>/dev/null); '
                    '  if echo "$state" | grep -qi active; then '
                    '    echo "[lifecycle] SLAM active OK (autostart)"; exit 0; '
                    '  fi; '
                    '  if ros2 lifecycle set /slam_toolbox configure 2>/dev/null; then '
                    '    sleep 1; '
                    '    ros2 lifecycle set /slam_toolbox activate 2>/dev/null && '
                    '      echo "[lifecycle] SLAM active OK (manual)" && exit 0; '
                    '  fi; '
                    '  echo "[lifecycle] /slam_toolbox 대기 중... ($i/10)"; '
                    '  sleep 3; '
                    'done; '
                    'echo "[lifecycle] SLAM 시작 실패 - slam_toolbox 확인 필요"'
                ],
                output='screen',
            ),
        ]),

        # ── explore_node ───────────────────────────────────────────────────
        # Frontier 기반 자율 맵핑 (slam_mode="explore" 수신 시 활성화)
        Node(
            package='motherv2_slam',
            executable='explore_node',
            name='explore_node',
            output='screen',
        ),

        # ── slam_localization_node ─────────────────────────────────────────
        # LiDAR 깊이 융합 + 맵 좌표 변환 + 궤적 예측 + 장소 인덱싱
        Node(
            package='motherv2_slam',
            executable='slam_localization_node',
            name='slam_localization_node',
            output='screen',
            parameters=[{
                'camera_hfov_deg': LaunchConfiguration('camera_hfov_deg'),
                'image_width': LaunchConfiguration('image_width'),
                'lidar_angle_offset_rad':
                    LaunchConfiguration('lidar_angle_offset_rad'),
                'target_class_id': LaunchConfiguration('target_class_id'),
                'lost_timeout_s': LaunchConfiguration('lost_timeout_s'),
                'prediction_horizon_s':
                    LaunchConfiguration('prediction_horizon_s'),
                'map_frame': LaunchConfiguration('map_frame'),
                'base_frame': LaunchConfiguration('base_frame'),
                'location_index_path':
                    LaunchConfiguration('location_index_path'),
                'depth_percentile': 15,
                'min_depth_m': 0.15,
                'max_depth_m': 8.0,
                'ema_alpha': 0.4,
                'sector_padding_rad': 0.05,
                'index_save_interval_s': 10.0,
                'index_cluster_radius_m': 0.5,
            }],
        ),
    ])
