"""
raft_flow 전체 스택 launch 파일

실행 순서:
  1. flir_camera_node  — FLIR 카메라 드라이버, /flir/image_raw 발행
  2. raft_flow_node    — /flir/image_raw 구독 후 RAFT 광학 흐름 추론·발행
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share   = FindPackageShare('raft_flow')
    default_cfg = PathJoinSubstitution([pkg_share, 'config', 'params.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_cfg,
            description='파라미터 YAML 파일 경로',
        ),

        # ── 카메라 드라이버 ──
        Node(
            package='raft_flow',
            executable='flir_camera_node',
            name='flir_camera_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),

        # ── RAFT 추론 노드 ──
        Node(
            package='raft_flow',
            executable='raft_flow_node',
            name='raft_flow_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
