#!/usr/bin/env python3
"""
Iron-MD 텔레옵 테스트용 런치 파일
노트북 can0에서 조종기 신호만 받아서 동작 확인
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # 패키지 경로
    pkg_share = FindPackageShare('rebar_control')
    
    # 파라미터 파일 경로
    params_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'test_params.yaml'
    ])
    
    return LaunchDescription([
        
        # 텔레옵 노드 (디버그 모드)
        Node(
            package='rebar_control',
            executable='iron_md_teleop',
            name='iron_md_teleop',
            output='screen',
            parameters=[params_file],
            emulate_tty=True,
        ),
        
    ])
