#!/usr/bin/env python3
"""
GUI와 하위제어 시스템 통합 런치 파일
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import launch
import launch.conditions
import launch.substitutions


def generate_launch_description():
    """런치 파일 생성"""
    
    # 패키지 경로 가져오기
    pkg_name = 'rmd_robot_control'
    config_dir = os.path.join(get_package_share_directory(pkg_name), 'config')
    robot_control_config = os.path.join(config_dir, 'robot_control.yaml')
    
    # 런치 인수 선언
    gui_only_arg = DeclareLaunchArgument(
        'gui_only',
        default_value='false',
        description='GUI만 실행 (하위제어 노드 없이)'
    )
    
    # 통합 제어 노드 (위치제어 + CMD_VEL 속도제어)
    # 0x141, 0x142: cmd_vel 속도 제어
    # 0x143, 0x144, 0x145, 0x146, 0x147: 위치 제어
    # 하나의 CAN Manager로 모든 모터 제어 -> Bus-Off 방지!
    unified_control_node = Node(
        package=pkg_name,
        executable='position_control_node',
        name='unified_control_node',
        output='screen',
        parameters=[robot_control_config],
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui_only'))
    )
    
    # GUI 노드
    gui_node = Node(
        package=pkg_name,
        executable='robot_control_gui',
        name='robot_control_gui',
        output='screen'
    )
    
    # 시작 메시지
    start_message = LogInfo(
        msg="RMD 로봇 제어 시스템 (GUI 포함) 시작"
    )

    # GUI 종료 시 전체 launch 종료 이벤트 핸들러
    gui_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=gui_node,
            on_exit=[
                LogInfo(msg="GUI 종료 감지 - 모든 노드 종료 중..."),
                EmitEvent(event=Shutdown())
            ]
        )
    )

    return LaunchDescription([
        gui_only_arg,
        start_message,
        unified_control_node,
        gui_node,
        gui_exit_handler
    ])
