from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """XYZ 스테이지 + 리미트 센서 테스트"""
    
    # RMD 스테이지 모터 (0x143-0x147)
    rmd_node = Node(
        package='rmd_robot_control',
        executable='position_control_node',
        name='rmd_stage',
        output='screen',
        parameters=[{
            'can_interface': 'can0',
            'motor_ids': [0x143, 0x144, 0x145, 0x146, 0x147],
        }]
    )
    
    # EZI-IO 리미트 센서
    ezi_io_node = Node(
        package='ezi_io_ros2',
        executable='ezi_io_node',
        name='ezi_io',
        output='screen',
    )
    
    # Joy 노드
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        output='screen',
    )
    
    # 텔레옵
    teleop_node = Node(
        package='rebar_control',
        executable='teleop_node',
        name='teleop',
        output='screen',
    )
    
    # 안전 모니터
    safety_node = Node(
        package='rebar_control',
        executable='safety_monitor',
        name='safety_monitor',
        output='screen',
    )
    
    return LaunchDescription([
        rmd_node,
        ezi_io_node,
        joy_node,
        teleop_node,
        safety_node,
    ])
