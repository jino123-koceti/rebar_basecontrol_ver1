from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """텔레옵 + 주행 모터만 테스트"""
    
    # RMD 주행 모터 (0x141, 0x142)
    rmd_node = Node(
        package='rmd_robot_control',
        executable='cmd_vel_control_node',
        name='rmd_driving',
        output='screen',
        parameters=[{
            'can_interface': 'can0',
            'motor_ids': [0x141, 0x142],
        }]
    )
    
    # Joy 노드
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        output='screen',
    )
    
    # 텔레옵 (주행만)
    teleop_node = Node(
        package='rebar_control',
        executable='teleop_node',
        name='teleop',
        output='screen',
    )
    
    return LaunchDescription([
        rmd_node,
        joy_node,
        teleop_node,
    ])
