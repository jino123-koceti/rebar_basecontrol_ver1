from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """그리퍼 + 트리거 테스트"""
    
    # Pololu 트리거
    pololu_node = Node(
        package='pololu_ros2',
        executable='pololu_node',
        name='pololu_trigger',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyACM0',
        }]
    )
    
    # Seengrip 그리퍼
    seengrip_node = Node(
        package='seengrip_ros2',
        executable='seengrip_node',
        name='seengrip',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
        }]
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
    
    return LaunchDescription([
        pololu_node,
        seengrip_node,
        joy_node,
        teleop_node,
    ])
