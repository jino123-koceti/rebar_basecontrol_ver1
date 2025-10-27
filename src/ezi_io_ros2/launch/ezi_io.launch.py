from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Launch arguments
    ip_arg = DeclareLaunchArgument(
        'ip_address',
        default_value='192.168.0.2',
        description='IP address of EZI-IO module'
    )
    
    # EZI-IO 노드
    ezi_io_node = Node(
        package='ezi_io_ros2',
        executable='ezi_io_node',
        name='ezi_io',
        output='screen',
        parameters=[{
            'ip_address': LaunchConfiguration('ip_address'),
            'port': 502,
            'slave_id': 1,
            'update_rate': 20.0,
            'input_start_address': 0,
            'input_count': 16,
            # XYZ 스테이지 리미트 센서 채널 매핑
            'limit_x_min_channel': 0,
            'limit_x_max_channel': 1,
            'limit_y_min_channel': 2,
            'limit_y_max_channel': 3,
            'limit_z_min_channel': 4,
            'limit_z_max_channel': 5,
        }]
    )
    
    return LaunchDescription([
        ip_arg,
        ezi_io_node,
    ])
