from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch pololu motor controller node"""
    
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for Pololu controller'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='9600',
        description='Serial baudrate'
    )
    
    # Pololu node
    pololu_node = Node(
        package='pololu_ros2',
        executable='pololu_node',
        name='pololu_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'motor_ids': [0],
            'motor_topics': ['motor_0/vel']
        }]
    )
    
    return LaunchDescription([
        serial_port_arg,
        baudrate_arg,
        pololu_node
    ])
