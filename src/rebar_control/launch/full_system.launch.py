from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """
    철근 결속 로봇 통합 런치 파일
    모든 하드웨어 노드를 한 번에 실행
    """
    
    # Launch arguments
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulation mode (no hardware)'
    )
    
    can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='CAN interface name'
    )
    
    # =============================================================
    # 1. RMD 모터 제어 (0x141-0x147)
    # =============================================================
    rmd_control_node = Node(
        package='rmd_robot_control',
        executable='robot_control_node',
        name='rmd_control',
        output='screen',
        parameters=[{
            'can_interface': LaunchConfiguration('can_interface'),
            'cmd_vel_motor_ids': [0x141, 0x142],
            'position_motor_ids': [0x143, 0x144, 0x145, 0x146, 0x147],
            'joint_names': ['left_wheel', 'right_wheel', 'lifting', 'x_axis', 'y_axis', 'z_axis', 'yaw'],
        }]
    )
    
    # =============================================================
    # 2. Pololu 액추에이터 (트리거)
    # =============================================================
    pololu_node = Node(
        package='pololu_ros2',
        executable='pololu_node',
        name='pololu_trigger',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyACM0',
            'baudrate': 9600,
            'motor_ids': [0],
            'motor_topics': ['motor_0/vel'],
        }]
    )
    
    # =============================================================
    # 3. Seengrip 그리퍼
    # =============================================================
    seengrip_node = Node(
        package='seengrip_ros2',
        executable='seengrip_node',
        name='seengrip',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'baudrate': 115200,
            'slave_id': 1,
            'default_speed': 500,
            'open_position': 0,
            'close_position': 2000,
        }]
    )
    
    # =============================================================
    # 4. EZI-IO 리미트 센서
    # =============================================================
    ezi_io_node = Node(
        package='ezi_io_ros2',
        executable='ezi_io_node',
        name='ezi_io',
        output='screen',
        parameters=[{
            'ip_address': '192.168.0.2',
            'port': 502,
            'slave_id': 1,
            'update_rate': 20.0,
            'input_start_address': 0,
            'input_count': 16,
            'limit_x_min_channel': 0,
            'limit_x_max_channel': 1,
            'limit_y_min_channel': 2,
            'limit_y_max_channel': 3,
            'limit_z_min_channel': 4,
            'limit_z_max_channel': 5,
        }]
    )
    
    # =============================================================
    # 5. Iron-MD 텔레옵 노드 (CAN 조종기)
    # =============================================================
    iron_md_teleop_node = Node(
        package='rebar_control',
        executable='iron_md_teleop_node',
        name='iron_md_teleop',
        output='screen',
        parameters=[{
            'can_interface': 'can1',
            'can_baudrate': 250000,
            'max_linear_speed': 0.5,
            'max_angular_speed': 1.0,
            'xyz_step_size': 0.01,
            'yaw_step_size': 0.1,
            'lifting_step_size': 0.05,
            'trigger_duration': 0.5,
            'joystick_center': 127,
            'joystick_deadzone': 20,
        }]
    )
    
    # =============================================================
    # 7. 안전 모니터
    # =============================================================
    safety_node = Node(
        package='rebar_control',
        executable='safety_monitor',
        name='safety_monitor',
        output='screen',
    )
    
    return LaunchDescription([
        use_sim_arg,
        can_interface_arg,
        
        # 하드웨어 노드들
        rmd_control_node,
        pololu_node,
        seengrip_node,
        ezi_io_node,
        
        # Iron-MD 조종기 및 제어
        iron_md_teleop_node,
        safety_node,
    ])
