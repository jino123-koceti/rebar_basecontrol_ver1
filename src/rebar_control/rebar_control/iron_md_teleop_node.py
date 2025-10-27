#!/usr/bin/env python3
"""
철근 결속 로봇 텔레옵 노드 (Iron-MD CAN 조종기)
Iron-MD 무선 조종기 CAN 메시지를 수신하여 로봇 제어

CAN 통신:
- 0x1E4 (484): 조이스틱 아날로그 데이터 (50ms)
- 0x2E4 (740): 스위치 및 상태 (50ms)
- 0x764 (1892): Heartbeat (300ms)

조종기 매핑:
[아날로그 조이스틱 - 연속 제어]
- Joystick_3 (AN3): 하부체 전후진 (0x141, 0x142)
- Joystick_1 (AN1): 상부체 X축 (0x144)
- Joystick_2 (AN2): 상부체 Y축 (0x145)

[3단 스위치 - 토글형]
- S19-S20: 모드 선택 (S19=Remote, S20=Automatic)
- S17-S18: 횡이동 (S17=+50mm, S18=-50mm) 0x143
- S21-S22: 작업 시퀀스 (S21=하강+그립, S22=결속+상승)
- S23-S24: Yaw 회전 (S23=+30°, S24=-30°) 0x147

[일반 스위치]
- S00: 트리거 당김
- S01: 트리거 해제
- Emergency_Stop: 비상 정지
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float64MultiArray, Int32, Bool
import can
import struct
import threading
import math


class IronMDTeleopNode(Node):
    """Iron-MD CAN 조종기로 로봇 단동 제어"""
    
    def __init__(self):
        super().__init__('iron_md_teleop')
        
        # 파라미터 선언
        self.declare_parameter('can_interface', 'can0')  # 테스트용 can0
        self.declare_parameter('can_baudrate', 250000)
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('xyz_step_size', 0.01)  # m, 10mm per command
        self.declare_parameter('lateral_move_distance', 0.05)  # m, 50mm per step (횡이동)
        self.declare_parameter('z_work_distance', 0.1)  # m, 100mm for work sequence
        self.declare_parameter('yaw_rotation_angle', 30.0)  # degrees, 30도
        self.declare_parameter('trigger_duration', 0.5)  # seconds
        self.declare_parameter('gripper_open_position', 0)  # 그리퍼 열림
        self.declare_parameter('gripper_close_position', 2000)  # 그리퍼 닫힘
        self.declare_parameter('joystick_center', 127)  # 중립값
        self.declare_parameter('joystick_deadzone', 20)  # 데드존
        self.declare_parameter('debug_mode', True)  # 디버그 모드 (터미널 출력 상세화)
        
        self.can_interface = self.get_parameter('can_interface').value
        self.can_baudrate = self.get_parameter('can_baudrate').value
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.xyz_step = self.get_parameter('xyz_step_size').value
        self.lateral_distance = self.get_parameter('lateral_move_distance').value
        self.z_work_distance = self.get_parameter('z_work_distance').value
        self.yaw_angle = self.get_parameter('yaw_rotation_angle').value
        self.trigger_duration = self.get_parameter('trigger_duration').value
        self.gripper_open = self.get_parameter('gripper_open_position').value
        self.gripper_close = self.get_parameter('gripper_close_position').value
        self.joy_center = self.get_parameter('joystick_center').value
        self.joy_deadzone = self.get_parameter('joystick_deadzone').value
        self.debug_mode = self.get_parameter('debug_mode').value
        
        # CAN 버스 초기화
        try:
            self.can_bus = can.interface.Bus(
                channel=self.can_interface,
                bustype='socketcan',
                bitrate=self.can_baudrate
            )
            self.get_logger().info(f'CAN bus opened: {self.can_interface} @ {self.can_baudrate} bps')
        except Exception as e:
            self.get_logger().error(f'Failed to open CAN bus: {e}')
            raise
        
        # CAN 수신 스레드
        self.can_thread = threading.Thread(target=self.can_receiver_thread, daemon=True)
        self.can_thread.start()
        
        # ROS2 발행자들
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint1_pub = self.create_publisher(Float64MultiArray, '/joint_1/position', 10)  # Lifting
        self.joint2_pub = self.create_publisher(Float64MultiArray, '/joint_2/position', 10)  # X-axis
        self.joint3_pub = self.create_publisher(Float64MultiArray, '/joint_3/position', 10)  # Y-axis
        self.joint4_pub = self.create_publisher(Float64MultiArray, '/joint_4/position', 10)  # Z-axis
        self.joint5_pub = self.create_publisher(Float64MultiArray, '/joint_5/position', 10)  # Yaw
        self.trigger_pub = self.create_publisher(Float32, '/motor_0/vel', 10)
        self.gripper_pos_pub = self.create_publisher(Float32, '/gripper/position', 10)
        self.gripper_cmd_pub = self.create_publisher(Int32, '/gripper/command', 10)
        self.estop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        
        # 조이스틱 및 스위치 상태
        self.joystick_data = {
            'AN1': 127,  # X축 (상부체)
            'AN2': 127,  # Y축 (상부체)
            'AN3': 127,  # 주행 전후진 (하부체)
            'AN4': 127,  # (예비)
        }
        
        self.switch_data = {
            'S00': 0, 'S01': 0, 'S02': 0, 'S03': 0,
            'S06': 0, 'S07': 0, 'S08': 0, 'S09': 0,
            'S17': 0, 'S18': 0, 'S19': 0, 'S20': 0,
            'S21': 0, 'S22': 0, 'S23': 0, 'S24': 0,
            'Emergency_Stop_Active': 0,
            'Emergency_Stop_Release': 1,
            'TX_Connected': 0,
        }
        
        # 이전 스위치 상태 (엣지 감지)
        self.prev_switches = self.switch_data.copy()
        
        # 현재 위치
        self.current_positions = {
            'lateral': 0.0,  # 횡이동 (0x143)
            'x': 0.0,  # X축 (0x144)
            'y': 0.0,  # Y축 (0x145)
            'z': 0.0,  # Z축 (0x146)
            'yaw': 0.0,  # Yaw (0x147)
        }
        
        # 제어 모드
        self.control_mode = 'remote'  # 'remote' or 'automatic'
        
        # 작업 시퀀스 상태
        self.work_sequence_active = False
        
        # 트리거 타이머
        self.trigger_timer = None
        
        # 안전 플래그
        self.emergency_stopped = False
        
        # 마지막 발행 값 (중복 방지)
        self.last_cmd_sent = {'linear': 0.0, 'angular': 0.0}
        
        # 제어 루프 타이머 (20Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        # 디버그 출력용 타이머 (1Hz)
        if self.debug_mode:
            self.debug_timer = self.create_timer(1.0, self.print_status)
        
        self.get_logger().info(f'Iron-MD Teleop node started (CAN: {self.can_interface})')
        if self.debug_mode:
            self.get_logger().info('🔍 DEBUG MODE: 터미널 출력 상세화')
        self.print_help()
    
    def print_help(self):
        """조종기 매핑 도움말"""
        help_text = """
╔════════════════════════════════════════════════════════════════════╗
║         철근 결속 로봇 Iron-MD 조종기 매핑 (CAN)                  ║
╠════════════════════════════════════════════════════════════════════╣
║ [연속 제어 - 아날로그 조이스틱]                                   ║
║   AN3 (조이스틱 3) : 하부체 전후진 (0x141, 0x142)                 ║
║   AN1 (조이스틱 1) : 상부체 X축 전후 (0x144)                      ║
║   AN2 (조이스틱 2) : 상부체 Y축 전후 (0x145)                      ║
║                                                                    ║
║ [모드 선택 - 3단 토글 스위치]                                     ║
║   S19 (상단)  : Remote Control 모드 (조종기 제어)                 ║
║   중립        : 대기                                               ║
║   S20 (하단)  : Automatic Control 모드 (상위제어 연동)            ║
║                                                                    ║
║ [횡이동 - 3단 토글 스위치 (엣지 트리거)]                          ║
║   S17 ON→OFF  : 하부체 횡이동 +50mm (0x143, 360도 회전)           ║
║   S18 ON→OFF  : 하부체 횡이동 -50mm (0x143, -360도 회전)          ║
║                                                                    ║
║ [작업 시퀀스 - 3단 토글 스위치 (엣지 트리거)]                     ║
║   S21 ON→OFF  : Z축 100mm 하강 → 그리퍼 닫기(0)                   ║
║   S22 ON→OFF  : 트리거 동작 → 그리퍼 열기(2000) → Z축 100mm 상승  ║
║                                                                    ║
║ [Yaw 회전 - 3단 토글 스위치 (엣지 트리거)]                        ║
║   S23 ON→OFF  : 공구 Yaw +30도 회전 (0x147)                       ║
║   S24 ON→OFF  : 공구 Yaw -30도 회전 (0x147)                       ║
║                                                                    ║
║ [기타 제어]                                                        ║
║   S00         : 트리거 당김 (수동)                                 ║
║   S01         : 트리거 해제 (수동)                                 ║
║   E-Stop      : 비상 정지                                          ║
╚════════════════════════════════════════════════════════════════════╝
"""
        self.get_logger().info(help_text) 
    
    def can_receiver_thread(self):
        """CAN 메시지 수신 스레드"""
        while rclpy.ok():
            try:
                msg = self.can_bus.recv(timeout=1.0)
                if msg is not None:
                    self.process_can_message(msg)
            except Exception as e:
                self.get_logger().error(f'CAN receive error: {e}')
    
    def process_can_message(self, msg):
        """CAN 메시지 파싱"""
        can_id = msg.arbitration_id
        data = msg.data
        
        if can_id == 0x1E4:  # 484: Joystick Data
            self.parse_joystick_data(data)
        
        elif can_id == 0x2E4:  # 740: Switch Status
            self.parse_switch_status(data)
        
        elif can_id == 0x764:  # 1892: Heartbeat
            pass  # Heartbeat는 연결 상태 확인용
    
    def parse_joystick_data(self, data):
        """조이스틱 데이터 파싱 (0x1E4)"""
        if len(data) >= 4:
            self.joystick_data['AN1'] = data[0]  # Joystick 1
            self.joystick_data['AN2'] = data[1]  # Joystick 2
            self.joystick_data['AN3'] = data[2]  # Joystick 3
            self.joystick_data['AN4'] = data[3]  # Joystick 4
            
            if self.debug_mode:
                self.get_logger().debug(
                    f'📊 조이스틱: AN1={data[0]:3d} AN2={data[1]:3d} '
                    f'AN3={data[2]:3d} AN4={data[3]:3d}'
                )
    
    def parse_switch_status(self, data):
        """스위치 상태 파싱 (0x2E4)"""
        if len(data) < 8:
            return
        
        # Byte 0: Start, Power, Engine, Emergency
        byte0 = data[0]
        self.switch_data['Emergency_Stop_Release'] = (byte0 >> 6) & 0x01
        self.switch_data['Emergency_Stop_Active'] = (byte0 >> 7) & 0x01
        
        # Byte 1: S00-S07
        byte1 = data[1]
        self.switch_data['S06'] = (byte1 >> 0) & 0x01
        self.switch_data['S07'] = (byte1 >> 1) & 0x01
        self.switch_data['S02'] = (byte1 >> 4) & 0x01
        self.switch_data['S03'] = (byte1 >> 5) & 0x01
        self.switch_data['S01'] = (byte1 >> 6) & 0x01
        self.switch_data['S00'] = (byte1 >> 7) & 0x01
        
        # Byte 2: S08-S09
        byte2 = data[2]
        self.switch_data['S08'] = (byte2 >> 0) & 0x01
        self.switch_data['S09'] = (byte2 >> 1) & 0x01
        
        # Byte 3: S17-S24
        byte3 = data[3]
        self.switch_data['S23'] = (byte3 >> 0) & 0x01
        self.switch_data['S24'] = (byte3 >> 1) & 0x01
        self.switch_data['S21'] = (byte3 >> 2) & 0x01
        self.switch_data['S22'] = (byte3 >> 3) & 0x01
        self.switch_data['S19'] = (byte3 >> 4) & 0x01
        self.switch_data['S20'] = (byte3 >> 5) & 0x01
        self.switch_data['S17'] = (byte3 >> 6) & 0x01
        self.switch_data['S18'] = (byte3 >> 7) & 0x01
        
        # Byte 6: TX Connected
        if len(data) >= 7:
            byte6 = data[6]
            self.switch_data['TX_Connected'] = (byte6 >> 6) & 0x01
    
    def switch_pressed(self, switch_name):
        """스위치 엣지 감지 (Rising Edge)"""
        current = self.switch_data.get(switch_name, 0)
        previous = self.prev_switches.get(switch_name, 0)
        return current == 1 and previous == 0
    
    def normalize_joystick(self, value):
        """조이스틱 값 정규화 (0-255 -> -1.0 to 1.0)"""
        centered = value - self.joy_center
        
        # 데드존 적용
        if abs(centered) < self.joy_deadzone:
            return 0.0
        
        # 정규화
        if centered > 0:
            return centered / (255 - self.joy_center)
        else:
            return centered / self.joy_center
    
    def control_loop(self):
        """제어 루프 (20Hz)"""
        # 비상 정지 체크
        if self.switch_data['Emergency_Stop_Active'] == 1:
            if not self.emergency_stopped:
                self.emergency_stop()
            return
        elif self.emergency_stopped and self.switch_data['Emergency_Stop_Release'] == 1:
            self.emergency_stopped = False
            self.get_logger().info('✓ 비상 정지 해제 (하드웨어)')
        
        if self.emergency_stopped:
            return
        
        # 연결 상태 확인
        if self.switch_data['TX_Connected'] == 0:
            # 송신기 연결 안됨 - 모든 모터 정지
            self.publish_zero_velocity()
            return
        
        # 제어 모드 확인
        self.update_control_mode()
        
        if self.control_mode != 'remote':
            # Automatic 모드일 경우 상위제어 패키지에 제어권 위임
            return
        
        # Remote Control 모드
        # 1. 주행 제어 (연속) - AN3
        self.handle_driving()
        
        # 2. XYZ 스테이지 (연속) - AN1, AN2
        self.handle_xyz_stage()
        
        # 3. 횡이동 (스텝) - S17, S18
        self.handle_lateral_move()
        
        # 4. 작업 시퀀스 - S21, S22
        self.handle_work_sequence()
        
        # 5. Yaw 회전 - S23, S24
        self.handle_yaw_rotation()
        
        # 6. 트리거 (수동) - S00, S01
        self.handle_trigger()
        
        # 이전 상태 저장
        self.prev_switches = self.switch_data.copy()
    
    def update_control_mode(self):
        """제어 모드 업데이트 (S19-S20)"""
        if self.switch_data['S19'] == 1:
            if self.control_mode != 'remote':
                self.control_mode = 'remote'
                self.get_logger().info('🎮 Remote Control 모드')
        elif self.switch_data['S20'] == 1:
            if self.control_mode != 'automatic':
                self.control_mode = 'automatic'
                self.get_logger().info('🤖 Automatic Control 모드 (상위제어 대기)')
    
    def handle_driving(self):
        """주행 제어 (AN3: 전후, AN2: 좌우)"""
        # AN3: 전진/후진 (0x141, 0x142)
        linear = -self.normalize_joystick(self.joystick_data['AN3'])
        # AN2: 좌우 회전 (차동 제어로 변환)
        angular = -self.normalize_joystick(self.joystick_data['AN2'])
        
        # 값 변화가 있을 때만 발행 (중복 방지)
        if (abs(linear - self.last_cmd_sent['linear']) > 0.01 or 
            abs(angular - self.last_cmd_sent['angular']) > 0.01):
            twist = Twist()
            twist.linear.x = linear * self.max_linear
            twist.angular.z = angular * self.max_angular
            
            if self.debug_mode and (abs(linear) > 0.1 or abs(angular) > 0.1):
                self.get_logger().info(
                    f'🚗 주행 → /cmd_vel: linear={twist.linear.x:.2f} m/s, '
                    f'angular={twist.angular.z:.2f} rad/s'
                )
            
            self.cmd_vel_pub.publish(twist)
            self.last_cmd_sent['linear'] = linear
            self.last_cmd_sent['angular'] = angular
    
    def handle_xyz_stage(self):
        """XYZ 스테이지 제어 (AN1: X축, AN2: Y축)"""
        # X축: AN1 (연속)
        x_value = -self.normalize_joystick(self.joystick_data['AN1'])
        if abs(x_value) > 0.1:
            self.current_positions['x'] += self.xyz_step * x_value
            self.publish_joint_position('x', self.joint2_pub, show_log=False)
            if self.debug_mode:
                self.get_logger().info(
                    f'📐 X축 → /joint_2/position: {self.current_positions["x"]:.4f} m '
                    f'(delta: {self.xyz_step * x_value:+.4f})'
                )
        
        # Y축: AN2 (연속)
        y_value = -self.normalize_joystick(self.joystick_data['AN2'])
        if abs(y_value) > 0.1:
            self.current_positions['y'] += self.xyz_step * y_value
            self.publish_joint_position('y', self.joint3_pub, show_log=False)
            if self.debug_mode:
                self.get_logger().info(
                    f'📐 Y축 → /joint_3/position: {self.current_positions["y"]:.4f} m '
                    f'(delta: {self.xyz_step * y_value:+.4f})'
                )
    
    def handle_lateral_move(self):
        """횡이동 제어 (S17: +50mm, S18: -50mm) 0x143"""
        if self.switch_pressed('S17'):
            # 양의 방향 360도 회전 (+50mm)
            self.current_positions['lateral'] += self.lateral_distance
            self.publish_joint_position('lateral', self.joint1_pub)
            self.get_logger().info(f'➡️  횡이동 +50mm (위치: {self.current_positions["lateral"]:.3f}m)')
        
        elif self.switch_pressed('S18'):
            # 음의 방향 360도 회전 (-50mm)
            self.current_positions['lateral'] -= self.lateral_distance
            self.publish_joint_position('lateral', self.joint1_pub)
            self.get_logger().info(f'⬅️  횡이동 -50mm (위치: {self.current_positions["lateral"]:.3f}m)')
    
    def handle_work_sequence(self):
        """작업 시퀀스 제어"""
        if self.switch_pressed('S21'):
            # Z축 하강 + 그리퍼 닫기
            self.get_logger().info('🔽 작업 시퀀스: Z축 하강 → 그리퍼 닫기')
            self.current_positions['z'] -= self.z_work_distance  # 100mm 하강
            self.publish_joint_position('z', self.joint4_pub)
            
            # 그리퍼 닫기
            gripper_pos = Float32()
            gripper_pos.data = float(self.gripper_open)
            self.gripper_pos_pub.publish(gripper_pos)
            
        elif self.switch_pressed('S22'):
            # 트리거 동작 → 그리퍼 열기 → Z축 상승
            self.get_logger().info('🔧 작업 시퀀스: 트리거 → 그리퍼 열기 → Z축 상승')
            
            # 1. 트리거 동작
            trigger_cmd = Float32()
            trigger_cmd.data = 1.0
            self.trigger_pub.publish(trigger_cmd)
            
            # 2. 그리퍼 열기
            gripper_pos = Float32()
            gripper_pos.data = float(self.gripper_close)
            self.gripper_pos_pub.publish(gripper_pos)
            
            # 3. Z축 상승
            self.current_positions['z'] += self.z_work_distance  # 100mm 상승
            self.publish_joint_position('z', self.joint4_pub)
    
    def handle_yaw_rotation(self):
        """Yaw 회전 제어 (S23: +30°, S24: -30°) 0x147"""
        if self.switch_pressed('S23'):
            # 양의 방향 30도 회전
            self.current_positions['yaw'] += math.radians(self.yaw_angle)
            self.publish_joint_position('yaw', self.joint5_pub)
            self.get_logger().info(f'↻  Yaw +30° (위치: {math.degrees(self.current_positions["yaw"]):.1f}°)')
        
        elif self.switch_pressed('S24'):
            # 음의 방향 30도 회전
            self.current_positions['yaw'] -= math.radians(self.yaw_angle)
            self.publish_joint_position('yaw', self.joint5_pub)
            self.get_logger().info(f'↺  Yaw -30° (위치: {math.degrees(self.current_positions["yaw"]):.1f}°)')
    
    def handle_trigger(self):
        """트리거 제어 (S00: 당김, S01: 해제)"""
        if self.switch_pressed('S00'):
            self.trigger_pull()
        
        elif self.switch_pressed('S01'):
            self.trigger_release()
    
    def trigger_pull(self):
        """트리거 당기기"""
        self.get_logger().info('🔫 트리거 당김!')
        
        trigger_msg = Float32()
        trigger_msg.data = 1.0
        self.trigger_pub.publish(trigger_msg)
        
        if self.trigger_timer:
            self.trigger_timer.cancel()
        
        self.trigger_timer = self.create_timer(
            self.trigger_duration,
            self.trigger_release
        )
    
    def trigger_release(self):
        """트리거 해제"""
        trigger_msg = Float32()
        trigger_msg.data = 0.0
        self.trigger_pub.publish(trigger_msg)
        
        if self.trigger_timer:
            self.trigger_timer.cancel()
            self.trigger_timer = None
        
        self.get_logger().info('🔫 트리거 해제')
    
    def publish_joint_position(self, joint_name, publisher, show_log=True):
        """관절 위치 명령 발행"""
        msg = Float64MultiArray()
        msg.data = [self.current_positions[joint_name]]
        publisher.publish(msg)
        
        if show_log:
            self.get_logger().info(
                f'📍 {joint_name.upper()}: {self.current_positions[joint_name]:.3f} m'
            )
    
    def publish_zero_velocity(self):
        """모든 모터 정지"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
    
    def emergency_stop(self):
        """비상 정지"""
        self.emergency_stopped = True
        self.get_logger().error('🚨 비상 정지 활성화!')
        
        # 모든 모터 정지
        self.publish_zero_velocity()
        self.trigger_release()
        
        # 비상 정지 신호 발행
        estop_msg = Bool()
        estop_msg.data = True
        self.estop_pub.publish(estop_msg)
    
    def print_status(self):
        """디버그 모드 상태 출력 (1Hz)"""
        if not self.debug_mode:
            return
        
        # 조이스틱 정규화 값
        an1_norm = self.normalize_joystick(self.joystick_data['AN1'])
        an2_norm = self.normalize_joystick(self.joystick_data['AN2'])
        an3_norm = self.normalize_joystick(self.joystick_data['AN3'])
        an4_norm = self.normalize_joystick(self.joystick_data['AN4'])
        
        status = f"""
┌─────────────────────────────────────────────────────────────────────┐
│                  🎮 Iron-MD 조종기 상태 (DEBUG)                    │
├─────────────────────────────────────────────────────────────────────┤
│ 연결 상태: {'✅ 송신기 연결됨' if self.switch_data['TX_Connected'] else '❌ 송신기 끊김'}
│ 제어 모드: {'🎮 Remote Control' if self.control_mode == 'remote' else '🤖 Automatic Control'}
│ 비상정지: {'🚨 활성화' if self.emergency_stopped else '✅ 정상'}
│
│ [조이스틱 값 - Raw / 정규화]
│   AN1 (X축):   {self.joystick_data['AN1']:3d} → {an1_norm:+.2f}
│   AN2 (Y축):   {self.joystick_data['AN2']:3d} → {an2_norm:+.2f}
│   AN3 (주행):  {self.joystick_data['AN3']:3d} → {an3_norm:+.2f}
│   AN4 (예비):  {self.joystick_data['AN4']:3d} → {an4_norm:+.2f}
│
│ [현재 위치]
│   하부체 횡이동: {self.current_positions['lateral']:+.3f} m
│   상부체 X축:   {self.current_positions['x']:+.3f} m
│   상부체 Y축:   {self.current_positions['y']:+.3f} m
│   상부체 Z축:   {self.current_positions['z']:+.3f} m
│   공구 Yaw:    {math.degrees(self.current_positions['yaw']):+.1f}°
│
│ [스위치 상태]
│   모드: S19={'ON' if self.switch_data['S19'] else 'off'} S20={'ON' if self.switch_data['S20'] else 'off'}
│   횡이동: S17={'ON' if self.switch_data['S17'] else 'off'} S18={'ON' if self.switch_data['S18'] else 'off'}
│   작업: S21={'ON' if self.switch_data['S21'] else 'off'} S22={'ON' if self.switch_data['S22'] else 'off'}
│   Yaw: S23={'ON' if self.switch_data['S23'] else 'off'} S24={'ON' if self.switch_data['S24'] else 'off'}
│   트리거: S00={'ON' if self.switch_data['S00'] else 'off'} S01={'ON' if self.switch_data['S01'] else 'off'}
└─────────────────────────────────────────────────────────────────────┘
"""
        print(status)  # 터미널에 직접 출력 (로거 대신)
    
    def destroy_node(self):
        """노드 종료"""
        if hasattr(self, 'can_bus'):
            self.can_bus.shutdown()
            self.get_logger().info('CAN bus closed')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = IronMDTeleopNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
        import traceback
        traceback.print_exc()
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
