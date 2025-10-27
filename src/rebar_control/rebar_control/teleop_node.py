#!/usr/bin/env python3
"""
철근 결속 로봇 텔레옵 노드
Iron-MD 무선 조종기로 로봇의 모든 하드웨어 단동 제어

조종기 매핑:
- 왼쪽 스틱 Y: 전진/후진 (주행)
- 왼쪽 스틱 X: 좌/우 회전 (주행)
- 오른쪽 스틱 Y: Z축 (상하)
- 오른쪽 스틱 X: X축 (좌우)
- D-Pad 상하: Y축 (전후)
- D-Pad 좌우: Yaw 회전
- L1: 리프팅 상승
- L2: 리프팅 하강
- R1: 그리퍼 열기
- R2: 그리퍼 닫기
- X 버튼: 트리거 당김
- O 버튼: 트리거 해제
- SELECT: 비상 정지
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float64MultiArray, Int32, Bool
import math


class TeleopNode(Node):
    """무선 조종기로 로봇 단동 제어"""
    
    def __init__(self):
        super().__init__('teleop_node')
        
        # 파라미터
        self.declare_parameter('max_linear_speed', 0.5)  # m/s
        self.declare_parameter('max_angular_speed', 1.0)  # rad/s
        self.declare_parameter('xyz_step_size', 0.01)  # m, 10mm per command
        self.declare_parameter('yaw_step_size', 0.1)  # rad
        self.declare_parameter('lifting_step_size', 0.05)  # m, 50mm per step
        self.declare_parameter('trigger_duration', 0.5)  # seconds
        
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.xyz_step = self.get_parameter('xyz_step_size').value
        self.yaw_step = self.get_parameter('yaw_step_size').value
        self.lift_step = self.get_parameter('lifting_step_size').value
        self.trigger_duration = self.get_parameter('trigger_duration').value
        
        # 조이스틱 입력 구독
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        
        # 주행 제어 (0x141, 0x142)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 개별 관절 제어 (0x143-0x147)
        self.joint1_pub = self.create_publisher(Float64MultiArray, '/joint_1/position', 10)  # Lifting
        self.joint2_pub = self.create_publisher(Float64MultiArray, '/joint_2/position', 10)  # X-axis
        self.joint3_pub = self.create_publisher(Float64MultiArray, '/joint_3/position', 10)  # Y-axis
        self.joint4_pub = self.create_publisher(Float64MultiArray, '/joint_4/position', 10)  # Z-axis
        self.joint5_pub = self.create_publisher(Float64MultiArray, '/joint_5/position', 10)  # Yaw
        
        # Pololu 액추에이터 (트리거)
        self.trigger_pub = self.create_publisher(Float32, '/motor_0/vel', 10)
        
        # Seengrip 그리퍼
        self.gripper_pos_pub = self.create_publisher(Float32, '/gripper/position', 10)
        self.gripper_cmd_pub = self.create_publisher(Int32, '/gripper/command', 10)
        
        # 비상 정지
        self.estop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        
        # 현재 위치 저장
        self.current_positions = {
            'lifting': 0.0,  # joint_1
            'x': 0.0,        # joint_2
            'y': 0.0,        # joint_3
            'z': 0.0,        # joint_4
            'yaw': 0.0,      # joint_5
        }
        
        # 이전 버튼 상태 (엣지 감지용)
        self.prev_buttons = []
        
        # 트리거 타이머
        self.trigger_timer = None
        
        # 안전 플래그
        self.emergency_stopped = False
        
        self.get_logger().info('Teleop node started - Ready for joystick input')
        self.print_help()
    
    def print_help(self):
        """조종기 매핑 도움말"""
        help_text = """
╔═══════════════════════════════════════════════════════════════╗
║          철근 결속 로봇 무선 조종기 매핑                      ║
╠═══════════════════════════════════════════════════════════════╣
║ [주행 제어]                                                   ║
║   왼쪽 스틱 Y축    : 전진/후진                                ║
║   왼쪽 스틱 X축    : 좌/우 회전                               ║
║                                                               ║
║ [XYZ 스테이지]                                                ║
║   오른쪽 스틱 X축  : X축 이동 (좌/우)                         ║
║   D-Pad 상/하      : Y축 이동 (전/후)                         ║
║   오른쪽 스틱 Y축  : Z축 이동 (상/하)                         ║
║   D-Pad 좌/우      : Yaw 회전                                 ║
║                                                               ║
║ [리프팅]                                                      ║
║   L1 버튼          : 상승 (50mm)                              ║
║   L2 버튼          : 하강 (50mm)                              ║
║                                                               ║
║ [그리퍼]                                                      ║
║   R1 버튼          : 열기                                     ║
║   R2 버튼          : 닫기                                     ║
║                                                               ║
║ [결속 건]                                                     ║
║   X 버튼           : 트리거 당김                              ║
║   O 버튼           : 트리거 해제                              ║
║                                                               ║
║ [안전]                                                        ║
║   SELECT 버튼      : 비상 정지                                ║
║   START 버튼       : 비상 정지 해제                           ║
╚═══════════════════════════════════════════════════════════════╝
"""
        self.get_logger().info(help_text)
    
    def joy_callback(self, msg):
        """조이스틱 입력 처리"""
        if self.emergency_stopped:
            # START 버튼으로 해제
            if self.button_pressed(msg, 9):  # START
                self.emergency_stopped = False
                self.get_logger().info('⚠️  비상 정지 해제')
            else:
                return  # 비상 정지 중에는 다른 명령 무시
        
        # 비상 정지 (SELECT 버튼)
        if self.button_pressed(msg, 8):  # SELECT
            self.emergency_stop()
            return
        
        # 1. 주행 제어 (연속 제어)
        self.handle_driving(msg)
        
        # 2. XYZ 스테이지 (스텝 제어)
        self.handle_xyz_stage(msg)
        
        # 3. 리프팅 제어 (스텝 제어)
        self.handle_lifting(msg)
        
        # 4. 그리퍼 제어
        self.handle_gripper(msg)
        
        # 5. 트리거 제어
        self.handle_trigger(msg)
        
        # 이전 버튼 상태 저장
        self.prev_buttons = list(msg.buttons)
    
    def button_pressed(self, msg, button_idx):
        """버튼 눌림 감지 (엣지)"""
        if button_idx >= len(msg.buttons):
            return False
        
        if len(self.prev_buttons) <= button_idx:
            return msg.buttons[button_idx] == 1
        
        return msg.buttons[button_idx] == 1 and self.prev_buttons[button_idx] == 0
    
    def handle_driving(self, msg):
        """주행 제어 (0x141, 0x142)"""
        # 왼쪽 스틱: axes[1]=전후, axes[0]=좌우
        if len(msg.axes) < 2:
            return
        
        linear_y = msg.axes[1]  # 전진/후진
        angular_z = msg.axes[0]  # 좌/우 회전
        
        # 데드존 적용
        if abs(linear_y) < 0.1:
            linear_y = 0.0
        if abs(angular_z) < 0.1:
            angular_z = 0.0
        
        # cmd_vel 발행
        twist = Twist()
        twist.linear.x = linear_y * self.max_linear
        twist.angular.z = angular_z * self.max_angular
        
        self.cmd_vel_pub.publish(twist)
    
    def handle_xyz_stage(self, msg):
        """XYZ 스테이지 제어 (0x144, 0x145, 0x146)"""
        # X축: 오른쪽 스틱 X (axes[2])
        # Y축: D-Pad 상하 (axes[7])
        # Z축: 오른쪽 스틱 Y (axes[3])
        # Yaw: D-Pad 좌우 (axes[6])
        
        if len(msg.axes) < 8:
            return
        
        # X축 (joint_2)
        x_input = msg.axes[2]
        if abs(x_input) > 0.5:  # 스틱을 충분히 움직였을 때만
            self.current_positions['x'] += self.xyz_step * (1 if x_input > 0 else -1)
            self.publish_joint_position('x', self.joint2_pub)
        
        # Y축 (joint_3)
        y_input = msg.axes[7]
        if abs(y_input) > 0.5:
            self.current_positions['y'] += self.xyz_step * (1 if y_input > 0 else -1)
            self.publish_joint_position('y', self.joint3_pub)
        
        # Z축 (joint_4)
        z_input = msg.axes[3]
        if abs(z_input) > 0.5:
            self.current_positions['z'] += self.xyz_step * (1 if z_input > 0 else -1)
            self.publish_joint_position('z', self.joint4_pub)
        
        # Yaw (joint_5)
        yaw_input = msg.axes[6]
        if abs(yaw_input) > 0.5:
            self.current_positions['yaw'] += self.yaw_step * (1 if yaw_input > 0 else -1)
            self.publish_joint_position('yaw', self.joint5_pub)
    
    def handle_lifting(self, msg):
        """리프팅 제어 (0x143)"""
        # L1: 상승 (버튼 4)
        # L2: 하강 (버튼 6)
        
        if len(msg.buttons) < 7:
            return
        
        if self.button_pressed(msg, 4):  # L1
            self.current_positions['lifting'] += self.lift_step
            self.publish_joint_position('lifting', self.joint1_pub)
            self.get_logger().info(f'⬆️  리프팅 상승: {self.current_positions["lifting"]:.3f}m')
        
        if self.button_pressed(msg, 6):  # L2
            self.current_positions['lifting'] -= self.lift_step
            self.publish_joint_position('lifting', self.joint1_pub)
            self.get_logger().info(f'⬇️  리프팅 하강: {self.current_positions["lifting"]:.3f}m')
    
    def handle_gripper(self, msg):
        """그리퍼 제어"""
        # R1: 열기 (버튼 5)
        # R2: 닫기 (버튼 7)
        
        if len(msg.buttons) < 8:
            return
        
        if self.button_pressed(msg, 5):  # R1
            gripper_msg = Float32()
            gripper_msg.data = 0.0  # 열기
            self.gripper_pos_pub.publish(gripper_msg)
            self.get_logger().info('🔓 그리퍼 열기')
        
        if self.button_pressed(msg, 7):  # R2
            gripper_msg = Float32()
            gripper_msg.data = 1.0  # 닫기
            self.gripper_pos_pub.publish(gripper_msg)
            self.get_logger().info('🔒 그리퍼 닫기')
    
    def handle_trigger(self, msg):
        """트리거 제어 (Pololu)"""
        # X 버튼: 당김 (버튼 0)
        # O 버튼: 해제 (버튼 1)
        
        if len(msg.buttons) < 2:
            return
        
        if self.button_pressed(msg, 0):  # X 버튼
            self.trigger_pull()
        
        if self.button_pressed(msg, 1):  # O 버튼
            self.trigger_release()
    
    def trigger_pull(self):
        """트리거 당기기 (자동으로 일정 시간 후 해제)"""
        self.get_logger().info('🔫 트리거 당김!')
        
        # 트리거 당김 (정방향)
        trigger_msg = Float32()
        trigger_msg.data = 1.0
        self.trigger_pub.publish(trigger_msg)
        
        # 타이머 설정 - 일정 시간 후 자동 해제
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
    
    def publish_joint_position(self, joint_name, publisher):
        """관절 위치 명령 발행"""
        msg = Float64MultiArray()
        msg.data = [self.current_positions[joint_name]]
        publisher.publish(msg)
        
        self.get_logger().info(
            f'📍 {joint_name.upper()}: {self.current_positions[joint_name]:.3f}'
        )
    
    def emergency_stop(self):
        """비상 정지"""
        self.emergency_stopped = True
        self.get_logger().error('🚨 비상 정지 활성화!')
        
        # 모든 모터 정지
        # 주행 정지
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
        # 트리거 정지
        self.trigger_release()
        
        # 비상 정지 신호 발행
        estop_msg = Bool()
        estop_msg.data = True
        self.estop_pub.publish(estop_msg)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = TeleopNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
