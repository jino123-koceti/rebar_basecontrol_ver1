#!/usr/bin/env python3
"""
CMD_VEL 제어 노드
0x141, 0x142 모터를 cmd_vel 토픽으로 제어
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import struct
import time
import threading
from typing import Dict, Optional

from .can_manager import CANManager
from .rmd_x4_protocol import RMDX4Protocol, CommandType


class CmdVelControlNode(Node):
    """CMD_VEL 제어 노드"""
    
    def __init__(self):
        super().__init__('cmd_vel_control_node')
        
        # 파라미터 선언
        self.declare_parameter('can_interface', 'can2')
        self.declare_parameter('left_motor_id', 0x141)
        self.declare_parameter('right_motor_id', 0x142)
        self.declare_parameter('wheel_radius', 0.1)  # 바퀴 반지름 (m)
        self.declare_parameter('wheel_base', 0.5)     # 바퀴 간 거리 (m)
        self.declare_parameter('max_linear_vel', 1.0)  # 최대 선속도 (m/s)
        self.declare_parameter('max_angular_vel', 1.0) # 최대 각속도 (rad/s)
        
        # 파라미터 가져오기
        self.can_interface = self.get_parameter('can_interface').value
        self.left_motor_id = self.get_parameter('left_motor_id').value
        self.right_motor_id = self.get_parameter('right_motor_id').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        
        # CAN 매니저 및 프로토콜 초기화
        self.can_manager = CANManager(self.can_interface)
        self.protocol = RMDX4Protocol()

        # 모터 상태 저장
        self.motor_states: Dict[int, Dict] = {
            self.left_motor_id: {'position': 0.0, 'velocity': 0.0, 'torque': 0.0},
            self.right_motor_id: {'position': 0.0, 'velocity': 0.0, 'torque': 0.0}
        }
        
        # 토픽 구독/발행
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.joint_state_publisher = self.create_publisher(
            JointState,
            'joint_states',
            10
        )
        
        self.motor_status_publisher = self.create_publisher(
            Float64MultiArray,
            'motor_status',
            10
        )

        # 개별 모터 RPM 발행용 퍼블리셔
        from std_msgs.msg import Float32
        self.left_rpm_publisher = self.create_publisher(
            Float32,
            f'motor_{hex(self.left_motor_id)}_rpm',
            10
        )
        self.right_rpm_publisher = self.create_publisher(
            Float32,
            f'motor_{hex(self.right_motor_id)}_rpm',
            10
        )

        # 타이머 설정
        self.status_timer = self.create_timer(0.1, self.publish_status)  # 10Hz
        self.motor_status_timer = self.create_timer(0.1, self.read_motor_status)  # 10Hz로 감소 (20Hz -> 10Hz)
        
        # CAN 연결 및 콜백 등록
        if self.can_manager.connect():
            self.get_logger().info(f"CAN 인터페이스 {self.can_interface} 연결 성공")

            # 모터 응답 콜백 등록
            # RMD-X4 프로토콜: 명령 ID와 응답 ID가 동일함
            # 예: 0x141로 명령 전송 -> 0x141로 응답 수신
            self.can_manager.register_callback(
                self.left_motor_id,
                self.left_motor_response_callback
            )
            self.get_logger().info(f"왼쪽 모터 콜백 등록: 0x{self.left_motor_id:03X}")

            self.can_manager.register_callback(
                self.right_motor_id,
                self.right_motor_response_callback
            )
            self.get_logger().info(f"오른쪽 모터 콜백 등록: 0x{self.right_motor_id:03X}")
            
            # 백그라운드 수신 스레드 시작
            self.can_manager.start_receive_thread()

            # 수신 스레드가 완전히 시작될 때까지 약간 대기
            time.sleep(0.1)

            # 모터 활성화
            self.enable_motors()
        else:
            self.get_logger().error(f"CAN 인터페이스 {self.can_interface} 연결 실패")
    
    def enable_motors(self):
        """모터 활성화 (브레이크 릴리즈 - RMD-X4는 자동 서보온)"""
        # 브레이크 릴리즈 (동기식 - 응답 확인)
        brake_release_cmd = self.protocol.create_system_command(CommandType.BRAKE_RELEASE)

        # 왼쪽 모터
        self.get_logger().info(f"왼쪽 모터 0x{self.left_motor_id:03X} 브레이크 릴리즈 전송...")
        response = self.can_manager.send_and_receive(
            self.left_motor_id,
            brake_release_cmd,
            self.left_motor_id,  # RMD-X4는 명령 ID = 응답 ID
            timeout=1.0
        )

        if response:
            self.get_logger().info(f"✓ 왼쪽 모터 0x{self.left_motor_id:03X} 브레이크 릴리즈 성공! 응답: {response.hex().upper()}")
        else:
            self.get_logger().warning(f"❌ 왼쪽 모터 0x{self.left_motor_id:03X} 브레이크 릴리즈 응답 없음")

        # 오른쪽 모터
        self.get_logger().info(f"오른쪽 모터 0x{self.right_motor_id:03X} 브레이크 릴리즈 전송...")
        response = self.can_manager.send_and_receive(
            self.right_motor_id,
            brake_release_cmd,
            self.right_motor_id,  # RMD-X4는 명령 ID = 응답 ID
            timeout=1.0
        )

        if response:
            self.get_logger().info(f"✓ 오른쪽 모터 0x{self.right_motor_id:03X} 브레이크 릴리즈 성공! 응답: {response.hex().upper()}")
        else:
            self.get_logger().warning(f"❌ 오른쪽 모터 0x{self.right_motor_id:03X} 브레이크 릴리즈 응답 없음")

        time.sleep(0.1)
    
    def cmd_vel_callback(self, msg: Twist):
        """CMD_VEL 콜백"""
        # 선속도와 각속도 제한
        linear_vel = max(-self.max_linear_vel, min(self.max_linear_vel, msg.linear.x))
        angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, msg.angular.z))

        self.get_logger().info(
            f"📥 CMD_VEL 수신: linear={msg.linear.x:.3f}, angular={msg.angular.z:.3f} "
            f"(제한 후: linear={linear_vel:.3f}, angular={angular_vel:.3f})"
        )

        # 정지 명령 확인 (linear와 angular가 모두 0)
        if abs(linear_vel) < 0.001 and abs(angular_vel) < 0.001:
            self.get_logger().info("🛑 정지 명령 감지! 정지 시퀀스 시작...")

            # CAN BUS-OFF 방지: 명령 간 충분한 지연 시간 확보
            # 속도 0 명령 전송 (왼쪽 -> 지연 -> 오른쪽 -> 지연)
            self.get_logger().info("  └─ 왼쪽 모터 정지...")
            self.send_speed_command(self.left_motor_id, 0, log_level='info')
            time.sleep(0.05)  # 50ms 대기 (모터 응답 시간 확보)

            self.get_logger().info("  └─ 오른쪽 모터 정지...")
            self.send_speed_command(self.right_motor_id, 0, log_level='info')
            time.sleep(0.05)  # 50ms 대기

            # MOTOR_STOP 명령 (개별 전송)
            self.get_logger().info("  └─ MOTOR_STOP 명령 전송...")
            stop_cmd = self.protocol.create_system_command(CommandType.MOTOR_STOP)
            self.can_manager.send_frame(self.left_motor_id, stop_cmd)
            time.sleep(0.05)
            self.can_manager.send_frame(self.right_motor_id, stop_cmd)
            time.sleep(0.05)

            self.get_logger().info("✅ CMD_VEL 정지 시퀀스 완료!")

            # 현재 모터 상태 출력
            self.get_logger().info(
                f"  └─ 현재 속도: 왼쪽={self.motor_states[self.left_motor_id]['velocity']:.1f}dps, "
                f"오른쪽={self.motor_states[self.right_motor_id]['velocity']:.1f}dps"
            )
            return

        # 차동 구동 계산
        left_wheel_vel = linear_vel - (angular_vel * self.wheel_base / 2.0)
        right_wheel_vel = linear_vel + (angular_vel * self.wheel_base / 2.0)

        # 바퀴 속도를 각속도(rad/s)로 변환
        left_wheel_angular_vel = left_wheel_vel / self.wheel_radius  # rad/s
        right_wheel_angular_vel = right_wheel_vel / self.wheel_radius  # rad/s

        # rad/s를 dps (degrees per second)로 변환
        left_dps = left_wheel_angular_vel * 180.0 / 3.14159  # rad/s -> dps
        right_dps = right_wheel_angular_vel * 180.0 / 3.14159  # rad/s -> dps

        # 모터 장착 방향 고려: 왼쪽 모터가 180도 반대로 장착되어 있음
        # 전진 시 왼쪽은 음의 방향, 오른쪽은 양의 방향으로 회전해야 함
        left_dps = -left_dps

        # dps를 RMD-X4 명령 단위로 변환 (0.01 dps/LSB)
        left_speed_control = int(left_dps * 100)  # int32_t
        right_speed_control = int(right_dps * 100)  # int32_t

        # 속도 명령 전송
        self.send_speed_command(self.left_motor_id, left_speed_control, log_level='info')
        self.send_speed_command(self.right_motor_id, right_speed_control, log_level='info')

        self.get_logger().info(
            f"🚗 CMD_VEL 주행: linear={linear_vel:.2f}m/s, angular={angular_vel:.2f}rad/s -> "
            f"왼쪽={left_dps:.1f}dps({left_speed_control}), 오른쪽={right_dps:.1f}dps({right_speed_control})"
        )
    
    def send_speed_command(self, motor_id: int, speed_control: int, log_level: str = 'debug'):
        """
        속도 명령 전송 (RMD-X4 0xA2 명령)

        Args:
            motor_id: 모터 ID
            speed_control: int32_t, 0.01dps/LSB 단위의 속도 값
            log_level: 로그 레벨 ('debug' 또는 'info')
        """
        # RMD-X4 0xA2 명령 형식:
        # DATA[0] = 0xA2 (명령)
        # DATA[1] = max_torque (0=무제한)
        # DATA[2] = 0x00
        # DATA[3] = 0x00
        # DATA[4:7] = speedControl (int32_t, little-endian)
        max_torque = 0x00  # 0 = 토크 제한 없음 (스톨 전류로 제한됨)

        command_data = struct.pack('<BBBB', CommandType.SET_MOTOR_SPEED, max_torque, 0x00, 0x00)
        command_data += struct.pack('<i', speed_control)  # int32_t, little-endian

        self.can_manager.send_frame(motor_id, command_data)

        log_msg = (
            f"    ├─ 모터 0x{motor_id:03X} 속도 명령 전송: {speed_control} (0.01dps 단위), "
            f"실제 속도: {speed_control * 0.01:.1f} dps, CAN 데이터: {command_data.hex().upper()}"
        )

        if log_level == 'info':
            self.get_logger().info(log_msg)
        else:
            self.get_logger().debug(log_msg)
    
    def read_motor_status(self):
        """모터 상태 읽기 (순차적 전송으로 CAN 버스 보호)"""
        # 상태 읽기 명령 전송
        status_cmd = self.protocol.create_system_command(CommandType.READ_MOTOR_STATUS)

        # 순차적으로 모터 상태 읽기 (CAN 버스 보호)
        success1 = self.can_manager.send_frame(self.left_motor_id, status_cmd)
        if success1:
            self.get_logger().debug(f"✓ 왼쪽 모터 상태 읽기 명령 전송 성공")
        else:
            self.get_logger().warning(f"❌ 왼쪽 모터 상태 읽기 명령 전송 실패")
        
        time.sleep(0.05)  # 50ms 지연으로 CAN 버스 보호 강화
        
        success2 = self.can_manager.send_frame(self.right_motor_id, status_cmd)
        if success2:
            self.get_logger().debug(f"✓ 오른쪽 모터 상태 읽기 명령 전송 성공")
        else:
            self.get_logger().warning(f"❌ 오른쪽 모터 상태 읽기 명령 전송 실패")
    
    def left_motor_response_callback(self, can_id: int, data: bytes):
        """왼쪽 모터 응답 콜백"""
        self.parse_motor_response(self.left_motor_id, data)
    
    def right_motor_response_callback(self, can_id: int, data: bytes):
        """오른쪽 모터 응답 콜백"""
        self.parse_motor_response(self.right_motor_id, data)
    
    def parse_motor_response(self, motor_id: int, data: bytes):
        """모터 응답 파싱"""
        try:
            if len(data) < 1:
                return

            command = data[0]

            if command == CommandType.READ_MOTOR_STATUS:
                # 상태 응답 파싱: [cmd][temp(1)][current(2)][speed(2)][angle(2)] = 8바이트
                if len(data) >= 8:
                    temperature = struct.unpack('<b', data[1:2])[0]  # 온도 (°C)
                    current = struct.unpack('<h', data[2:4])[0] * 0.01  # 전류 (A)
                    speed = struct.unpack('<h', data[4:6])[0]  # 속도 (dps)
                    angle = struct.unpack('<h', data[6:8])[0]  # 각도 (도)

                    self.motor_states[motor_id] = {
                        'position': float(angle),
                        'velocity': float(speed),
                        'torque': current
                    }

                    self.get_logger().debug(
                        f"모터 0x{motor_id:03X} 상태: 위치={angle}°, 속도={speed} dps, "
                        f"전류={current:.2f}A, 온도={temperature}°C"
                    )
                else:
                    self.get_logger().debug(f"모터 0x{motor_id:03X} 응답 데이터 길이 부족: {len(data)} bytes")
            elif command == CommandType.SET_MOTOR_SPEED:
                # 0xA2 속도 제어 응답: [cmd][temp(1)][current(2)][speed(2)][angle(2)]
                if len(data) >= 8:
                    temperature = struct.unpack('<b', data[1:2])[0]  # 온도 (°C)
                    current = struct.unpack('<h', data[2:4])[0] * 0.01  # 전류 (A)
                    speed = struct.unpack('<h', data[4:6])[0]  # 속도 (dps)
                    angle = struct.unpack('<h', data[6:8])[0]  # 각도 (도)

                    # 모터 상태 업데이트
                    self.motor_states[motor_id]['velocity'] = float(speed)

                    self.get_logger().info(
                        f"    └─ 모터 0x{motor_id:03X} 0xA2 응답: 속도={speed} dps, "
                        f"전류={current:.2f}A, 온도={temperature}°C, 각도={angle}°, "
                        f"RAW={data.hex().upper()}"
                    )
                else:
                    self.get_logger().warning(
                        f"모터 0x{motor_id:03X} 0xA2 응답 데이터 길이 부족: {len(data)} bytes"
                    )
            elif command in [CommandType.BRAKE_RELEASE, CommandType.MOTOR_ENABLE,
                           CommandType.MOTOR_STOP, CommandType.MOTOR_SHUTDOWN]:
                # 이러한 명령들은 단순 확인 응답만 함
                self.get_logger().info(
                    f"    └─ 모터 0x{motor_id:03X} 명령 0x{command:02X} 응답 수신, RAW={data.hex().upper()}"
                )
            else:
                self.get_logger().debug(f"모터 0x{motor_id:03X} 알 수 없는 명령 응답: 0x{command:02X}")

        except Exception as e:
            self.get_logger().error(f"모터 0x{motor_id:03X} 응답 파싱 오류: {e}")
    
    def publish_status(self):
        """상태 발행"""
        from std_msgs.msg import Float32

        # JointState 메시지 생성
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = 'base_link'

        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state.position = [
            self.motor_states[self.left_motor_id]['position'] * 3.14159 / 180.0,  # 도 -> 라디안
            self.motor_states[self.right_motor_id]['position'] * 3.14159 / 180.0
        ]
        joint_state.velocity = [
            self.motor_states[self.left_motor_id]['velocity'] * 2 * 3.14159 / 60.0,  # dps -> rad/s
            self.motor_states[self.right_motor_id]['velocity'] * 2 * 3.14159 / 60.0
        ]

        self.joint_state_publisher.publish(joint_state)

        # 모터 상태 발행
        motor_status = Float64MultiArray()
        motor_status.data = [
            self.motor_states[self.left_motor_id]['position'],
            self.motor_states[self.left_motor_id]['velocity'],
            self.motor_states[self.left_motor_id]['torque'],
            self.motor_states[self.right_motor_id]['position'],
            self.motor_states[self.right_motor_id]['velocity'],
            self.motor_states[self.right_motor_id]['torque']
        ]

        self.motor_status_publisher.publish(motor_status)

        # 개별 RPM 발행 (dps를 RPM으로 변환)
        left_rpm_msg = Float32()
        left_rpm_msg.data = self.motor_states[self.left_motor_id]['velocity'] * 60.0 / 360.0  # dps -> RPM
        self.left_rpm_publisher.publish(left_rpm_msg)

        right_rpm_msg = Float32()
        right_rpm_msg.data = self.motor_states[self.right_motor_id]['velocity'] * 60.0 / 360.0  # dps -> RPM
        self.right_rpm_publisher.publish(right_rpm_msg)
    
    def stop_motors(self):
        """모터 정지 (순차적 전송으로 CAN 버스 보호)"""
        self.get_logger().info("⏹ 모터 정지 명령 시작...")
        stop_cmd = self.protocol.create_system_command(CommandType.MOTOR_STOP)

        # 순차적으로 모터 정지 (CAN 버스 보호)
        success1 = self.can_manager.send_frame(self.left_motor_id, stop_cmd)
        if success1:
            self.get_logger().debug(f"✓ 왼쪽 모터 정지 명령 전송 성공")
        else:
            self.get_logger().warning(f"❌ 왼쪽 모터 정지 명령 전송 실패")
        
        time.sleep(0.1)  # 100ms 지연으로 CAN 버스 보호
        
        success2 = self.can_manager.send_frame(self.right_motor_id, stop_cmd)
        if success2:
            self.get_logger().debug(f"✓ 오른쪽 모터 정지 명령 전송 성공")
        else:
            self.get_logger().warning(f"❌ 오른쪽 모터 정지 명령 전송 실패")

        self.get_logger().info("✅ 모터 정지 명령 전송 완료")

    def shutdown_motors(self):
        """모터 셧다운 (순차적 전송으로 CAN 버스 보호)"""
        self.get_logger().info("🔌 모터 셧다운 명령 시작...")
        shutdown_cmd = self.protocol.create_system_command(CommandType.MOTOR_SHUTDOWN)

        # 순차적으로 모터 셧다운 (CAN 버스 보호)
        success1 = self.can_manager.send_frame(self.left_motor_id, shutdown_cmd)
        if success1:
            self.get_logger().debug(f"✓ 왼쪽 모터 셧다운 명령 전송 성공")
        else:
            self.get_logger().warning(f"❌ 왼쪽 모터 셧다운 명령 전송 실패")
        
        time.sleep(0.1)  # 100ms 지연으로 CAN 버스 보호
        
        success2 = self.can_manager.send_frame(self.right_motor_id, shutdown_cmd)
        if success2:
            self.get_logger().debug(f"✓ 오른쪽 모터 셧다운 명령 전송 성공")
        else:
            self.get_logger().warning(f"❌ 오른쪽 모터 셧다운 명령 전송 실패")

        self.get_logger().info("✅ 모터 셧다운 명령 전송 완료")
    
    def destroy_node(self):
        """노드 종료 시 정리"""
        self.stop_motors()
        time.sleep(0.1)
        self.shutdown_motors()
        self.can_manager.disconnect()
        super().destroy_node()


def main(args=None):
    """메인 함수"""
    rclpy.init(args=args)
    
    node = CmdVelControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("사용자에 의해 중단됨")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
