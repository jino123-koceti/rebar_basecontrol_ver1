#!/usr/bin/env python3
"""
통합 제어 노드
- 0x141, 0x142: cmd_vel 속도 제어
- 0x143, 0x144, 0x145, 0x146, 0x147: 위치 제어
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
import struct
import time
import threading
from typing import Dict, Optional, List

from .can_manager import CANManager
from .rmd_x4_protocol import RMDX4Protocol, CommandType


class PositionControlNode(Node):
    """통합 제어 노드 (위치제어 + cmd_vel 속도제어)"""

    def __init__(self):
        super().__init__('unified_control_node')

        # 디버그 로그 파일 설정 (에러/경고만 기록)
        import logging
        self.debug_logger = logging.getLogger('unified_control_debug')
        self.debug_logger.setLevel(logging.WARNING)  # WARNING 이상만 기록
        fh = logging.FileHandler('/tmp/unified_control_debug.log')
        fh.setLevel(logging.WARNING)
        formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        fh.setFormatter(formatter)
        self.debug_logger.addHandler(fh)
        self.debug_logger.warning("="*60)
        self.debug_logger.warning("Unified Control Node 시작")
        self.debug_logger.warning("="*60)

        # 파라미터 선언
        self.declare_parameter('can_interface', 'can2')
        self.declare_parameter('motor_ids', [0x143, 0x144, 0x145, 0x146, 0x147])
        self.declare_parameter('joint_names', ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5'])
        self.declare_parameter('max_position', 1800.0)  # 최대 위치 (도)
        self.declare_parameter('min_position', -1800.0)  # 최소 위치 (도)
        self.declare_parameter('max_velocity', 100.0)  # 최대 속도 (도/초)
        self.declare_parameter('position_tolerance', 1.0)  # 위치 허용 오차 (도)
        
        # CMD_VEL 파라미터 선언
        self.declare_parameter('left_motor_id', 0x141)
        self.declare_parameter('right_motor_id', 0x142)
        self.declare_parameter('wheel_radius', 0.1)  # 바퀴 반지름 (m)
        self.declare_parameter('wheel_base', 0.5)     # 바퀴 간 거리 (m)
        self.declare_parameter('max_linear_vel', 1.0)  # 최대 선속도 (m/s)
        self.declare_parameter('max_angular_vel', 1.0) # 최대 각속도 (rad/s)
        
        # 파라미터 가져오기
        self.can_interface = self.get_parameter('can_interface').value
        self.motor_ids = self.get_parameter('motor_ids').value
        self.joint_names = self.get_parameter('joint_names').value
        self.max_position = self.get_parameter('max_position').value
        self.min_position = self.get_parameter('min_position').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        
        # CMD_VEL 파라미터 가져오기
        self.left_motor_id = self.get_parameter('left_motor_id').value
        self.right_motor_id = self.get_parameter('right_motor_id').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        
        # CAN 매니저 및 프로토콜 초기화
        self.can_manager = CANManager(self.can_interface)
        self.protocol = RMDX4Protocol()

        # 모터 상태 저장 (위치제어 모터)
        self.motor_states: Dict[int, Dict] = {}
        self.target_positions: Dict[int, float] = {}
        self.current_positions: Dict[int, float] = {}

        for motor_id in self.motor_ids:
            self.motor_states[motor_id] = {
                'position': 0.0,
                'velocity': 0.0,
                'torque': 0.0,
                'target_position': 0.0,
                'is_moving': False
            }
            self.target_positions[motor_id] = 0.0
            self.current_positions[motor_id] = 0.0
        
        # CMD_VEL 모터 상태 저장
        self.motor_states[self.left_motor_id] = {'position': 0.0, 'velocity': 0.0, 'torque': 0.0}
        self.motor_states[self.right_motor_id] = {'position': 0.0, 'velocity': 0.0, 'torque': 0.0}
        
        # 토픽 구독/발행
        # 위치제어 토픽
        self.trajectory_subscription = self.create_subscription(
            JointTrajectory,
            'joint_trajectory',
            self.trajectory_callback,
            10
        )
        
        # CMD_VEL 토픽 추가
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

        # 개별 모터 위치 발행용 퍼블리셔
        from std_msgs.msg import Float32
        self.motor_position_publishers = {}
        for motor_id in self.motor_ids:
            topic_name = f'motor_{hex(motor_id)}_position'
            self.motor_position_publishers[motor_id] = self.create_publisher(
                Float32, topic_name, 10
            )
        
        # CMD_VEL 모터 RPM 발행용 퍼블리셔
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

        # 개별 관절 제어용 토픽들
        self.joint_position_subscriptions = []
        for i, joint_name in enumerate(self.joint_names):
            topic_name = f'{joint_name}/position'
            subscription = self.create_subscription(
                Float64MultiArray,
                topic_name,
                lambda msg, idx=i: self.single_joint_callback(msg, idx),
                10
            )
            self.joint_position_subscriptions.append(subscription)
        
        # 타이머 설정 (bus-off 방지: 모터 상태 읽기 비활성화)
        self.status_timer = self.create_timer(0.1, self.publish_status)  # 10Hz
        # self.motor_status_timer = self.create_timer(0.1, self.read_motor_status)  # 비활성화 (CAN 부하 감소)
        self.position_control_timer = self.create_timer(0.02, self.position_control_loop)  # 50Hz
        
        # 서비스 생성
        self.brake_release_service = self.create_service(
            Trigger, 
            'safe_brake_release', 
            self.brake_release_service_callback
        )
        
        # CAN 연결 및 콜백 등록
        if self.can_manager.connect():
            self.get_logger().info(f"✅ CAN {self.can_interface} 연결 성공 (통합 노드)")
            
            # 위치제어 모터 응답 콜백 등록 (0x143~0x147)
            for motor_id in self.motor_ids:
                motor_number = motor_id - 0x140
                response_id = 0x240 + motor_number
                self.can_manager.register_callback(
                    response_id,
                    lambda can_id, data, mid=motor_id: self.motor_response_callback(mid, data)
                )
            
            # CMD_VEL 모터 응답 콜백 등록 (0x141, 0x142 -> 0x241, 0x242로 응답)
            left_motor_number = self.left_motor_id - 0x140  # 0x141 - 0x140 = 1
            left_response_id = 0x240 + left_motor_number  # 0x240 + 1 = 0x241

            right_motor_number = self.right_motor_id - 0x140  # 0x142 - 0x140 = 2
            right_response_id = 0x240 + right_motor_number  # 0x240 + 2 = 0x242

            self.can_manager.register_callback(
                left_response_id,  # 0x241
                self.left_motor_response_callback
            )

            self.can_manager.register_callback(
                right_response_id,  # 0x242
                self.right_motor_response_callback
            )

            self.get_logger().info(f"✓ CMD_VEL 콜백 등록: 0x{left_response_id:03X}, 0x{right_response_id:03X}")
            
            # 백그라운드 수신 스레드 시작
            self.can_manager.start_receive_thread()

            # 수신 스레드가 완전히 시작될 때까지 약간 대기
            time.sleep(0.1)

            # 모터 활성화 (자동 브레이크 해제 비활성화 - GUI에서 수동 제어)
            # self.enable_motors()  # GUI에서 수동으로 브레이크 해제하므로 주석 처리
            self.get_logger().info("✅ 통합 노드 초기화 완료 (7개 모터)")
        else:
            self.get_logger().error(f"❌ CAN {self.can_interface} 연결 실패")
    
    def enable_motors(self):
        """모터 활성화 (브레이크 릴리즈 - 순차적 전송으로 CAN 버스 보호)"""
        self.get_logger().info("🔓 브레이크 해제 시작...")
        
        # 브레이크 릴리즈 명령
        brake_release_cmd = self.protocol.create_system_command(CommandType.BRAKE_RELEASE)

        # 순차적으로 브레이크 릴리즈 (CAN 버스 보호)
        for i, motor_id in enumerate(self.motor_ids):
            # 브레이크 릴리즈 명령 전송
            success = self.can_manager.send_frame(motor_id, brake_release_cmd)
            
            if not success:
                self.debug_logger.warning(f"❌ 모터 0x{motor_id:03X} 브레이크 해제 실패")
                self.get_logger().warning(f"❌ M{motor_id:03X} 브레이크 해제 실패")

            # 마지막 모터가 아니면 지연 시간 추가
            if i < len(self.motor_ids) - 1:
                time.sleep(0.1)  # 100ms 지연으로 CAN 버스 보호

        self.get_logger().info("✅ 브레이크 해제 완료")
        time.sleep(0.5)  # 모든 명령 처리 대기
    
    def safe_brake_release(self):
        """안전한 브레이크 해제 (모터 7개 대응)"""
        self.get_logger().info("🔓 브레이크 해제 중...")
        
        brake_release_cmd = self.protocol.create_system_command(CommandType.BRAKE_RELEASE)
        
        # 모터 7개에 대해 매우 안전하게 순차 처리
        failed_motors = []
        for i, motor_id in enumerate(self.motor_ids):
            # 각 모터마다 충분한 지연 시간 확보
            if i > 0:
                time.sleep(0.5)  # 500ms 지연으로 증가
            
            # 브레이크 해제 명령 전송
            success = self.can_manager.send_frame(motor_id, brake_release_cmd)
            
            if not success:
                failed_motors.append(motor_id)
                self.debug_logger.error(f"❌ 모터 0x{motor_id:03X} 브레이크 해제 실패")
        
        # 모든 명령 완료 후 충분한 대기 시간
        time.sleep(2.0)  # 2초 대기
        
        if failed_motors:
            self.get_logger().warning(f"⚠️ 브레이크 해제 실패: {[hex(m) for m in failed_motors]}")
        else:
            self.get_logger().info("✅ 브레이크 해제 완료")
    
    def brake_release_service_callback(self, request, response):
        """브레이크 해제 서비스 콜백"""
        try:
            self.get_logger().info("🔓 브레이크 해제 서비스 호출됨")
            self.safe_brake_release()
            
            response.success = True
            response.message = "브레이크 해제가 안전하게 완료되었습니다"
            return response
            
        except Exception as e:
            self.get_logger().error(f"❌ 브레이크 해제 서비스 오류: {e}")
            response.success = False
            response.message = f"브레이크 해제 실패: {e}"
            return response
    
    def trajectory_callback(self, msg: JointTrajectory):
        """궤적 명령 콜백"""
        self.debug_logger.info(f"📥 Trajectory 메시지 수신: joint_names={msg.joint_names}, points={len(msg.points)}")
        self.get_logger().info(f"📥 Trajectory 메시지 수신: {msg.joint_names}")

        if not msg.points:
            self.debug_logger.warning("❌ Trajectory 포인트가 없습니다")
            return

        # 첫 번째 포인트 사용 (간단한 구현)
        point = msg.points[0]
        self.debug_logger.info(f"📍 첫 번째 포인트: positions={point.positions}")

        # 관절 이름과 위치 매핑
        for i, joint_name in enumerate(msg.joint_names):
            if joint_name in self.joint_names:
                joint_index = self.joint_names.index(joint_name)
                if joint_index < len(self.motor_ids) and i < len(point.positions):
                    motor_id = self.motor_ids[joint_index]
                    target_position = point.positions[i] * 180.0 / 3.14159  # 라디안 -> 도

                    # 위치 제한
                    target_position = max(self.min_position, min(self.max_position, target_position))

                    self.target_positions[motor_id] = target_position
                    self.motor_states[motor_id]['target_position'] = target_position
                    self.motor_states[motor_id]['is_moving'] = True

                    self.debug_logger.info(
                        f"🎯 모터 0x{motor_id:03X} 목표 위치 설정: {target_position:.1f}도"
                    )

                    # 직접 위치 명령 전송
                    self.send_position_command(motor_id, target_position)

                    self.get_logger().info(
                        f"관절 {joint_name} (ID: 0x{motor_id:03X}) 목표 위치: {target_position:.1f}도"
                    )
    
    def single_joint_callback(self, msg: Float64MultiArray, joint_index: int):
        """단일 관절 위치 명령 콜백"""
        if joint_index >= len(self.motor_ids) or not msg.data:
            return
        
        motor_id = self.motor_ids[joint_index]
        target_position = msg.data[0] * 180.0 / 3.14159  # 라디안 -> 도
        
        # 위치 제한
        target_position = max(self.min_position, min(self.max_position, target_position))
        
        self.target_positions[motor_id] = target_position
        self.motor_states[motor_id]['target_position'] = target_position
        self.motor_states[motor_id]['is_moving'] = True
        
        self.get_logger().info(
            f"관절 {self.joint_names[joint_index]} (ID: 0x{motor_id:03X}) 목표 위치: {target_position:.1f}도"
        )
    
    def position_control_loop(self):
        """위치 제어 루프 - 목표 도달 확인"""
        for motor_id in self.motor_ids:
            if not self.motor_states[motor_id]['is_moving']:
                continue

            current_pos = self.motor_states[motor_id]['position']
            target_pos = self.target_positions[motor_id]

            # 위치 오차 계산
            position_error = abs(target_pos - current_pos)

            # 목표 위치에 도달했는지 확인
            # 0xA8 명령은 한 번 전송하면 모터가 자동으로 목표 위치까지 이동하므로
            # 오차가 크더라도 명령을 재전송하지 않음 (일정 시간 후 자동으로 is_moving=False로 설정)
            # 단, 매우 큰 오차가 있고 오랜 시간이 지났다면 재전송
            if position_error < self.position_tolerance:
                self.motor_states[motor_id]['is_moving'] = False
                self.get_logger().info(
                    f"모터 0x{motor_id:03X} 목표 위치 도달: {current_pos:.1f}도 (목표: {target_pos:.1f}도)"
                )
    
    def read_motor_error(self, motor_id: int):
        """모터 에러 읽기 (0x9A 명령)"""
        error_read_cmd = bytes([0x9A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        self.can_manager.send_frame(motor_id, error_read_cmd)
        self.get_logger().warning(f"🔍 0x{motor_id:03X} 에러 읽기 명령 전송")
    
    def send_speed_command(self, motor_id: int, velocity: float):
        """속도 명령 전송 (도/초)"""
        # RMD-X4 속도 명령 형식: [0xA2][0x00][speed(4bytes float, dps)]
        # speed: 도/초 단위
        command_data = bytes([self.COMMANDS['SET_MOTOR_SPEED'], 0x00]) + \
                      struct.pack('<f', velocity) + \
                      b'\x00\x00'  # 패딩

        self.can_manager.send_frame(motor_id, command_data)

        self.get_logger().debug(
            f"모터 0x{motor_id:03X} 속도 명령: {velocity:.1f} dps"
        )
    
    def send_position_command(self, motor_id: int, target_position: float):
        """
        위치 명령 전송 (0xA8 Incremental Position Control 사용)
        현재 위치와 목표 위치의 차이(상대 이동)를 계산하여 전송
        """
        # 현재 위치 가져오기
        current_position = self.motor_states[motor_id]['position']

        # 상대 이동 각도 계산
        incremental_angle = target_position - current_position

        # 이동 각도가 너무 작으면 무시 (노이즈 방지)
        if abs(incremental_angle) < 0.1:
            self.get_logger().info(
                f"모터 0x{motor_id:03X} 이동 각도가 너무 작아 무시: {incremental_angle:.2f}도"
            )
            self.motor_states[motor_id]['is_moving'] = False
            return

        # 0xA8 명령 형식:
        # DATA[0] = 0xA8 (명령 바이트)
        # DATA[1] = 0x00 (NULL)
        # DATA[2:3] = maxSpeed (uint16_t, dps)
        # DATA[4:7] = angleControl (int32_t, 0.01 degree/LSB)

        max_speed_dps = 500  # 최대 속도 500 dps

        # 각도를 프로토콜 단위로 변환 (0.01 degree/LSB)
        # 주의: 모터 방향 반전 (부호 반대로)
        angle_control = int(-incremental_angle * 100)

        # int32_t로 변환 (부호 있는 32비트 정수)
        if angle_control < 0:
            angle_control = (1 << 32) + angle_control

        # 데이터 패킷 생성
        command_data = struct.pack('<BB H I',
                                  CommandType.SET_MOTOR_INCREMENTAL_POSITION,  # 0xA8
                                  0x00,  # NULL
                                  max_speed_dps,  # 최대 속도 (uint16_t)
                                  angle_control & 0xFFFFFFFF  # 각도 제어 (int32_t)
        )

        self.can_manager.send_frame(motor_id, command_data)

        # 명령 전송 후 위치 업데이트를 위해 0x92 멀티턴 각도 읽기
        time.sleep(0.02)  # 모터 응답 대기
        multi_turn_cmd = self.protocol.create_system_command(CommandType.READ_MULTI_TURN_ANGLE)
        self.can_manager.send_frame(motor_id, multi_turn_cmd)

        self.debug_logger.info(
            f"📤 0xA8 명령 전송: 모터 0x{motor_id:03X}, "
            f"현재={current_position:.1f}도, 목표={target_position:.1f}도, "
            f"상대이동={incremental_angle:.1f}도, 속도={max_speed_dps} dps, "
            f"데이터={command_data.hex().upper()}"
        )

        self.get_logger().info(
            f"모터 0x{motor_id:03X} 위치 명령 전송 (0xA8): "
            f"현재={current_position:.1f}도, 목표={target_position:.1f}도, "
            f"상대이동={incremental_angle:.1f}도, 속도={max_speed_dps} dps"
        )
    
    def send_stop_command(self, motor_id: int):
        """정지 명령 전송"""
        stop_cmd = self.protocol.create_system_command(CommandType.MOTOR_STOP)
        self.can_manager.send_frame(motor_id, stop_cmd)

    def read_motor_status(self):
        """모터 상태 읽기 (순차적 전송으로 CAN 버스 보호)"""
        status_cmd = self.protocol.create_system_command(CommandType.READ_MOTOR_STATUS)

        # 순차적으로 모터 상태 읽기 (CAN 버스 보호)
        for i, motor_id in enumerate(self.motor_ids):
            self.debug_logger.debug(f"📤 0x9C 명령 전송: 모터 0x{motor_id:03X}")
            success = self.can_manager.send_frame(motor_id, status_cmd)
            
            if success:
                self.debug_logger.debug(f"✓ 모터 0x{motor_id:03X} 상태 읽기 명령 전송 성공")
            else:
                self.debug_logger.warning(f"❌ 모터 0x{motor_id:03X} 상태 읽기 명령 전송 실패")
            
            # 마지막 모터가 아니면 지연 시간 추가
            if i < len(self.motor_ids) - 1:
                time.sleep(0.05)  # 50ms 지연으로 CAN 버스 보호 강화
    
    def motor_response_callback(self, motor_id: int, data: bytes):
        """모터 응답 콜백"""
        try:
            if len(data) < 1:
                self.debug_logger.warning(f"❌ 모터 0x{motor_id:03X} 응답 데이터 없음")
                return

            command = data[0]

            # RAW 응답은 debug 레벨로 (터미널 로그 방지)
            self.get_logger().debug(
                f"📥 [RAW] 모터 0x{motor_id:03X} 응답: 명령=0x{command:02X}, "
                f"데이터={data.hex().upper()}, 길이={len(data)}"
            )

            if command == CommandType.READ_MULTI_TURN_ANGLE:
                # 0x92 멀티턴 각도 응답 파싱: [cmd][reserved(3)][angle(4)] = 8바이트
                # angle: int32, 0.01도/LSB, 누적 각도
                if len(data) >= 8:
                    # CAN 로그 분석 결과: 멀티턴 각도는 data[4:8]에 위치
                    # 예: 920000004A1F0000 -> [4A, 1F, 00, 00] = 0x00001F4A = 7946 (0.01도) = 79.46도
                    angle_raw = struct.unpack('<i', data[4:8])[0]  # int32, 0.01도 단위
                    angle_degrees = angle_raw * 0.01  # 도 단위로 변환

                    # 상태 업데이트
                    self.motor_states[motor_id]['position'] = float(angle_degrees)
                    self.current_positions[motor_id] = float(angle_degrees)

                    # 0x92 응답은 debug 레벨로 (터미널 로그 방지)
                    self.get_logger().debug(
                        f"모터 0x{motor_id:03X} 0x92 멀티턴: 위치={angle_degrees:.1f}°, RAW={angle_raw}"
                    )
                else:
                    self.get_logger().warning(f"모터 0x{motor_id:03X} 0x92 응답 데이터 길이 부족: {len(data)} bytes")
            elif command == CommandType.READ_MOTOR_STATUS:
                # 상태 응답 파싱: [cmd][temp(1)][current(2)][speed(2)][angle(2)] = 8바이트
                if len(data) >= 8:
                    temperature = struct.unpack('<b', data[1:2])[0]  # 온도 (°C)
                    current = struct.unpack('<h', data[2:4])[0] * 0.01  # 전류 (A)
                    speed = struct.unpack('<h', data[4:6])[0]  # 속도 (dps)
                    angle = struct.unpack('<h', data[6:8])[0]  # 각도 (도, -180~+180)

                    # 0x9C는 단일 회전 각도이므로 위치 업데이트 안 함
                    self.motor_states[motor_id]['velocity'] = float(speed)
                    self.motor_states[motor_id]['torque'] = current

                    # 0x9C 상태 응답은 모두 debug 레벨로 (터미널 로그 방지)
                    self.get_logger().debug(
                        f"모터 0x{motor_id:03X} 0x9C 상태: 단일각도={angle}°, 속도={speed} dps, "
                        f"전류={current:.2f}A, 온도={temperature}°C"
                    )
                else:
                    self.get_logger().warning(f"모터 0x{motor_id:03X} 응답 데이터 길이 부족: {len(data)} bytes")
            elif command == CommandType.SET_MOTOR_INCREMENTAL_POSITION:
                # 0xA8 명령 응답 파싱: [cmd][temp(1)][current(2)][speed(2)][angle(2)] = 8바이트
                # 주의: angle은 단일 회전 각도(-180~+180)이므로 절대 위치 업데이트에 사용하지 않음!
                if len(data) >= 8:
                    temperature = struct.unpack('<b', data[1:2])[0]  # 온도 (°C)
                    current = struct.unpack('<h', data[2:4])[0] * 0.01  # 전류 (A)
                    speed = struct.unpack('<h', data[4:6])[0]  # 속도 (dps)
                    angle = struct.unpack('<h', data[6:8])[0]  # 단일 회전 각도 (도, -180~+180)

                    # 속도와 전류만 업데이트 (위치는 업데이트하지 않음!)
                    self.motor_states[motor_id]['velocity'] = float(speed)
                    self.motor_states[motor_id]['torque'] = current

                    # 0xA8 응답도 debug 레벨로 (터미널 로그 방지)
                    self.get_logger().debug(
                        f"모터 0x{motor_id:03X} 0xA8 응답: 속도={speed} dps, "
                        f"전류={current:.2f}A, 온도={temperature}°C, 단일각도={angle}°"
                    )
            elif command in [CommandType.BRAKE_RELEASE, CommandType.MOTOR_ENABLE,
                           CommandType.MOTOR_STOP, CommandType.MOTOR_SHUTDOWN,
                           CommandType.SET_MOTOR_SPEED, CommandType.SET_MOTOR_POSITION]:
                # 이러한 명령들은 단순 확인 응답만 함
                self.get_logger().debug(f"모터 0x{motor_id:03X} 명령 0x{command:02X} 응답 수신")
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

        joint_state.name = self.joint_names
        joint_state.position = []
        joint_state.velocity = []

        for motor_id in self.motor_ids:
            # 도를 라디안으로 변환
            position_rad = self.motor_states[motor_id]['position'] * 3.14159 / 180.0
            velocity_rad = self.motor_states[motor_id]['velocity'] * 2 * 3.14159 / 60.0  # dps -> rad/s

            joint_state.position.append(position_rad)
            joint_state.velocity.append(velocity_rad)

            # 개별 모터 위치 발행 (도 단위)
            if motor_id in self.motor_position_publishers:
                position_msg = Float32()
                position_msg.data = self.motor_states[motor_id]['position']
                self.motor_position_publishers[motor_id].publish(position_msg)

                # 위치 발행 로그는 debug 레벨로 (터미널 로그 방지)
                self.get_logger().debug(
                    f"위치 발행: 모터 0x{motor_id:03X} -> {self.motor_states[motor_id]['position']:.1f}° "
                    f"(토픽: motor_{hex(motor_id)}_position)"
                )

        self.joint_state_publisher.publish(joint_state)

        # 모터 상태 발행
        motor_status = Float64MultiArray()
        motor_status.data = []

        for motor_id in self.motor_ids:
            motor_status.data.extend([
                self.motor_states[motor_id]['position'],
                self.motor_states[motor_id]['velocity'],
                self.motor_states[motor_id]['torque'],
                self.motor_states[motor_id]['target_position'],
                1.0 if self.motor_states[motor_id]['is_moving'] else 0.0
            ])

        self.motor_status_publisher.publish(motor_status)
    
    def stop_all_motors(self):
        """모든 모터 정지 (순차적 전송으로 CAN 버스 보호)"""
        self.get_logger().info("⏹ 모든 모터 정지 명령 시작...")
        stop_cmd = self.protocol.create_system_command(CommandType.MOTOR_STOP)

        # 순차적으로 모터 정지 (CAN 버스 보호)
        for i, motor_id in enumerate(self.motor_ids):
            success = self.can_manager.send_frame(motor_id, stop_cmd)
            self.motor_states[motor_id]['is_moving'] = False
            
            if success:
                self.get_logger().debug(f"✓ 모터 0x{motor_id:03X} 정지 명령 전송 성공")
            else:
                self.get_logger().warning(f"❌ 모터 0x{motor_id:03X} 정지 명령 전송 실패")
            
            # 마지막 모터가 아니면 지연 시간 추가
            if i < len(self.motor_ids) - 1:
                time.sleep(0.1)  # 100ms 지연으로 CAN 버스 보호

        self.get_logger().info("✅ 모든 모터 정지 명령 전송 완료")

    def shutdown_all_motors(self):
        """모든 모터 셧다운 (순차적 전송으로 CAN 버스 보호)"""
        self.get_logger().info("🔌 모든 모터 셧다운 명령 시작...")
        shutdown_cmd = self.protocol.create_system_command(CommandType.MOTOR_SHUTDOWN)

        # 순차적으로 모터 셧다운 (CAN 버스 보호)
        for i, motor_id in enumerate(self.motor_ids):
            success = self.can_manager.send_frame(motor_id, shutdown_cmd)
            
            if success:
                self.get_logger().debug(f"✓ 모터 0x{motor_id:03X} 셧다운 명령 전송 성공")
            else:
                self.get_logger().warning(f"❌ 모터 0x{motor_id:03X} 셧다운 명령 전송 실패")
            
            # 마지막 모터가 아니면 지연 시간 추가
            if i < len(self.motor_ids) - 1:
                time.sleep(0.1)  # 100ms 지연으로 CAN 버스 보호

        self.get_logger().info("✅ 모든 모터 셧다운 명령 전송 완료")
    
    # ============================================================
    # CMD_VEL 제어 함수들
    # ============================================================
    
    def cmd_vel_callback(self, msg: Twist):
        """CMD_VEL 콜백 (0x141, 0x142 모터 속도 제어)"""
        # 선속도와 각속도 제한
        linear_vel = max(-self.max_linear_vel, min(self.max_linear_vel, msg.linear.x))
        angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, msg.angular.z))

        # 정지 명령 확인
        if abs(linear_vel) < 0.001 and abs(angular_vel) < 0.001:
            # 속도 0 명령으로 정지 (0xA2)
            self.get_logger().warning(f"🛑 정지 명령: 0x141, 0x142 (속도=0)")
            self.debug_logger.warning(f"🛑 정지 명령: 0x141, 0x142 (속도=0)")
            
            # 0x141 속도 0
            self.send_speed_command(self.left_motor_id, 0)
            self.get_logger().warning(f"   → 0x141 속도=0 전송")
            self.debug_logger.warning(f"   → 0x141 속도=0 전송")
            time.sleep(0.01)
            
            # 0x142 속도 0
            self.send_speed_command(self.right_motor_id, 0)
            self.get_logger().warning(f"   → 0x142 속도=0 전송")
            self.debug_logger.warning(f"   → 0x142 속도=0 전송")
            
            return

        # 차동 구동 계산
        left_wheel_vel = linear_vel - (angular_vel * self.wheel_base / 2.0)
        right_wheel_vel = linear_vel + (angular_vel * self.wheel_base / 2.0)

        # RPM 계산 (rad/s -> RPM)
        left_rpm = (left_wheel_vel / self.wheel_radius) * 60.0 / (2 * 3.14159)
        right_rpm = (right_wheel_vel / self.wheel_radius) * 60.0 / (2 * 3.14159)

        # dps (도/초) 계산
        left_dps = left_rpm * 6.0
        right_dps = right_rpm * 6.0

        # 0x141(왼쪽) 모터 방향 반전 (180도 반대 장착)
        left_dps = -left_dps

        # 속도 제어값 변환 (0.01dps/LSB)
        left_speed_control = int(left_dps * 100)
        right_speed_control = int(right_dps * 100)

        # 속도 명령 전송
        self.send_speed_command(self.left_motor_id, left_speed_control)
        time.sleep(0.05)
        self.send_speed_command(self.right_motor_id, right_speed_control)
    
    def send_speed_command(self, motor_id: int, speed_control: int, log_stop: bool = False):
        """속도 명령 전송 (0xA2 - Speed Control Command)"""
        data = bytearray(8)
        data[0] = 0xA2  # Speed Control Command
        data[1] = 0x00
        data[2] = 0x00
        data[3] = 0x00
        data[4] = speed_control & 0xFF
        data[5] = (speed_control >> 8) & 0xFF
        data[6] = (speed_control >> 16) & 0xFF
        data[7] = (speed_control >> 24) & 0xFF

        success = self.can_manager.send_frame(motor_id, data)
        
        if log_stop and speed_control == 0:
            if success:
                self.get_logger().info(f"    ✓ 0x{motor_id:03X} 속도=0 전송 성공")
            else:
                self.get_logger().warning(f"    ❌ 0x{motor_id:03X} 속도=0 전송 실패")
        elif not success:
            self.get_logger().warning(f"❌ 모터 0x{motor_id:03X} 속도 명령 전송 실패")
    
    def left_motor_response_callback(self, can_id: int, data: bytes):
        """왼쪽 모터 (0x141) 응답 콜백"""
        if len(data) >= 8:
            try:
                cmd = data[0]
                
                # 에러 읽기 응답 (0x9A)
                if cmd == 0x9A:
                    error_code = data[7]
                    error_msgs = {
                        0x00: "정상",
                        0x01: "과전류",
                        0x02: "과전압",
                        0x03: "엔코더 에러",
                        0x04: "과열",
                        0x08: "저전압",
                        0x10: "홀센서 에러",
                        0x20: "과부하"
                    }
                    error_msg = error_msgs.get(error_code, f"알 수 없는 에러 (0x{error_code:02X})")
                    self.get_logger().error(f"🔴 0x141 에러: {error_msg} (코드: 0x{error_code:02X})")
                    self.debug_logger.error(f"🔴 0x141 에러: {error_msg}, RAW={data.hex().upper()}")
                    return
                
                # 속도 명령 응답 감지 (0xA2)
                if cmd == 0xA2:
                    temperature = int.from_bytes([data[1]], byteorder='little', signed=True)
                    torque_raw = int.from_bytes(data[2:4], byteorder='little', signed=True)
                    speed_raw = int.from_bytes(data[4:6], byteorder='little', signed=True)
                    torque = torque_raw * 0.01
                    speed = speed_raw
                    
                    # 비정상 값 감지 시 에러 읽기
                    if abs(torque) > 10.0 or temperature > 80:
                        self.get_logger().error(f"⚠️ 0x141 비정상: torque={torque:.2f}A, temp={temperature}°C")
                        self.read_motor_error(self.left_motor_id)
                    
                    self.get_logger().warning(f"✓ 0x141 속도 응답: speed={speed:.1f}dps, torque={torque:.2f}A")
                    self.debug_logger.warning(f"✓ 0x141 속도 응답: speed={speed:.1f}dps, torque={torque:.2f}A, RAW={data.hex().upper()}")
                
                temperature = int.from_bytes([data[1]], byteorder='little', signed=True)
                torque_raw = int.from_bytes(data[2:4], byteorder='little', signed=True)
                speed_raw = int.from_bytes(data[4:6], byteorder='little', signed=True)
                
                torque = torque_raw * 0.01  # A
                speed = speed_raw  # dps
                
                self.motor_states[self.left_motor_id]['velocity'] = speed
                self.motor_states[self.left_motor_id]['torque'] = torque
                
                # RPM 발행
                from std_msgs.msg import Float32
                rpm_msg = Float32()
                rpm_msg.data = speed / 6.0  # dps -> RPM
                self.left_rpm_publisher.publish(rpm_msg)
                
            except Exception as e:
                self.get_logger().error(f"0x141 응답 파싱 오류: {e}")
                self.debug_logger.error(f"0x141 응답 파싱 오류: {e}")
    
    def right_motor_response_callback(self, can_id: int, data: bytes):
        """오른쪽 모터 (0x142) 응답 콜백"""
        if len(data) >= 8:
            try:
                cmd = data[0]
                
                # 에러 읽기 응답 (0x9A)
                if cmd == 0x9A:
                    error_code = data[7]
                    error_msgs = {
                        0x00: "정상",
                        0x01: "과전류",
                        0x02: "과전압",
                        0x03: "엔코더 에러",
                        0x04: "과열",
                        0x08: "저전압",
                        0x10: "홀센서 에러",
                        0x20: "과부하"
                    }
                    error_msg = error_msgs.get(error_code, f"알 수 없는 에러 (0x{error_code:02X})")
                    self.get_logger().error(f"🔴 0x142 에러: {error_msg} (코드: 0x{error_code:02X})")
                    self.debug_logger.error(f"🔴 0x142 에러: {error_msg}, RAW={data.hex().upper()}")
                    return
                
                # 속도 명령 응답 감지 (0xA2) - 0x142 특별 추적
                if cmd == 0xA2:
                    temperature = int.from_bytes([data[1]], byteorder='little', signed=True)
                    torque_raw = int.from_bytes(data[2:4], byteorder='little', signed=True)
                    speed_raw = int.from_bytes(data[4:6], byteorder='little', signed=True)
                    torque = torque_raw * 0.01
                    speed = speed_raw
                    
                    # 비정상 값 감지 시 에러 읽기
                    if abs(torque) > 10.0 or temperature > 80:
                        self.get_logger().error(f"⚠️ 0x142 비정상: torque={torque:.2f}A, temp={temperature}°C")
                        self.read_motor_error(self.right_motor_id)
                    
                    self.get_logger().warning(f"★ 0x142 속도 응답: speed={speed:.1f}dps, torque={torque:.2f}A")
                    self.debug_logger.warning(f"★ 0x142 속도 응답: speed={speed:.1f}dps, torque={torque:.2f}A, RAW={data.hex().upper()}")
                    
                    # 속도가 0이 아니면 경고
                    if abs(speed) > 1.0:
                        self.get_logger().warning(f"⚠⚠⚠ 0x142 정지 실패! 여전히 회전 중: {speed:.1f}dps")
                        self.debug_logger.warning(f"⚠⚠⚠ 0x142 정지 실패! 여전히 회전 중: {speed:.1f}dps")
                
                temperature = int.from_bytes([data[1]], byteorder='little', signed=True)
                torque_raw = int.from_bytes(data[2:4], byteorder='little', signed=True)
                speed_raw = int.from_bytes(data[4:6], byteorder='little', signed=True)
                
                torque = torque_raw * 0.01  # A
                speed = speed_raw  # dps
                
                self.motor_states[self.right_motor_id]['velocity'] = speed
                self.motor_states[self.right_motor_id]['torque'] = torque
                
                # RPM 발행
                from std_msgs.msg import Float32
                rpm_msg = Float32()
                rpm_msg.data = speed / 6.0  # dps -> RPM
                self.right_rpm_publisher.publish(rpm_msg)
                
            except Exception as e:
                self.get_logger().error(f"0x142 응답 파싱 오류: {e}")
                self.debug_logger.error(f"0x142 응답 파싱 오류: {e}")
    
    def destroy_node(self):
        """노드 종료 시 정리"""
        # 모든 모터 정지 (위치제어 + 속도제어)
        self.get_logger().info("🛑 모든 모터 정지 중...")
        
        # CMD_VEL 모터 정지
        self.send_speed_command(self.left_motor_id, 0)
        time.sleep(0.05)
        self.send_speed_command(self.right_motor_id, 0)
        time.sleep(0.1)
        
        # 위치제어 모터 정지
        self.stop_all_motors()
        time.sleep(0.1)
        self.shutdown_all_motors()
        
        self.can_manager.disconnect()
        super().destroy_node()


def main(args=None):
    """메인 함수"""
    rclpy.init(args=args)
    
    node = PositionControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("사용자에 의해 중단됨")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
