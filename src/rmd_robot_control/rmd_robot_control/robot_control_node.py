#!/usr/bin/env python3
"""
통합 로봇 제어 노드
CMD_VEL과 위치제어를 통합하여 관리
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import time
from typing import Dict, List


class RobotControlNode(Node):
    """통합 로봇 제어 노드"""
    
    def __init__(self):
        super().__init__('robot_control_node')
        
        # 파라미터 선언
        self.declare_parameter('can_interface', 'can2')
        self.declare_parameter('cmd_vel_motor_ids', [0x141, 0x142])
        self.declare_parameter('position_motor_ids', [0x143, 0x144, 0x145, 0x146, 0x147])
        self.declare_parameter('joint_names', ['left_wheel', 'right_wheel', 'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5'])
        
        # 파라미터 가져오기
        self.can_interface = self.get_parameter('can_interface').value
        self.cmd_vel_motor_ids = self.get_parameter('cmd_vel_motor_ids').value
        self.position_motor_ids = self.get_parameter('position_motor_ids').value
        self.joint_names = self.get_parameter('joint_names').value
        
        # 모터 상태 저장
        self.motor_states: Dict[int, Dict] = {}
        self.joint_states: Dict[str, float] = {}
        
        # 모든 모터 ID 통합
        all_motor_ids = self.cmd_vel_motor_ids + self.position_motor_ids
        
        for motor_id in all_motor_ids:
            self.motor_states[motor_id] = {
                'position': 0.0,
                'velocity': 0.0,
                'torque': 0.0,
                'target_position': 0.0,
                'is_moving': False
            }
        
        # 관절 상태 초기화
        for joint_name in self.joint_names:
            self.joint_states[joint_name] = 0.0
        
        # 토픽 구독/발행
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.trajectory_subscription = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory',
            self.trajectory_callback,
            10
        )
        
        # 통합된 관절 상태 발행
        self.joint_state_publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        # 모터 상태 발행
        self.motor_status_publisher = self.create_publisher(
            Float64MultiArray,
            '/motor_status',
            10
        )
        
        # 개별 관절 제어용 토픽들
        self.joint_position_subscriptions = []
        position_joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
        
        for i, joint_name in enumerate(position_joint_names):
            topic_name = f'/{joint_name}/position'
            subscription = self.create_subscription(
                Float64MultiArray,
                topic_name,
                lambda msg, idx=i: self.single_joint_callback(msg, idx),
                10
            )
            self.joint_position_subscriptions.append(subscription)
        
        # 타이머 설정
        self.status_timer = self.create_timer(0.1, self.publish_integrated_status)
        
        self.get_logger().info("통합 로봇 제어 노드 시작")
        self.get_logger().info(f"CMD_VEL 모터 ID: {[hex(x) for x in self.cmd_vel_motor_ids]}")
        self.get_logger().info(f"위치제어 모터 ID: {[hex(x) for x in self.position_motor_ids]}")
    
    def cmd_vel_callback(self, msg: Twist):
        """CMD_VEL 콜백"""
        # CMD_VEL은 별도 노드에서 처리되므로 여기서는 로깅만
        self.get_logger().debug(
            f"CMD_VEL 수신: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}"
        )
    
    def trajectory_callback(self, msg: JointTrajectory):
        """궤적 명령 콜백"""
        if not msg.points:
            return
        
        # 궤적은 별도 노드에서 처리되므로 여기서는 로깅만
        point = msg.points[0]
        self.get_logger().debug(
            f"궤적 명령 수신: 관절={msg.joint_names}, 위치={point.positions}"
        )
    
    def single_joint_callback(self, msg: Float64MultiArray, joint_index: int):
        """단일 관절 위치 명령 콜백"""
        if joint_index >= len(self.position_motor_ids) or not msg.data:
            return
        
        motor_id = self.position_motor_ids[joint_index]
        target_position = msg.data[0] * 180.0 / 3.14159  # 라디안 -> 도
        
        self.motor_states[motor_id]['target_position'] = target_position
        self.motor_states[motor_id]['is_moving'] = True
        
        self.get_logger().debug(
            f"관절 {joint_index} (ID: 0x{motor_id:03X}) 목표 위치: {target_position:.1f}도"
        )
    
    def publish_integrated_status(self):
        """통합된 상태 발행"""
        # 통합된 JointState 메시지 생성
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = 'base_link'
        
        joint_state.name = self.joint_names
        joint_state.position = []
        joint_state.velocity = []
        
        # CMD_VEL 모터 상태 추가
        for motor_id in self.cmd_vel_motor_ids:
            position_rad = self.motor_states[motor_id]['position'] * 3.14159 / 180.0
            velocity_rad = self.motor_states[motor_id]['velocity'] * 2 * 3.14159 / 60.0
            
            joint_state.position.append(position_rad)
            joint_state.velocity.append(velocity_rad)
        
        # 위치제어 모터 상태 추가
        for motor_id in self.position_motor_ids:
            position_rad = self.motor_states[motor_id]['position'] * 3.14159 / 180.0
            velocity_rad = self.motor_states[motor_id]['velocity'] * 2 * 3.14159 / 60.0
            
            joint_state.position.append(position_rad)
            joint_state.velocity.append(velocity_rad)
        
        self.joint_state_publisher.publish(joint_state)
        
        # 통합된 모터 상태 발행
        motor_status = Float64MultiArray()
        motor_status.data = []
        
        # 모든 모터 상태 추가
        all_motor_ids = self.cmd_vel_motor_ids + self.position_motor_ids
        
        for motor_id in all_motor_ids:
            motor_status.data.extend([
                self.motor_states[motor_id]['position'],
                self.motor_states[motor_id]['velocity'],
                self.motor_states[motor_id]['torque'],
                self.motor_states[motor_id]['target_position'],
                1.0 if self.motor_states[motor_id]['is_moving'] else 0.0
            ])
        
        self.motor_status_publisher.publish(motor_status)
    
    def update_motor_state(self, motor_id: int, position: float, velocity: float, torque: float):
        """모터 상태 업데이트"""
        if motor_id in self.motor_states:
            self.motor_states[motor_id]['position'] = position
            self.motor_states[motor_id]['velocity'] = velocity
            self.motor_states[motor_id]['torque'] = torque
    
    def get_motor_status(self, motor_id: int) -> Dict:
        """모터 상태 조회"""
        return self.motor_states.get(motor_id, {})
    
    def get_all_motor_status(self) -> Dict[int, Dict]:
        """모든 모터 상태 조회"""
        return self.motor_states.copy()


def main(args=None):
    """메인 함수"""
    rclpy.init(args=args)
    
    node = RobotControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("사용자에 의해 중단됨")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
