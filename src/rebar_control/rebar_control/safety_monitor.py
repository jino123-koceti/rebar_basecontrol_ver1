#!/usr/bin/env python3
"""
안전 모니터 노드
리미트 센서를 감시하고 위험 상황 시 모터 정지
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float64MultiArray


class SafetyMonitor(Node):
    """안전 모니터 - 리미트 센서 감시 및 안전 제어"""
    
    def __init__(self):
        super().__init__('safety_monitor')
        
        # 리미트 센서 상태
        self.limits = {
            'x_min': False,
            'x_max': False,
            'y_min': False,
            'y_max': False,
            'z_min': False,
            'z_max': False,
        }
        
        # 비상 정지 상태
        self.emergency_stop = False
        
        # 리미트 센서 구독
        for axis in self.limits.keys():
            self.create_subscription(
                Bool,
                f'/limit_sensors/{axis}',
                lambda msg, a=axis: self.limit_callback(a, msg),
                10
            )
        
        # 비상 정지 구독
        self.create_subscription(
            Bool,
            '/emergency_stop',
            self.estop_callback,
            10
        )
        
        # 안전 상태 발행
        self.safe_pub = self.create_publisher(Bool, '/system/safe', 10)
        
        # 타이머 - 주기적으로 안전 상태 체크
        self.create_timer(0.1, self.check_safety)
        
        self.get_logger().info('Safety monitor started')
    
    def limit_callback(self, axis, msg):
        """리미트 센서 콜백"""
        prev_state = self.limits[axis]
        self.limits[axis] = msg.data
        
        # 상태 변화 로깅
        if prev_state != msg.data:
            if msg.data:
                self.get_logger().warn(f'⚠️  LIMIT TRIGGERED: {axis.upper()}')
            else:
                self.get_logger().info(f'✓ Limit cleared: {axis.upper()}')
    
    def estop_callback(self, msg):
        """비상 정지 콜백"""
        if msg.data and not self.emergency_stop:
            self.get_logger().error('🚨 EMERGENCY STOP ACTIVATED')
            self.emergency_stop = True
        elif not msg.data and self.emergency_stop:
            self.get_logger().info('✓ Emergency stop released')
            self.emergency_stop = False
    
    def check_safety(self):
        """안전 상태 체크 및 발행"""
        # 어떤 리미트라도 트리거되었거나 비상정지 상태면 unsafe
        any_limit = any(self.limits.values())
        is_safe = not (any_limit or self.emergency_stop)
        
        # 안전 상태 발행
        msg = Bool()
        msg.data = is_safe
        self.safe_pub.publish(msg)
        
        # 위험 상태일 때 경고 (1초에 1번)
        if not is_safe and hasattr(self, '_warn_counter'):
            self._warn_counter += 1
            if self._warn_counter % 10 == 0:
                triggered = [k for k, v in self.limits.items() if v]
                if triggered:
                    self.get_logger().warn(f'Limits active: {", ".join(triggered)}')
        else:
            self._warn_counter = 0
    
    def is_motion_safe(self, direction):
        """
        특정 방향으로의 움직임이 안전한지 체크
        direction: 'x+', 'x-', 'y+', 'y-', 'z+', 'z-'
        """
        if self.emergency_stop:
            return False
        
        axis = direction[0]
        sign = direction[1]
        
        # 최대 리미트로 움직이려는지 체크
        if sign == '+' and self.limits[f'{axis}_max']:
            return False
        
        # 최소 리미트로 움직이려는지 체크
        if sign == '-' and self.limits[f'{axis}_min']:
            return False
        
        return True


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SafetyMonitor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
