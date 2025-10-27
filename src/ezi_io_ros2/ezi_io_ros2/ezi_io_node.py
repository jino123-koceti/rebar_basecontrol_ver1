#!/usr/bin/env python3
"""
EZI-IO ROS2 노드
FASTECH EZI-IO-EN-L16O16N-T I/O 모듈에서 리미트 센서 읽기
Modbus TCP를 사용한 입력 상태 모니터링
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class EziIoNode(Node):
    """EZI-IO 리미트 센서 모니터링 노드"""
    
    def __init__(self):
        super().__init__('ezi_io_node')
        
        # 파라미터 선언
        self.declare_parameter('ip_address', '192.168.0.2')
        self.declare_parameter('port', 502)
        self.declare_parameter('slave_id', 1)
        self.declare_parameter('update_rate', 20.0)  # Hz
        self.declare_parameter('input_start_address', 0)
        self.declare_parameter('input_count', 16)
        
        # 리미트 센서 매핑 (입력 채널 번호)
        self.declare_parameter('limit_x_min_channel', 0)
        self.declare_parameter('limit_x_max_channel', 1)
        self.declare_parameter('limit_y_min_channel', 2)
        self.declare_parameter('limit_y_max_channel', 3)
        self.declare_parameter('limit_z_min_channel', 4)
        self.declare_parameter('limit_z_max_channel', 5)
        
        # 파라미터 가져오기
        self.ip_address = self.get_parameter('ip_address').value
        self.port = self.get_parameter('port').value
        self.slave_id = self.get_parameter('slave_id').value
        self.update_rate = self.get_parameter('update_rate').value
        self.input_start = self.get_parameter('input_start_address').value
        self.input_count = self.get_parameter('input_count').value
        
        # 리미트 채널
        self.limit_channels = {
            'x_min': self.get_parameter('limit_x_min_channel').value,
            'x_max': self.get_parameter('limit_x_max_channel').value,
            'y_min': self.get_parameter('limit_y_min_channel').value,
            'y_max': self.get_parameter('limit_y_max_channel').value,
            'z_min': self.get_parameter('limit_z_min_channel').value,
            'z_max': self.get_parameter('limit_z_max_channel').value,
        }
        
        # Modbus 클라이언트
        self.modbus_client = None
        self.connect_modbus()
        
        # 발행자 생성 - 개별 리미트 센서
        self.limit_publishers = {}
        for name in self.limit_channels.keys():
            topic = f'/limit_sensors/{name}'
            self.limit_publishers[name] = self.create_publisher(Bool, topic, 10)
        
        # 전체 입력 상태 발행
        self.all_inputs_pub = self.create_publisher(
            DiagnosticArray,
            '/ezi_io/diagnostics',
            10
        )
        
        # 타이머 - 주기적으로 입력 읽기
        timer_period = 1.0 / self.update_rate
        self.timer = self.create_timer(timer_period, self.read_inputs_callback)
        
        # 이전 상태 저장 (변화 감지용)
        self.previous_limits = {}
        
        self.get_logger().info(f'EZI-IO node started: {self.ip_address}:{self.port}')
    
    def connect_modbus(self):
        """Modbus TCP 연결"""
        try:
            from pymodbus.client import ModbusTcpClient
            
            self.modbus_client = ModbusTcpClient(
                host=self.ip_address,
                port=self.port,
                timeout=2
            )
            
            if self.modbus_client.connect():
                self.get_logger().info(f'Connected to EZI-IO at {self.ip_address}:{self.port}')
                return True
            else:
                self.get_logger().error(f'Failed to connect to {self.ip_address}:{self.port}')
                return False
                
        except ImportError:
            self.get_logger().error('pymodbus not installed! Install: pip install pymodbus')
            raise
        except Exception as e:
            self.get_logger().error(f'Modbus connection error: {e}')
            return False
    
    def read_inputs_callback(self):
        """입력 상태 읽기 및 발행"""
        if not self.modbus_client or not self.modbus_client.connected:
            if not self.connect_modbus():
                return
        
        try:
            # 입력 레지스터 읽기 (Discrete Inputs)
            # EZI-IO는 보통 read_discrete_inputs 또는 read_input_registers 사용
            result = self.modbus_client.read_discrete_inputs(
                address=self.input_start,
                count=self.input_count,
                slave=self.slave_id
            )
            
            if result.isError():
                # Discrete inputs 실패시 Coils 시도
                result = self.modbus_client.read_coils(
                    address=self.input_start,
                    count=self.input_count,
                    slave=self.slave_id
                )
            
            if result.isError():
                self.get_logger().warn(f'Failed to read inputs: {result}')
                return
            
            # 입력 비트 배열
            inputs = result.bits[:self.input_count]
            
            # 리미트 센서 상태 발행
            for name, channel in self.limit_channels.items():
                if channel < len(inputs):
                    state = inputs[channel]
                    
                    # 상태 변화시에만 로그
                    if name not in self.previous_limits or self.previous_limits[name] != state:
                        self.get_logger().info(f'Limit {name}: {"TRIGGERED" if state else "CLEAR"}')
                        self.previous_limits[name] = state
                    
                    # 발행
                    msg = Bool()
                    msg.data = bool(state)
                    self.limit_publishers[name].publish(msg)
            
            # 진단 메시지 발행
            self.publish_diagnostics(inputs)
            
        except Exception as e:
            self.get_logger().error(f'Error reading inputs: {e}')
            # 재연결 시도
            self.modbus_client.close()
            self.connect_modbus()
    
    def publish_diagnostics(self, inputs):
        """진단 정보 발행"""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        
        status = DiagnosticStatus()
        status.name = "EZI-IO Limit Sensors"
        status.hardware_id = f"{self.ip_address}:{self.port}"
        
        # 리미트 센서 상태
        any_triggered = False
        for name, channel in self.limit_channels.items():
            if channel < len(inputs):
                state = inputs[channel]
                status.values.append(KeyValue(
                    key=name,
                    value="TRIGGERED" if state else "CLEAR"
                ))
                if state:
                    any_triggered = True
        
        # 전체 상태 레벨
        if any_triggered:
            status.level = DiagnosticStatus.WARN
            status.message = "Some limits triggered"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "All limits clear"
        
        diag_array.status.append(status)
        self.all_inputs_pub.publish(diag_array)
    
    def destroy_node(self):
        """노드 종료"""
        if self.modbus_client:
            self.modbus_client.close()
            self.get_logger().info('Modbus connection closed')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = EziIoNode()
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
