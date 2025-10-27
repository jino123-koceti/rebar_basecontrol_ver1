#!/usr/bin/env python3
"""
Seengrip ROS2 노드
Seengrip Optimum Gripper를 Modbus RTU로 제어
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, Int32
from sensor_msgs.msg import JointState
import serial
import time


# Seengrip Register Definitions
MB_COMMAND = 24
MB_GOAL_POS = 29
MB_GOAL_SPD = 30
MB_ACTUAL_POS = 26
MB_STATUS = 15
MB_FAULT = 16
MB_TEMP = 13

DRV_CMD_HOME = 3
DRV_CMD_STOP = 1


class SeengripNode(Node):
    """ROS2 노드로 Seengrip 그리퍼 제어"""
    
    def __init__(self):
        super().__init__('seengrip_node')
        
        # 파라미터 선언
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('slave_id', 1)
        self.declare_parameter('default_speed', 500)
        self.declare_parameter('open_position', 0)
        self.declare_parameter('close_position', 2000)
        
        # 파라미터 가져오기
        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.slave_id = self.get_parameter('slave_id').value
        self.default_speed = self.get_parameter('default_speed').value
        self.open_position = self.get_parameter('open_position').value
        self.close_position = self.get_parameter('close_position').value
        
        # 시리얼 연결
        self.ser = None
        self.connect()
        
        # 구독자 생성
        self.position_sub = self.create_subscription(
            Float32,
            '/gripper/position',
            self.position_callback,
            10
        )
        
        self.command_sub = self.create_subscription(
            Int32,
            '/gripper/command',
            self.command_callback,
            10
        )
        
        # 발행자 생성
        self.status_pub = self.create_publisher(
            JointState,
            '/gripper/joint_states',
            10
        )
        
        self.fault_pub = self.create_publisher(
            Int32,
            '/gripper/fault',
            10
        )
        
        # 타이머 - 10Hz로 상태 발행
        self.status_timer = self.create_timer(0.1, self.publish_status)
        
        self.get_logger().info(f'Seengrip node started on {self.serial_port}')
        
        # 초기화 - Home 수행
        time.sleep(0.5)
        self.home()
    
    def connect(self):
        """시리얼 포트 연결"""
        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            self.get_logger().info(f'Connected to {self.serial_port} at {self.baudrate} baud')
            time.sleep(0.1)
        except Exception as e:
            self.get_logger().error(f'Failed to open port {self.serial_port}: {e}')
            raise
    
    def calculate_crc(self, data):
        """Modbus CRC16 계산"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc
    
    def write_register(self, register, value):
        """레지스터 쓰기 (Function code 0x06)"""
        if not self.ser or not self.ser.is_open:
            self.get_logger().error('Serial port not open')
            return False
        
        data = bytearray([
            self.slave_id,
            0x06,  # Write single register
            (register >> 8) & 0xFF,
            register & 0xFF,
            (value >> 8) & 0xFF,
            value & 0xFF
        ])
        crc = self.calculate_crc(data)
        data.append(crc & 0xFF)
        data.append((crc >> 8) & 0xFF)
        
        try:
            self.ser.write(data)
            time.sleep(0.05)
            response = self.ser.read(100)
            return len(response) > 0
        except Exception as e:
            self.get_logger().error(f'Write register error: {e}')
            return False
    
    def read_register(self, register, count=1):
        """레지스터 읽기 (Function code 0x03)"""
        if not self.ser or not self.ser.is_open:
            return None
        
        data = bytearray([
            self.slave_id,
            0x03,  # Read holding registers
            (register >> 8) & 0xFF,
            register & 0xFF,
            (count >> 8) & 0xFF,
            count & 0xFF
        ])
        crc = self.calculate_crc(data)
        data.append(crc & 0xFF)
        data.append((crc >> 8) & 0xFF)
        
        try:
            self.ser.write(data)
            time.sleep(0.05)
            response = self.ser.read(100)
            
            if len(response) >= 5:
                # Response: [ID][Func][ByteCount][Data...][CRC]
                byte_count = response[2]
                if len(response) >= 3 + byte_count + 2:
                    # Extract data (2 bytes per register)
                    values = []
                    for i in range(count):
                        high = response[3 + i*2]
                        low = response[4 + i*2]
                        values.append((high << 8) | low)
                    return values
        except Exception as e:
            self.get_logger().error(f'Read register error: {e}')
        
        return None
    
    def home(self):
        """그리퍼 홈 이동"""
        self.get_logger().info('Homing gripper...')
        return self.write_register(MB_COMMAND, DRV_CMD_HOME)
    
    def set_position(self, position, speed=None):
        """그리퍼 위치 설정 (0-2000)"""
        if speed is None:
            speed = self.default_speed
        
        position = int(max(0, min(2000, position)))
        
        self.write_register(MB_GOAL_SPD, speed)
        time.sleep(0.01)
        result = self.write_register(MB_GOAL_POS, position)
        
        self.get_logger().info(f'Set position: {position} at speed {speed}')
        return result
    
    def open_gripper(self, speed=None):
        """그리퍼 열기"""
        return self.set_position(self.open_position, speed)
    
    def close_gripper(self, speed=None):
        """그리퍼 닫기"""
        return self.set_position(self.close_position, speed)
    
    def position_callback(self, msg):
        """위치 명령 콜백 (0.0-1.0 정규화)"""
        # 0.0 = open, 1.0 = close
        normalized = max(0.0, min(1.0, msg.data))
        position = int(normalized * (self.close_position - self.open_position) + self.open_position)
        self.set_position(position)
    
    def command_callback(self, msg):
        """명령 콜백"""
        cmd = msg.data
        if cmd == DRV_CMD_HOME:
            self.home()
        elif cmd == DRV_CMD_STOP:
            self.write_register(MB_COMMAND, DRV_CMD_STOP)
        elif cmd == 100:  # Custom: Open
            self.open_gripper()
        elif cmd == 101:  # Custom: Close
            self.close_gripper()
        else:
            self.get_logger().warn(f'Unknown command: {cmd}')
    
    def publish_status(self):
        """그리퍼 상태 발행"""
        # 현재 위치 읽기
        pos_data = self.read_register(MB_ACTUAL_POS, 1)
        
        if pos_data:
            # JointState 메시지 발행
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = ['gripper']
            joint_state.position = [float(pos_data[0])]
            
            self.status_pub.publish(joint_state)
        
        # 에러 상태 읽기 (주기적으로)
        if hasattr(self, '_fault_counter'):
            self._fault_counter += 1
        else:
            self._fault_counter = 0
        
        if self._fault_counter % 10 == 0:  # 1Hz
            fault_data = self.read_register(MB_FAULT, 1)
            if fault_data and fault_data[0] != 0:
                fault_msg = Int32()
                fault_msg.data = fault_data[0]
                self.fault_pub.publish(fault_msg)
                self.get_logger().warn(f'Gripper fault: {fault_data[0]}')
    
    def destroy_node(self):
        """노드 종료"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info('Serial port closed')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SeengripNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
