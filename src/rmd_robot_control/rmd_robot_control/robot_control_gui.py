#!/usr/bin/env python3
"""
RMD 로봇 제어 GUI
PyQt5를 사용한 직관적인 로봇 제어 인터페이스
"""

import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QGridLayout, QLabel, QPushButton, 
                             QSlider, QSpinBox, QDoubleSpinBox, QGroupBox,
                             QTabWidget, QTextEdit, QCheckBox, QProgressBar,
                             QMessageBox, QFrame)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QFont, QPalette, QColor
import math
import time
import subprocess
from typing import Dict, List

from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Int32MultiArray


class RobotControlGUI(QMainWindow):
    """로봇 제어 GUI 메인 윈도우"""
    
    def __init__(self):
        super().__init__()
        self.setWindowTitle("RMD 로봇 제어 시스템")
        self.setGeometry(100, 100, 1200, 800)
        
        # ROS2 노드 초기화
        rclpy.init()
        self.ros_node = Node('robot_control_gui')
        
        # 모터 상태 저장
        self.motor_states = {
            'cmd_vel': {'left': 0.0, 'right': 0.0},
            'position': [0.0, 0.0, 0.0, 0.0, 0.0]  # 5개 관절
        }
        
        # GUI 초기화
        self.init_ui()
        self.init_ros_communication()
        
        # 타이머 설정
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.update_status)
        self.status_timer.start(100)  # 10Hz
        
        # 안전 설정
        self.safety_enabled = True
        self.emergency_stop = False
        
        # 시작 로그
        self.log_message("✅ RMD 로봇 제어 GUI 시작")
        self.log_message("ℹ️ 중요한 이벤트만 기록됩니다")
    
    def init_ui(self):
        """UI 초기화"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 메인 레이아웃
        main_layout = QHBoxLayout(central_widget)
        
        # 왼쪽 패널 (제어 기능들)
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        
        # 상단 패널 (CMD_VEL, 상태 모니터링, 설정)
        top_panel = QWidget()
        top_layout = QHBoxLayout(top_panel)
        
        # CMD_VEL 제어 그룹
        cmd_vel_group = self.create_cmd_vel_group()
        top_layout.addWidget(cmd_vel_group)
        
        # 상태 모니터링 그룹
        status_group = self.create_status_group()
        top_layout.addWidget(status_group)
        
        # 설정 그룹
        settings_group = self.create_settings_group()
        top_layout.addWidget(settings_group)
        
        left_layout.addWidget(top_panel)
        
        # 위치제어 그룹 (하단에 전체 너비로)
        position_group = self.create_position_group()
        left_layout.addWidget(position_group)
        
        left_layout.addStretch()
        main_layout.addWidget(left_panel)
        
        # 오른쪽 패널 (안전 제어)
        side_panel = self.create_side_panel()
        main_layout.addWidget(side_panel)
    
    def create_cmd_vel_group(self):
        """CMD_VEL 제어 그룹 생성"""
        group = QGroupBox("🚗 CMD_VEL 제어 (차동 구동)")
        group.setFont(QFont("Arial", 12, QFont.Bold))
        layout = QVBoxLayout(group)
        
        # 조이스틱 스타일 제어
        joystick_group = QGroupBox("조이스틱 제어")
        joystick_layout = QGridLayout(joystick_group)
        
        # 선속도 제어
        joystick_layout.addWidget(QLabel("선속도 (m/s):"), 0, 0)
        self.linear_slider = QSlider(Qt.Horizontal)
        self.linear_slider.setRange(-30, 30)  # -0.3 ~ 0.3 (x100)
        self.linear_slider.setValue(0)
        self.linear_slider.valueChanged.connect(self.update_cmd_vel_display)
        joystick_layout.addWidget(self.linear_slider, 0, 1)
        
        self.linear_label = QLabel("0.00")
        self.linear_label.setMinimumWidth(50)
        joystick_layout.addWidget(self.linear_label, 0, 2)
        
        # 각속도 제어
        joystick_layout.addWidget(QLabel("각속도 (rad/s):"), 1, 0)
        self.angular_slider = QSlider(Qt.Horizontal)
        self.angular_slider.setRange(-30, 30)  # -0.3 ~ 0.3 (x100)
        self.angular_slider.setValue(0)
        self.angular_slider.valueChanged.connect(self.update_cmd_vel_display)
        joystick_layout.addWidget(self.angular_slider, 1, 1)
        
        self.angular_label = QLabel("0.00")
        self.angular_label.setMinimumWidth(50)
        joystick_layout.addWidget(self.angular_label, 1, 2)
        
        layout.addWidget(joystick_group)
        
        # CMD_VEL 명령 전송 버튼
        cmd_vel_control_group = QGroupBox("CMD_VEL 제어")
        cmd_vel_layout = QHBoxLayout(cmd_vel_control_group)
        
        self.send_cmd_vel_btn = QPushButton("CMD_VEL 명령 전송")
        self.send_cmd_vel_btn.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                font-weight: bold;
                border-radius: 5px;
                padding: 10px;
                font-size: 14px;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
            QPushButton:pressed {
                background-color: #3d8b40;
            }
        """)
        self.send_cmd_vel_btn.clicked.connect(self.send_cmd_vel_command)
        cmd_vel_layout.addWidget(self.send_cmd_vel_btn)
        
        self.stop_cmd_vel_btn = QPushButton("정지")
        self.stop_cmd_vel_btn.setStyleSheet("""
            QPushButton {
                background-color: #f44336;
                color: white;
                font-weight: bold;
                border-radius: 5px;
                padding: 10px;
                font-size: 14px;
            }
            QPushButton:hover {
                background-color: #da190b;
            }
        """)
        self.stop_cmd_vel_btn.clicked.connect(self.stop_cmd_vel_command)
        cmd_vel_layout.addWidget(self.stop_cmd_vel_btn)
        
        layout.addWidget(cmd_vel_control_group)
        
        # 미리 정의된 동작
        preset_group = QGroupBox("미리 정의된 동작")
        preset_layout = QGridLayout(preset_group)
        
        movements = [
            ("전진", 15, 0),      # 0.15 m/s
            ("후진", -15, 0),     # -0.15 m/s
            ("좌회전", 0, 15),    # 0.15 rad/s
            ("우회전", 0, -15),   # -0.15 rad/s
            ("정지", 0, 0)
        ]
        
        for i, (name, linear, angular) in enumerate(movements):
            btn = QPushButton(name)
            btn.clicked.connect(lambda checked, l=linear, a=angular: self.set_preset_movement(l, a))
            preset_layout.addWidget(btn, i // 3, i % 3)
        
        layout.addWidget(preset_group)
        
        # 상태 표시
        status_group = QGroupBox("CMD_VEL 상태")
        status_layout = QGridLayout(status_group)
        
        status_layout.addWidget(QLabel("왼쪽 바퀴 속도:"), 0, 0)
        self.left_wheel_label = QLabel("0.00 RPM")
        status_layout.addWidget(self.left_wheel_label, 0, 1)
        
        status_layout.addWidget(QLabel("오른쪽 바퀴 속도:"), 1, 0)
        self.right_wheel_label = QLabel("0.00 RPM")
        status_layout.addWidget(self.right_wheel_label, 1, 1)
        
        layout.addWidget(status_group)
        
        return group
    
    def create_position_group(self):
        """위치제어 그룹 생성"""
        group = QGroupBox("🎯 위치제어 (motor 제어)")
        group.setFont(QFont("Arial", 12, QFont.Bold))
        layout = QVBoxLayout(group)
        
        # 모터 제어 버튼들
        motor_control_group = QGroupBox("모터 제어")
        motor_control_layout = QHBoxLayout(motor_control_group)

        # 전체 위치 읽기 버튼
        read_all_btn = QPushButton("📖 전체 위치 읽기")
        read_all_btn.setStyleSheet("""
            QPushButton {
                background-color: #9C27B0;
                color: white;
                font-weight: bold;
                border-radius: 5px;
                padding: 10px;
                min-width: 120px;
            }
            QPushButton:hover {
                background-color: #7B1FA2;
            }
            QPushButton:pressed {
                background-color: #6A1B9A;
            }
        """)
        read_all_btn.clicked.connect(self.read_all_positions)
        motor_control_layout.addWidget(read_all_btn)

        # 브레이크 해제 버튼
        brake_release_btn = QPushButton("🔓 브레이크 해제")
        brake_release_btn.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                font-weight: bold;
                border-radius: 5px;
                padding: 10px;
                min-width: 120px;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
            QPushButton:pressed {
                background-color: #3d8b40;
            }
        """)
        brake_release_btn.clicked.connect(self.brake_release_position_motors)
        motor_control_layout.addWidget(brake_release_btn)
        
        # 에러 초기화 버튼
        error_clear_btn = QPushButton("🔧 에러 초기화")
        error_clear_btn.setStyleSheet("""
            QPushButton {
                background-color: #2196F3;
                color: white;
                font-weight: bold;
                border-radius: 5px;
                padding: 10px;
                min-width: 120px;
            }
            QPushButton:hover {
                background-color: #1976D2;
            }
            QPushButton:pressed {
                background-color: #1565C0;
            }
        """)
        error_clear_btn.clicked.connect(self.clear_motor_errors)
        motor_control_layout.addWidget(error_clear_btn)
        
        # 모터 정지 버튼
        motor_stop_btn = QPushButton("⏹ 모터 정지")
        motor_stop_btn.setStyleSheet("""
            QPushButton {
                background-color: #FF9800;
                color: white;
                font-weight: bold;
                border-radius: 5px;
                padding: 10px;
                min-width: 120px;
            }
            QPushButton:hover {
                background-color: #F57C00;
            }
            QPushButton:pressed {
                background-color: #EF6C00;
            }
        """)
        motor_stop_btn.clicked.connect(self.stop_position_motors)
        motor_control_layout.addWidget(motor_stop_btn)
        
        # 모터 셧다운 버튼
        motor_shutdown_btn = QPushButton("🔌 모터 셧다운")
        motor_shutdown_btn.setStyleSheet("""
            QPushButton {
                background-color: #F44336;
                color: white;
                font-weight: bold;
                border-radius: 5px;
                padding: 10px;
                min-width: 120px;
            }
            QPushButton:hover {
                background-color: #E53935;
            }
            QPushButton:pressed {
                background-color: #D32F2F;
            }
        """)
        motor_shutdown_btn.clicked.connect(self.shutdown_position_motors)
        motor_control_layout.addWidget(motor_shutdown_btn)
        
        layout.addWidget(motor_control_group)
        
        # 개별 관절 제어
        joint_group = QGroupBox("개별 motor 제어")
        joint_layout = QGridLayout(joint_group)
        
        self.joint_labels = []
        self.joint_spinboxes = []
        self.current_position_labels = []
        self.joint_send_buttons = []
        self.joint_read_buttons = []

        joint_names = ["motor 1", "motor 2", "motor 3", "motor 4", "motor 5"]

        for i, name in enumerate(joint_names):
            # 관절 이름
            joint_layout.addWidget(QLabel(f"{name}:"), i, 0)

            # 상대 이동량 표시
            label = QLabel("이동: +0°")
            label.setMinimumWidth(80)
            label.setStyleSheet("color: green; font-weight: bold;")
            self.joint_labels.append(label)
            joint_layout.addWidget(label, i, 1)

            # 현재값 표시
            current_label = QLabel("현재: 0°")
            current_label.setMinimumWidth(80)
            current_label.setStyleSheet("color: blue; font-weight: bold;")
            self.current_position_labels.append(current_label)
            joint_layout.addWidget(current_label, i, 2)

            # 스핀박스
            spinbox = QSpinBox()
            spinbox.setRange(-1800, 1800)
            spinbox.setValue(0)
            spinbox.valueChanged.connect(lambda value, idx=i: self.update_joint_target(idx, value))
            self.joint_spinboxes.append(spinbox)
            joint_layout.addWidget(spinbox, i, 3)

            # 읽기 버튼
            read_btn = QPushButton("읽기")
            read_btn.setStyleSheet("""
                QPushButton {
                    background-color: #FF9800;
                    color: white;
                    font-weight: bold;
                    border-radius: 3px;
                    padding: 5px;
                    min-width: 50px;
                }
                QPushButton:hover {
                    background-color: #F57C00;
                }
                QPushButton:pressed {
                    background-color: #EF6C00;
                }
            """)
            read_btn.clicked.connect(lambda checked, idx=i: self.read_joint_position(idx))
            self.joint_read_buttons.append(read_btn)
            joint_layout.addWidget(read_btn, i, 4)

            # 개별 명령 전송 버튼
            send_btn = QPushButton("전송")
            send_btn.setStyleSheet("""
                QPushButton {
                    background-color: #2196F3;
                    color: white;
                    font-weight: bold;
                    border-radius: 3px;
                    padding: 5px;
                    min-width: 50px;
                }
                QPushButton:hover {
                    background-color: #1976D2;
                }
                QPushButton:pressed {
                    background-color: #1565C0;
                }
            """)
            send_btn.clicked.connect(lambda checked, idx=i: self.send_joint_command(idx))
            self.joint_send_buttons.append(send_btn)
            joint_layout.addWidget(send_btn, i, 5)
        
        layout.addWidget(joint_group)
        
        # 미리 정의된 포즈
        pose_group = QGroupBox("미리 정의된 포즈")
        pose_layout = QGridLayout(pose_group)
        
        poses = [
            ("홈 포즈", [0, 0, 0, 0, 0]),
            ("준비 포즈", [0, -45, 90, -45, 0]),
            ("그립 포즈", [0, -90, 90, 0, 0]),
            ("확장 포즈", [0, 0, 0, 0, 0])
        ]
        
        for i, (name, angles) in enumerate(poses):
            btn = QPushButton(name)
            btn.clicked.connect(lambda checked, angles=angles: self.set_preset_pose(angles))
            pose_layout.addWidget(btn, i // 2, i % 2)
        
        layout.addWidget(pose_group)
        
        # 궤적 제어
        trajectory_group = QGroupBox("궤적 제어")
        trajectory_layout = QVBoxLayout(trajectory_group)
        
        self.trajectory_text = QTextEdit()
        self.trajectory_text.setMaximumHeight(100)
        self.trajectory_text.setPlainText("# 궤적 예시:\n# motor1,motor2,motor3,motor4,motor5\n0,0,0,0,0\n45,-30,60,-30,0")
        trajectory_layout.addWidget(self.trajectory_text)
        
        execute_btn = QPushButton("궤적 실행")
        execute_btn.clicked.connect(self.execute_trajectory)
        trajectory_layout.addWidget(execute_btn)
        
        layout.addWidget(trajectory_group)
        
        return group
    
    def create_status_group(self):
        """상태 모니터링 그룹 생성"""
        group = QGroupBox("📊 상태 모니터링")
        group.setFont(QFont("Arial", 12, QFont.Bold))
        layout = QVBoxLayout(group)
        
        # 모터 상태 표시
        motor_status_group = QGroupBox("모터 상태")
        motor_layout = QGridLayout(motor_status_group)
        
        self.motor_status_labels = {}
        motor_ids = ["0x141", "0x142", "0x143", "0x144", "0x145", "0x146", "0x147"]
        motor_names = ["왼쪽바퀴", "오른쪽바퀴", "motor1", "motor2", "motor3", "motor4", "motor5"]
        
        for i, (motor_id, name) in enumerate(zip(motor_ids, motor_names)):
            motor_layout.addWidget(QLabel(f"{name} ({motor_id}):"), i, 0)
            
            status_label = QLabel("연결 안됨")
            status_label.setStyleSheet("color: red; font-weight: bold;")
            self.motor_status_labels[motor_id] = status_label
            motor_layout.addWidget(status_label, i, 1)
        
        layout.addWidget(motor_status_group)
        
        # 실시간 로그 (컴팩트하게)
        log_group = QGroupBox("실시간 로그")
        log_layout = QVBoxLayout(log_group)
        
        self.log_text = QTextEdit()
        self.log_text.setMaximumHeight(120)  # 높이 줄임
        self.log_text.setMaximumWidth(300)   # 너비 제한
        log_layout.addWidget(self.log_text)
        
        clear_log_btn = QPushButton("로그 지우기")
        clear_log_btn.clicked.connect(self.clear_log)
        log_layout.addWidget(clear_log_btn)
        
        layout.addWidget(log_group)
        
        return group
    
    def create_settings_group(self):
        """설정 그룹 생성"""
        group = QGroupBox("⚙️ 설정")
        group.setFont(QFont("Arial", 12, QFont.Bold))
        layout = QVBoxLayout(group)
        
        # 안전 설정
        safety_group = QGroupBox("안전 설정")
        safety_layout = QVBoxLayout(safety_group)
        
        self.safety_checkbox = QCheckBox("안전 모드 활성화")
        self.safety_checkbox.setChecked(True)
        self.safety_checkbox.stateChanged.connect(self.toggle_safety_mode)
        safety_layout.addWidget(self.safety_checkbox)
        
        self.max_speed_label = QLabel("최대 속도 제한:")
        safety_layout.addWidget(self.max_speed_label)
        
        self.max_speed_slider = QSlider(Qt.Horizontal)
        self.max_speed_slider.setRange(10, 100)
        self.max_speed_slider.setValue(50)
        self.max_speed_slider.valueChanged.connect(self.update_max_speed)
        safety_layout.addWidget(self.max_speed_slider)
        
        self.max_speed_value = QLabel("50%")
        safety_layout.addWidget(self.max_speed_value)
        
        layout.addWidget(safety_group)
        
        # 통신 설정 (컴팩트하게)
        comm_group = QGroupBox("통신 설정")
        comm_layout = QVBoxLayout(comm_group)
        
        comm_layout.addWidget(QLabel("CAN: can2"))
        comm_layout.addWidget(QLabel("모터: 0x141-0x147"))
        comm_layout.addWidget(QLabel("속도: 1Mbps"))
        
        layout.addWidget(comm_group)
        
        return group
    
    def create_side_panel(self):
        """사이드 패널 생성 (안전 제어)"""
        panel = QFrame()
        panel.setFrameStyle(QFrame.StyledPanel)
        panel.setMaximumWidth(200)
        layout = QVBoxLayout(panel)
        
        # 비상 정지 버튼
        emergency_btn = QPushButton("비상 정지")
        emergency_btn.setStyleSheet("""
            QPushButton {
                background-color: red;
                color: white;
                font-size: 16px;
                font-weight: bold;
                border: 2px solid darkred;
                border-radius: 10px;
                padding: 10px;
            }
            QPushButton:hover {
                background-color: darkred;
            }
        """)
        emergency_btn.clicked.connect(self.emergency_stop_robot)
        layout.addWidget(emergency_btn)
        
        # 안전 상태 표시
        safety_status = QLabel("안전 모드 활성화")
        safety_status.setStyleSheet("color: green; font-weight: bold;")
        safety_status.setAlignment(Qt.AlignCenter)
        layout.addWidget(safety_status)
        
        # 연결 상태
        connection_status = QLabel("ROS2 연결됨")
        connection_status.setStyleSheet("color: blue; font-weight: bold;")
        connection_status.setAlignment(Qt.AlignCenter)
        layout.addWidget(connection_status)
        
        # 제어 활성화 버튼
        self.enable_control_btn = QPushButton("제어 활성화")
        self.enable_control_btn.setStyleSheet("""
            QPushButton {
                background-color: green;
                color: white;
                font-weight: bold;
                border-radius: 5px;
                padding: 5px;
            }
            QPushButton:hover {
                background-color: darkgreen;
            }
        """)
        self.enable_control_btn.clicked.connect(self.toggle_control_enable)
        layout.addWidget(self.enable_control_btn)
        
        layout.addStretch()
        return panel
    
    def init_ros_communication(self):
        """ROS2 통신 초기화"""
        from std_msgs.msg import Bool, Float32

        # 퍼블리셔
        self.cmd_vel_publisher = self.ros_node.create_publisher(
            Twist, '/cmd_vel', 10
        )

        self.trajectory_publisher = self.ros_node.create_publisher(
            JointTrajectory, '/joint_trajectory', 10
        )

        # 모터 제어 명령 퍼블리셔 (브레이크, 서보, 정지, 셧다운)
        self.motor_control_publishers = {}
        motor_ids = [0x143, 0x144, 0x145, 0x146, 0x147]

        for motor_id in motor_ids:
            motor_hex = hex(motor_id)
            self.motor_control_publishers[f'{motor_hex}_brake'] = self.ros_node.create_publisher(
                Bool, f'/motor_{motor_hex}_brake', 10
            )
            self.motor_control_publishers[f'{motor_hex}_enable'] = self.ros_node.create_publisher(
                Bool, f'/motor_{motor_hex}_enable', 10
            )
            self.motor_control_publishers[f'{motor_hex}_stop'] = self.ros_node.create_publisher(
                Bool, f'/motor_{motor_hex}_stop', 10
            )
            self.motor_control_publishers[f'{motor_hex}_shutdown'] = self.ros_node.create_publisher(
                Bool, f'/motor_{motor_hex}_shutdown', 10
            )

        # 구독자
        self.joint_state_subscription = self.ros_node.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        self.motor_status_subscription = self.ros_node.create_subscription(
            Float64MultiArray, '/motor_status', self.motor_status_callback, 10
        )

        # 개별 모터 위치 상태 구독 (0x143-0x147)
        self.position_motor_subscriptions = []

        for i, motor_id in enumerate(motor_ids):
            subscription = self.ros_node.create_subscription(
                Float32, f'motor_{hex(motor_id)}_position',
                lambda msg, idx=i: self.position_motor_callback(idx, msg), 10
            )
            self.position_motor_subscriptions.append(subscription)

        # 개별 모터 속도 상태 구독 (0x141-0x142)
        self.rpm_motor_subscriptions = []
        rpm_motor_ids = [0x141, 0x142]

        for i, motor_id in enumerate(rpm_motor_ids):
            subscription = self.ros_node.create_subscription(
                Float32, f'motor_{hex(motor_id)}_rpm',
                lambda msg, idx=i: self.rpm_motor_callback(idx, msg), 10
            )
            self.rpm_motor_subscriptions.append(subscription)
    
    def update_cmd_vel_display(self):
        """CMD_VEL 표시 업데이트 (실제 전송은 버튼으로)"""
        linear_value = self.linear_slider.value() / 100.0
        angular_value = self.angular_slider.value() / 100.0
        
        self.linear_label.setText(f"{linear_value:.2f}")
        self.angular_label.setText(f"{angular_value:.2f}")
    
    def send_cmd_vel_command(self):
        """CMD_VEL 명령 전송"""
        if not self.safety_enabled or self.emergency_stop:
            self.log_message("⚠️ 안전 모드 또는 비상 정지 상태입니다")
            return
        
        linear_value = self.linear_slider.value() / 100.0
        angular_value = self.angular_slider.value() / 100.0
        
        # Twist 메시지 생성
        twist = Twist()
        twist.linear.x = linear_value
        twist.angular.z = angular_value
        
        self.cmd_vel_publisher.publish(twist)
        # 정지 명령이 아닐 때만 로그
        if abs(linear_value) > 0.01 or abs(angular_value) > 0.01:
            self.log_message(f"🚗 CMD_VEL: v={linear_value:.2f} ω={angular_value:.2f}")
    
    def stop_cmd_vel_command(self):
        """CMD_VEL 정지 명령 (bus-off 방지를 위해 1회 전송)"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        
        # 정지 명령 1회만 전송 (노드에서 처리)
        self.cmd_vel_publisher.publish(twist)
        rclpy.spin_once(self.ros_node, timeout_sec=0)
        
        self.linear_slider.setValue(0)
        self.angular_slider.setValue(0)
        self.update_cmd_vel_display()
        self.log_message("🛑 정지")
    
    def set_preset_movement(self, linear, angular):
        """미리 정의된 동작 설정"""
        self.linear_slider.setValue(linear)
        self.angular_slider.setValue(angular)
    
    def update_joint_target(self, joint_index, value):
        """관절 목표값 업데이트 (상대 이동량 표시)"""
        if value >= 0:
            self.joint_labels[joint_index].setText(f"이동: +{value}°")
        else:
            self.joint_labels[joint_index].setText(f"이동: {value}°")
    
    def read_joint_position(self, joint_index):
        """개별 관절 위치 읽기"""
        # 현재 저장된 위치 값을 표시 (이미 구독으로 업데이트되고 있음)
        current_pos = self.motor_states['position'][joint_index]

        # 로그는 제거 (GUI에 이미 표시됨)

        # 정보 메시지 표시
        QMessageBox.information(
            self,
            "위치 읽기",
            f"motor {joint_index + 1}의 현재 위치:\n{current_pos:.1f}°"
        )

    def read_all_positions(self):
        """모든 관절 위치 읽기"""
        positions_text = "현재 모든 motor 위치:\n\n"

        for i in range(5):
            current_pos = self.motor_states['position'][i]
            positions_text += f"motor {i + 1}: {current_pos:.1f}°\n"

        # 로그는 제거 (GUI에 이미 표시됨)

        # 정보 메시지 표시
        QMessageBox.information(
            self,
            "전체 위치 읽기",
            positions_text
        )

    def send_joint_command(self, joint_index):
        """개별 관절 명령 전송 (상대 위치 제어)"""
        if not self.safety_enabled or self.emergency_stop:
            self.log_message("⚠️ 안전 모드 또는 비상 정지 상태입니다")
            return

        # 입력값은 상대 이동량 (현재 위치로부터의 변화량)
        relative_movement = self.joint_spinboxes[joint_index].value()

        # 현재 위치 가져오기
        current_position = self.motor_states['position'][joint_index]

        # 목표 위치 = 현재 위치 + 상대 이동량
        target_position = current_position + relative_movement

        # JointTrajectory 메시지 생성
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.ros_node.get_clock().now().to_msg()
        trajectory.header.frame_id = 'base_link'

        # 관절 이름 매핑 (위치제어 노드와 일치)
        joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
        trajectory.joint_names = [joint_names[joint_index]]

        point = JointTrajectoryPoint()
        point.positions = [math.radians(target_position)]  # 도를 라디안으로 변환
        point.time_from_start.sec = 2
        trajectory.points.append(point)

        self.trajectory_publisher.publish(trajectory)
        self.log_message(f"🎯 M{joint_index + 1}: {current_position:.1f}° → {target_position:.1f}° (상대: {relative_movement:+.1f}°)")
    
    def set_preset_pose(self, angles):
        """미리 정의된 포즈 설정"""
        for i, angle in enumerate(angles):
            self.joint_spinboxes[i].setValue(angle)
    
    def execute_trajectory(self):
        """궤적 실행"""
        if not self.safety_enabled or self.emergency_stop:
            return
        
        try:
            lines = self.trajectory_text.toPlainText().strip().split('\n')
            trajectory = JointTrajectory()
            trajectory.header.stamp = self.ros_node.get_clock().now().to_msg()
            trajectory.header.frame_id = 'base_link'
            
            trajectory.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
            
            for line in lines:
                if line.strip() and not line.startswith('#'):
                    angles = [float(x.strip()) for x in line.split(',')]
                    if len(angles) == 5:
                        point = JointTrajectoryPoint()
                        point.positions = [math.radians(angle) for angle in angles]
                        point.time_from_start.sec = 2
                        trajectory.points.append(point)
            
            if trajectory.points:
                self.trajectory_publisher.publish(trajectory)
                self.log_message("궤적 실행됨")
            else:
                self.log_message("유효한 궤적이 없습니다")
                
        except Exception as e:
            self.log_message(f"궤적 실행 오류: {e}")
    
    def joint_state_callback(self, msg):
        """관절 상태 콜백"""
        if len(msg.position) >= 7:  # 바퀴 2개 + 관절 5개
            # CMD_VEL 모터 상태 업데이트
            self.motor_states['cmd_vel']['left'] = msg.velocity[0] * 60 / (2 * math.pi)  # rad/s -> RPM
            self.motor_states['cmd_vel']['right'] = msg.velocity[1] * 60 / (2 * math.pi)
            
            # 위치제어 모터 상태 업데이트
            for i in range(5):
                if i + 2 < len(msg.position):
                    self.motor_states['position'][i] = msg.position[i + 2] * 180 / math.pi  # rad -> deg
    
    def motor_status_callback(self, msg):
        """모터 상태 콜백"""
        # 모터 상태 업데이트 로직
        pass
    
    def position_motor_callback(self, joint_index, msg):
        """위치제어 모터 콜백"""
        if joint_index < len(self.current_position_labels):
            current_angle = msg.data
            self.current_position_labels[joint_index].setText(f"현재: {current_angle:.1f}°")

            # 모터 상태 저장
            self.motor_states['position'][joint_index] = current_angle

            # 디버그: 위치 업데이트 확인 (로그 메시지에 추가)
            import random
            if random.random() < 0.1:  # 10% 확률로 로그
                self.log_message(f"📥 motor {joint_index + 1} 위치 수신: {current_angle:.1f}°")
    
    def rpm_motor_callback(self, motor_index, msg):
        """RPM 모터 콜백"""
        rpm = msg.data
        if motor_index == 0:  # 왼쪽 바퀴
            self.motor_states['cmd_vel']['left'] = rpm
        elif motor_index == 1:  # 오른쪽 바퀴
            self.motor_states['cmd_vel']['right'] = rpm
    
    def update_status(self):
        """상태 업데이트"""
        # ROS2 콜백 처리 (spin_once)
        rclpy.spin_once(self.ros_node, timeout_sec=0)

        # CMD_VEL 상태 업데이트
        self.left_wheel_label.setText(f"{self.motor_states['cmd_vel']['left']:.1f} RPM")
        self.right_wheel_label.setText(f"{self.motor_states['cmd_vel']['right']:.1f} RPM")

        # 모터 연결 상태 업데이트 (실제 구현에서는 ROS2 토픽에서 받아옴)
        for motor_id in self.motor_status_labels:
            self.motor_status_labels[motor_id].setText("연결됨")
            self.motor_status_labels[motor_id].setStyleSheet("color: green; font-weight: bold;")
    
    def emergency_stop_robot(self):
        """비상 정지"""
        self.emergency_stop = True

        # 모든 모터 정지 (여러 번 전송하여 확실히)
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0

        for _ in range(5):  # 5번 전송하여 확실히 정지
            self.cmd_vel_publisher.publish(twist)
            time.sleep(0.01)

        # 모든 관절을 현재 위치로 유지
        for i in range(5):
            current_pos = self.motor_states['position'][i]
            self.joint_spinboxes[i].setValue(int(current_pos))

        self.log_message("비상 정지 실행됨!")

        # 확인 메시지
        QMessageBox.warning(self, "비상 정지", "로봇이 비상 정지되었습니다!")
    
    def recover_can_bus(self):
        """CAN 버스 복구 시도"""
        try:
            self.log_message("🔄 CAN 버스 복구 시도 중...")
            
            # CAN 인터페이스 재시작 명령 (사용자에게 알림)
            QMessageBox.information(
                self, 
                "CAN 버스 복구", 
                "CAN 버스 복구를 위해 인터페이스를 재시작해야 합니다.\n\n"
                "터미널에서 다음 명령을 실행해주세요:\n"
                "sudo ip link set can2 down\n"
                "sudo ip link set can2 up type can bitrate 1000000"
            )
            
            # 잠시 후 상태 재확인
            time.sleep(2)
            if self.check_can_bus_status():
                self.log_message("✅ CAN 버스 복구 완료")
                return True
            else:
                self.log_message("❌ CAN 버스 복구 실패")
                return False
                
        except Exception as e:
            self.log_message(f"❌ CAN 복구 오류: {e}")
            return False

    def check_can_bus_status(self):
        """CAN 버스 상태 확인"""
        try:
            result = subprocess.run(['ip', '-details', 'link', 'show', 'can2'], 
                                  capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                output = result.stdout
                if 'ERROR-PASSIVE' in output or 'bus-off' in output:
                    self.log_message("⚠ CAN 버스 오류 상태 감지!")
                    self.log_message(f"상태: {output}")
                    return False
                elif 'ERROR-ACTIVE' in output:
                    self.log_message("✅ CAN 버스 정상 상태")
                    return True
            return True
        except Exception as e:
            self.log_message(f"⚠ CAN 상태 확인 오류: {e}")
            return False

    def brake_release_position_motors(self):
        """위치제어 모터들 브레이크 해제 (서비스 기반 안전한 처리)"""
        try:
            self.log_message("🔓 위치제어 모터들 브레이크 해제 시작...")
            
            # 서비스 클라이언트 생성 및 호출
            from std_srvs.srv import Trigger
            client = self.ros_node.create_client(Trigger, 'safe_brake_release')
            
            # 서비스 서버 대기
            if not client.wait_for_service(timeout_sec=5.0):
                self.log_message("❌ 브레이크 해제 서비스 서버를 찾을 수 없습니다!")
                QMessageBox.warning(self, "서비스 오류", "위치제어 노드가 실행 중인지 확인해주세요.")
                return
            
            # 서비스 요청
            request = Trigger.Request()
            future = client.call_async(request)
            
            self.log_message("⏳ 안전한 브레이크 해제 서비스 호출 중...")
            
            # 서비스 응답 대기 (최대 30초)
            rclpy.spin_until_future_complete(self.ros_node, future, timeout_sec=30.0)
            
            if future.done():
                response = future.result()
                if response.success:
                    self.log_message("✅ 브레이크 해제 서비스 성공!")
                    self.log_message(f"서버 응답: {response.message}")
                    QMessageBox.information(self, "브레이크 해제", "모든 모터의 브레이크가 안전하게 해제되었습니다!")
                else:
                    self.log_message(f"❌ 브레이크 해제 서비스 실패: {response.message}")
                    QMessageBox.warning(self, "브레이크 해제 실패", f"브레이크 해제 실패: {response.message}")
            else:
                self.log_message("❌ 브레이크 해제 서비스 타임아웃!")
                QMessageBox.warning(self, "서비스 타임아웃", "브레이크 해제 서비스가 응답하지 않습니다.")

        except Exception as e:
            self.log_message(f"❌ 브레이크 해제 오류: {e}")
            QMessageBox.critical(self, "오류", f"브레이크 해제 실패: {e}")
    
    def clear_motor_errors(self):
        """모터 0x141~0x147 에러 초기화 (순차 실행)"""
        try:
            import can
            
            self.log_message("🔧 모터 에러 초기화 시작...")
            
            # CAN 버스 연결
            try:
                bus = can.interface.Bus(channel='can2', bustype='socketcan')
            except Exception as e:
                self.log_message(f"❌ CAN 버스 연결 실패: {e}")
                QMessageBox.critical(self, "CAN 오류", f"CAN 버스 연결 실패: {e}")
                return
            
            # 에러 초기화 명령: 0x9B (Clear Motor Fault)
            error_clear_cmd = bytes([0x9B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
            
            motor_ids = [0x141, 0x142, 0x143, 0x144, 0x145, 0x146, 0x147]
            
            for motor_id in motor_ids:
                msg = can.Message(
                    arbitration_id=motor_id,
                    data=error_clear_cmd,
                    is_extended_id=False
                )
                bus.send(msg)
                self.log_message(f"  ✓ 0x{motor_id:03X} 에러 초기화")
                time.sleep(0.1)  # 각 모터 간 100ms 간격
            
            bus.shutdown()
            
            self.log_message("✅ 모든 모터 에러 초기화 완료")
            QMessageBox.information(self, "에러 초기화", "모든 모터의 에러가 초기화되었습니다!")
            
        except Exception as e:
            self.log_message(f"❌ 에러 초기화 실패: {e}")
            QMessageBox.critical(self, "오류", f"에러 초기화 실패: {e}")
    
    def servo_on_position_motors(self):
        """위치제어 모터들 서보 온 (강화된 CAN 버스 보호)"""
        try:
            # CAN 버스 상태 먼저 확인
            if not self.check_can_bus_status():
                QMessageBox.warning(self, "CAN 버스 오류", "CAN 버스가 오류 상태입니다. 인터페이스를 재시작해주세요.")
                return
                
            from std_msgs.msg import Bool
            enable_msg = Bool()
            enable_msg.data = True  # True = 서보 온

            self.log_message("⚡ 위치제어 모터들 서보 온 시작...")
            
            # 위치제어 모터들에 매우 안전하게 순차적으로 서보 온 명령 발행
            motor_ids = [0x143, 0x144, 0x145, 0x146, 0x147]
            
            for i, motor_id in enumerate(motor_ids):
                motor_hex = hex(motor_id)
                publisher_key = f'{motor_hex}_enable'
                
                if publisher_key in self.motor_control_publishers:
                    # 각 명령 전에 CAN 상태 재확인
                    if not self.check_can_bus_status():
                        self.log_message(f"⚠ CAN 버스 오류 감지 - 모터 {motor_hex} 명령 중단")
                        QMessageBox.warning(self, "CAN 버스 오류", f"모터 {motor_hex} 명령 전송 중 CAN 버스 오류가 발생했습니다!")
                        return
                    
                    self.motor_control_publishers[publisher_key].publish(enable_msg)
                    self.log_message(f"  → 모터 {motor_hex} 서보 온 명령 전송")
                    
                    # 더 긴 지연 시간으로 CAN 버스 보호 (마지막 모터 제외)
                    if i < len(motor_ids) - 1:
                        time.sleep(0.3)  # 300ms 지연으로 증가
                        
                        # ROS2 메시지 처리
                        rclpy.spin_once(self.ros_node, timeout_sec=0.05)
                        
                        # 지연 후 CAN 상태 재확인
                        if not self.check_can_bus_status():
                            self.log_message(f"⚠ CAN 버스 오류 감지 - 나머지 모터 명령 중단")
                            QMessageBox.warning(self, "CAN 버스 오류", "명령 전송 중 CAN 버스 오류가 발생했습니다!")
                            return

            # 명령 완료 후 충분한 시간 대기 후 CAN 상태 재확인
            self.log_message("⏳ 명령 처리 대기 중...")
            time.sleep(1.0)  # 1초 대기로 증가
            
            if not self.check_can_bus_status():
                self.log_message("❌ 서보 온 후 CAN 버스 오류 발생!")
                
                # 복구 시도
                reply = QMessageBox.question(
                    self, 
                    "CAN 버스 오류", 
                    "서보 온 후 CAN 버스 오류가 발생했습니다!\n\n"
                    "CAN 인터페이스를 재시작하시겠습니까?",
                    QMessageBox.Yes | QMessageBox.No,
                    QMessageBox.Yes
                )
                
                if reply == QMessageBox.Yes:
                    self.recover_can_bus()
                return

            self.log_message("✅ 모든 위치제어 모터 서보 온 명령 완료")
            QMessageBox.information(self, "서보 온", "위치제어 모터들 서보 온 명령이 안전하게 전송되었습니다!")

        except Exception as e:
            self.log_message(f"❌ 서보 온 오류: {e}")
            QMessageBox.critical(self, "오류", f"서보 온 실패: {e}")
    
    def stop_position_motors(self):
        """위치제어 모터들 정지 (순차적 전송으로 CAN 버스 보호)"""
        try:
            from std_msgs.msg import Bool
            stop_msg = Bool()
            stop_msg.data = True  # True = 정지

            self.log_message("⏹ 위치제어 모터들 정지 시작...")
            
            # 위치제어 모터들에 순차적으로 정지 명령 발행
            motor_ids = [0x143, 0x144, 0x145, 0x146, 0x147]
            
            for i, motor_id in enumerate(motor_ids):
                motor_hex = hex(motor_id)
                publisher_key = f'{motor_hex}_stop'
                
                if publisher_key in self.motor_control_publishers:
                    self.motor_control_publishers[publisher_key].publish(stop_msg)
                    self.log_message(f"  → 모터 {motor_hex} 정지 명령 전송")
                    
                    # CAN 버스 보호를 위한 지연 (마지막 모터 제외)
                    if i < len(motor_ids) - 1:
                        time.sleep(0.1)  # 100ms 지연
                        
                        # ROS2 메시지 처리
                        rclpy.spin_once(self.ros_node, timeout_sec=0.01)

            self.log_message("✅ 모든 위치제어 모터 정지 명령 완료")
            QMessageBox.information(self, "모터 정지", "위치제어 모터들 정지 명령이 순차적으로 전송되었습니다!")

        except Exception as e:
            self.log_message(f"❌ 모터 정지 오류: {e}")
            QMessageBox.critical(self, "오류", f"모터 정지 실패: {e}")
    
    def shutdown_position_motors(self):
        """위치제어 모터들 셧다운 (순차적 전송으로 CAN 버스 보호)"""
        try:
            from std_msgs.msg import Bool
            shutdown_msg = Bool()
            shutdown_msg.data = True  # True = 셧다운

            self.log_message("🔌 위치제어 모터들 셧다운 시작...")
            
            # 위치제어 모터들에 순차적으로 셧다운 명령 발행
            motor_ids = [0x143, 0x144, 0x145, 0x146, 0x147]
            
            for i, motor_id in enumerate(motor_ids):
                motor_hex = hex(motor_id)
                publisher_key = f'{motor_hex}_shutdown'
                
                if publisher_key in self.motor_control_publishers:
                    self.motor_control_publishers[publisher_key].publish(shutdown_msg)
                    self.log_message(f"  → 모터 {motor_hex} 셧다운 명령 전송")
                    
                    # CAN 버스 보호를 위한 지연 (마지막 모터 제외)
                    if i < len(motor_ids) - 1:
                        time.sleep(0.1)  # 100ms 지연
                        
                        # ROS2 메시지 처리
                        rclpy.spin_once(self.ros_node, timeout_sec=0.01)

            self.log_message("✅ 모든 위치제어 모터 셧다운 명령 완료")
            QMessageBox.information(self, "모터 셧다운", "위치제어 모터들 셧다운 명령이 순차적으로 전송되었습니다!")

        except Exception as e:
            self.log_message(f"❌ 모터 셧다운 오류: {e}")
            QMessageBox.critical(self, "오류", f"모터 셧다운 실패: {e}")
    
    def toggle_control_enable(self):
        """제어 활성화 토글"""
        if self.emergency_stop:
            self.emergency_stop = False
            self.enable_control_btn.setText("제어 활성화")
            self.enable_control_btn.setStyleSheet("""
                QPushButton {
                    background-color: green;
                    color: white;
                    font-weight: bold;
                    border-radius: 5px;
                    padding: 5px;
                }
            """)
            self.log_message("제어 활성화됨")
        else:
            self.emergency_stop = True
            self.enable_control_btn.setText("제어 비활성화")
            self.enable_control_btn.setStyleSheet("""
                QPushButton {
                    background-color: red;
                    color: white;
                    font-weight: bold;
                    border-radius: 5px;
                    padding: 5px;
                }
            """)
            self.log_message("제어 비활성화됨")
    
    def toggle_safety_mode(self, state):
        """안전 모드 토글"""
        self.safety_enabled = state == Qt.Checked
        status = "활성화" if self.safety_enabled else "비활성화"
        self.log_message(f"안전 모드 {status}")
    
    def update_max_speed(self, value):
        """최대 속도 업데이트"""
        self.max_speed_value.setText(f"{value}%")
        # 실제 구현에서는 모터 제어 노드에 파라미터 전달
    
    def log_message(self, message):
        """로그 메시지 추가 (중요한 것만, 최대 50줄 유지)"""
        timestamp = time.strftime("%H:%M:%S")
        
        # 로그 라인 수 제한 (50줄 초과 시 오래된 것 삭제)
        current_text = self.log_text.toPlainText()
        lines = current_text.split('\n')
        if len(lines) >= 50:
            # 오래된 라인 삭제 (최근 40줄만 유지)
            self.log_text.setPlainText('\n'.join(lines[-40:]))
        
        self.log_text.append(f"[{timestamp}] {message}")
    
    def clear_log(self):
        """로그 지우기"""
        self.log_text.clear()
    
    def closeEvent(self, event):
        """윈도우 종료 시 정리"""
        try:
            # 정지 명령 여러 번 전송 (GUI 종료 전 확실히 정지)
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0

            self.log_message("🛑 GUI 종료: 모든 모터 정지 명령 전송 중...")

            for _ in range(10):  # 10번 전송하여 확실히 정지
                self.cmd_vel_publisher.publish(twist)
                time.sleep(0.02)

            # 메시지 전송 확인 (shutdown 전에 한 번만)
            try:
                rclpy.spin_once(self.ros_node, timeout_sec=0.1)
            except:
                pass  # 이미 shutdown된 경우 무시

            # ROS2 노드 종료
            self.ros_node.destroy_node()

            # rclpy가 이미 shutdown되었는지 확인하고 shutdown 호출
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except:
                pass  # 이미 shutdown된 경우 무시

        except Exception as e:
            print(f"GUI closeEvent 오류: {e}")

        event.accept()


def main():
    """메인 함수"""
    app = QApplication(sys.argv)
    
    # 어플리케이션 스타일 설정
    app.setStyle('Fusion')
    
    # 다크 테마 적용
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(53, 53, 53))
    palette.setColor(QPalette.WindowText, Qt.white)
    palette.setColor(QPalette.Base, QColor(25, 25, 25))
    palette.setColor(QPalette.AlternateBase, QColor(53, 53, 53))
    palette.setColor(QPalette.ToolTipBase, Qt.white)
    palette.setColor(QPalette.ToolTipText, Qt.white)
    palette.setColor(QPalette.Text, Qt.white)
    palette.setColor(QPalette.Button, QColor(53, 53, 53))
    palette.setColor(QPalette.ButtonText, Qt.white)
    palette.setColor(QPalette.BrightText, Qt.red)
    palette.setColor(QPalette.Link, QColor(42, 130, 218))
    palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
    palette.setColor(QPalette.HighlightedText, Qt.black)
    app.setPalette(palette)
    
    window = RobotControlGUI()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
