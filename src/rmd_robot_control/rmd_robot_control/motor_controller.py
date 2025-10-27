#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import threading
from typing import Optional, Dict, Any, List
import logging
from enum import IntEnum

from .rmd_x4_protocol import RMDX4Protocol, ControlMode, CommandType
from .can_manager import CANManager

class MotorState(IntEnum):
    """모터 상태"""
    IDLE = 0
    RUNNING = 1
    ERROR = 2
    EMERGENCY_STOP = 3
    CALIBRATING = 4

class MotorController:
    """
    RMD-X4 모터 제어 클래스
    6가지 제어 모드와 듀얼 엔코더 지원
    """
    
    def __init__(self, can_manager: CANManager):
        self.can_manager = can_manager
        self.protocol = RMDX4Protocol()
        self.logger = logging.getLogger(__name__)
        
        # 모터 상태
        self.current_state = MotorState.IDLE
        self.control_mode = ControlMode.POSITION_CONTROL
        self.emergency_stop_active = False
        
        # 제어 파라미터
        self.target_position = 0.0
        self.target_velocity = 0.0
        self.target_current = 0.0
        self.target_torque = 0.0
        
        # 현재 상태
        self.current_position = 0.0
        self.current_velocity = 0.0
        self.current_current = 0.0
        self.current_torque = 0.0
        
        # 엔코더 데이터
        self.encoder1_data = 0
        self.encoder2_data = 0
        self.encoder1_angle = 0.0
        self.encoder2_angle = 0.0
        
        # 안전 제한
        self.max_velocity = 10.0  # rad/s
        self.max_current = 10.0   # A
        self.max_torque = 10.0    # Nm
        
        # 제어 루프
        self.control_thread = None
        self.control_frequency = 100  # Hz
        self.running = False
        
        # PID 제어 파라미터 (기본값)
        self.pid_params = {
            'position': {'kp': 100.0, 'ki': 0.0, 'kd': 10.0},
            'velocity': {'kp': 50.0, 'ki': 0.0, 'kd': 5.0},
            'current': {'kp': 10.0, 'ki': 0.0, 'kd': 1.0}
        }
        
        # 제어 히스토리
        self.control_history = []
        self.max_history_size = 1000
    
    def initialize(self) -> bool:
        """모터 제어기 초기화"""
        try:
            # CAN 매니저 초기화
            if not self.can_manager.initialize():
                self.logger.error("CAN 매니저 초기화 실패")
                return False
            
            # 모터 상태 확인
            if not self._check_motor_status():
                self.logger.error("모터 상태 확인 실패")
                return False
            
            # 제어 루프 시작
            self.running = True
            self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
            self.control_thread.start()
            
            self.logger.info("모터 제어기 초기화 완료")
            return True
            
        except Exception as e:
            self.logger.error(f"모터 제어기 초기화 오류: {e}")
            return False
    
    def _check_motor_status(self) -> bool:
        """모터 상태 확인"""
        try:
            # 모터 상태 읽기 명령 전송
            status_cmd = self.protocol.create_system_command(CommandType.READ_MOTOR_STATUS)
            if not self.can_manager.send_message(self.protocol.MOTOR_CMD_ID, status_cmd):
                return False
            
            # 응답 대기
            time.sleep(0.1)
            response = self.can_manager.get_received_message()
            
            if response and response['id'] == self.protocol.MOTOR_RESP_ID:
                parsed_response = self.protocol.parse_motor_response(response['id'], response['data'])
                if parsed_response and parsed_response.get('parsed', False):
                    self.logger.info("모터 상태 확인 완료")
                    return True
            
            self.logger.warning("모터 상태 확인 실패, 계속 진행")
            return True  # 경고만 하고 계속 진행
            
        except Exception as e:
            self.logger.error(f"모터 상태 확인 오류: {e}")
            return False
    
    def _control_loop(self):
        """제어 루프"""
        while self.running:
            try:
                start_time = time.time()
                
                # 응답 메시지 처리
                self._process_motor_responses()
                
                # 현재 제어 모드에 따른 제어 실행
                if self.current_state == MotorState.RUNNING and not self.emergency_stop_active:
                    self._execute_control()
                
                # 제어 주기 유지
                elapsed = time.time() - start_time
                sleep_time = max(0, (1.0 / self.control_frequency) - elapsed)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                    
            except Exception as e:
                self.logger.error(f"제어 루프 오류: {e}")
                time.sleep(0.01)
    
    def _process_motor_responses(self):
        """모터 응답 메시지 처리"""
        while True:
            response = self.can_manager.get_received_message()
            if not response:
                break
            
            if response['id'] == self.protocol.MOTOR_RESP_ID:
                parsed_response = self.protocol.parse_motor_response(response['id'], response['data'])
                if parsed_response:
                    self._update_motor_state(parsed_response)
    
    def _update_motor_state(self, response: Dict[str, Any]):
        """모터 상태 업데이트"""
        try:
            if response.get('parsed', False):
                # 위치 정보 업데이트
                if 'position' in response:
                    self.current_position = response['position']
                
                # 속도 정보 업데이트
                if 'velocity' in response:
                    self.current_velocity = response['velocity']
                
                # 전류 정보 업데이트
                if 'current' in response:
                    self.current_current = response['current']
                
                # 토크 정보 업데이트
                if 'torque' in response:
                    self.current_torque = response['torque']
                
                # 엔코더 정보 업데이트
                if 'encoder1' in response:
                    self.encoder1_data = response['encoder1']
                    self.encoder1_angle = response.get('encoder1_angle', 0.0)
                
                if 'encoder2' in response:
                    self.encoder2_data = response['encoder2']
                    self.encoder2_angle = response.get('encoder2_angle', 0.0)
                
                # 제어 히스토리에 추가
                self._add_to_history(response)
                
        except Exception as e:
            self.logger.error(f"모터 상태 업데이트 오류: {e}")
    
    def _add_to_history(self, data: Dict[str, Any]):
        """제어 히스토리에 데이터 추가"""
        try:
            history_entry = {
                'timestamp': time.time(),
                'position': self.current_position,
                'velocity': self.current_velocity,
                'current': self.current_current,
                'torque': self.current_torque,
                'control_mode': self.control_mode.value
            }
            
            self.control_history.append(history_entry)
            
            # 히스토리 크기 제한
            if len(self.control_history) > self.max_history_size:
                self.control_history.pop(0)
                
        except Exception as e:
            self.logger.error(f"히스토리 추가 오류: {e}")
    
    def _execute_control(self):
        """현재 제어 모드에 따른 제어 실행"""
        try:
            if self.control_mode == ControlMode.POSITION_CONTROL:
                self._execute_position_control()
            elif self.control_mode == ControlMode.VELOCITY_CONTROL:
                self._execute_velocity_control()
            elif self.control_mode == ControlMode.CURRENT_CONTROL:
                self._execute_current_control()
            elif self.control_mode == ControlMode.TORQUE_CONTROL:
                self._execute_torque_control()
            elif self.control_mode == ControlMode.FORCE_CONTROL:
                self._execute_force_control()
            elif self.control_mode == ControlMode.INCREMENTAL_CONTROL:
                self._execute_incremental_control()
                
        except Exception as e:
            self.logger.error(f"제어 실행 오류: {e}")
    
    def _execute_position_control(self):
        """위치 제어 실행"""
        try:
            # 안전 제한 확인
            if abs(self.target_velocity) > self.max_velocity:
                self.target_velocity = self.max_velocity if self.target_velocity > 0 else -self.max_velocity
            
            # 위치 제어 명령 생성 및 전송
            cmd_data = self.protocol.create_position_control_command(
                self.target_position, self.target_velocity, 0.0, 0
            )
            
            if cmd_data:
                self.can_manager.send_message(self.protocol.MOTOR_CMD_ID, cmd_data)
                
        except Exception as e:
            self.logger.error(f"위치 제어 실행 오류: {e}")
    
    def _execute_velocity_control(self):
        """속도 제어 실행"""
        try:
            # 안전 제한 확인
            if abs(self.target_velocity) > self.max_velocity:
                self.target_velocity = self.max_velocity if self.target_velocity > 0 else -self.max_velocity
            
            # 속도 제어 명령 생성 및 전송
            cmd_data = self.protocol.create_velocity_control_command(self.target_velocity, 0.0)
            
            if cmd_data:
                self.can_manager.send_message(self.protocol.MOTOR_CMD_ID, cmd_data)
                
        except Exception as e:
            self.logger.error(f"속도 제어 실행 오류: {e}")
    
    def _execute_current_control(self):
        """전류 제어 실행"""
        try:
            # 안전 제한 확인
            if abs(self.target_current) > self.max_current:
                self.target_current = self.max_current if self.target_current > 0 else -self.max_current
            
            # 전류 제어 명령 생성 및 전송
            cmd_data = self.protocol.create_current_control_command(self.target_current)
            
            if cmd_data:
                self.can_manager.send_message(self.protocol.MOTOR_CMD_ID, cmd_data)
                
        except Exception as e:
            self.logger.error(f"전류 제어 실행 오류: {e}")
    
    def _execute_torque_control(self):
        """토크 제어 실행"""
        try:
            # 안전 제한 확인
            if abs(self.target_torque) > self.max_torque:
                self.target_torque = self.max_torque if self.target_torque > 0 else -self.max_torque
            
            # 토크 제어 명령 생성 및 전송
            cmd_data = self.protocol.create_torque_control_command(self.target_torque)
            
            if cmd_data:
                self.can_manager.send_message(self.protocol.MOTOR_CMD_ID, cmd_data)
                
        except Exception as e:
            self.logger.error(f"토크 제어 실행 오류: {e}")
    
    def _execute_force_control(self):
        """힘 제어 실행 (토크 제어와 동일)"""
        self._execute_torque_control()
    
    def _execute_incremental_control(self):
        """증분 제어 실행"""
        try:
            # 증분 위치 계산
            incremental_position = self.current_position + self.target_position
            
            # 위치 제어로 실행
            cmd_data = self.protocol.create_position_control_command(
                incremental_position, self.target_velocity, 0.0, 1  # 증분 모드
            )
            
            if cmd_data:
                self.can_manager.send_message(self.protocol.MOTOR_CMD_ID, cmd_data)
                
        except Exception as e:
            self.logger.error(f"증분 제어 실행 오류: {e}")
    
    # 공개 인터페이스 메서드들
    def set_position_target(self, position: float, velocity: float = 0.0):
        """위치 목표 설정"""
        self.target_position = position
        self.target_velocity = velocity
        self.control_mode = ControlMode.POSITION_CONTROL
        self.current_state = MotorState.RUNNING
    
    def set_velocity_target(self, velocity: float):
        """속도 목표 설정"""
        self.target_velocity = velocity
        self.control_mode = ControlMode.VELOCITY_CONTROL
        self.current_state = MotorState.RUNNING
    
    def set_current_target(self, current: float):
        """전류 목표 설정"""
        self.target_current = current
        self.control_mode = ControlMode.CURRENT_CONTROL
        self.current_state = MotorState.RUNNING
    
    def set_torque_target(self, torque: float):
        """토크 목표 설정"""
        self.target_torque = torque
        self.control_mode = ControlMode.TORQUE_CONTROL
        self.current_state = MotorState.RUNNING
    
    def set_force_target(self, force: float):
        """힘 목표 설정"""
        self.set_torque_target(force)  # 힘 제어는 토크 제어와 동일
    
    def set_incremental_target(self, incremental_position: float, velocity: float = 0.0):
        """증분 위치 목표 설정"""
        self.target_position = incremental_position
        self.target_velocity = velocity
        self.control_mode = ControlMode.INCREMENTAL_CONTROL
        self.current_state = MotorState.RUNNING
    
    def stop(self):
        """모터 정지"""
        self.current_state = MotorState.IDLE
        self.target_velocity = 0.0
        self.target_current = 0.0
        self.target_torque = 0.0
    
    def emergency_stop(self):
        """비상 정지"""
        self.emergency_stop_active = True
        self.current_state = MotorState.EMERGENCY_STOP
        self.stop()
    
    def clear_emergency_stop(self):
        """비상 정지 해제"""
        self.emergency_stop_active = False
        self.current_state = MotorState.IDLE
    
    def get_motor_status(self) -> Dict[str, Any]:
        """모터 상태 반환"""
        return {
            'state': self.current_state.value,
            'control_mode': self.control_mode.value,
            'position': self.current_position,
            'velocity': self.current_velocity,
            'current': self.current_current,
            'torque': self.current_torque,
            'encoder1_angle': self.encoder1_angle,
            'encoder2_angle': self.encoder2_angle,
            'emergency_stop': self.emergency_stop_active
        }
    
    def set_safety_limits(self, max_velocity: float, max_current: float, max_torque: float):
        """안전 제한 설정"""
        self.max_velocity = max_velocity
        self.max_current = max_current
        self.max_torque = max_torque
    
    def set_pid_parameters(self, control_type: str, kp: float, ki: float, kd: float):
        """PID 파라미터 설정"""
        if control_type in self.pid_params:
            self.pid_params[control_type] = {'kp': kp, 'ki': ki, 'kd': kd}
    
    def shutdown(self):
        """모터 제어기 종료"""
        self.running = False
        self.stop()
        
        if self.control_thread and self.control_thread.is_alive():
            self.control_thread.join(timeout=1.0)
        
        self.logger.info("모터 제어기 종료 완료")
