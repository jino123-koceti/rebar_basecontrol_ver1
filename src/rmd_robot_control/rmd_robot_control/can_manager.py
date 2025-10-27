"""CAN Communication Manager for RMD Motors"""

import socket
import struct
import threading
import logging
import time
from typing import Optional, Callable

# 디버그 로거 설정
debug_logger = logging.getLogger('can_manager_debug')
debug_logger.setLevel(logging.DEBUG)
fh = logging.FileHandler('/tmp/can_manager_debug.log')
fh.setLevel(logging.DEBUG)
formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
fh.setFormatter(formatter)
debug_logger.addHandler(fh)


class CANManager:
    """
    CAN 통신 매니저
    SocketCAN을 사용하여 RMD 모터와 통신
    """

    def __init__(self, interface: str = 'can2'):
        """
        Args:
            interface: CAN 인터페이스 이름 (기본값: 'can2')
        """
        self.interface = interface
        self.socket: Optional[socket.socket] = None
        self.running = False
        self.receive_thread: Optional[threading.Thread] = None
        self.callbacks = {}
        self._lock = threading.Lock()
        self._last_send_time = 0.0  # 마지막 전송 시간 추적
        self._min_send_interval = 0.01  # 최소 전송 간격 (10ms)

    def connect(self) -> bool:
        """CAN 인터페이스 연결"""
        try:
            self.socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            # 여러 프로세스가 같은 CAN 인터페이스를 공유할 수 있도록 설정
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.bind((self.interface,))
            self.socket.settimeout(0.1)
            debug_logger.info(f"✓ CAN 인터페이스 연결 성공: {self.interface}")
            return True
        except Exception as e:
            print(f"CAN 연결 실패: {e}")
            debug_logger.error(f"❌ CAN 연결 실패: {self.interface}, 오류={e}")
            return False

    def disconnect(self):
        """CAN 인터페이스 연결 해제"""
        self.running = False
        if self.receive_thread and self.receive_thread.is_alive():
            self.receive_thread.join(timeout=1.0)

        if self.socket:
            self.socket.close()
            self.socket = None

    def send_frame(self, can_id: int, data: bytes) -> bool:
        """
        CAN 프레임 전송 (강화된 버퍼 보호)

        Args:
            can_id: CAN ID
            data: 전송할 데이터 (최대 8바이트)

        Returns:
            전송 성공 여부
        """
        if not self.socket:
            return False

        try:
            # 데이터를 8바이트로 패딩
            padded_data = data.ljust(8, b'\x00')
            frame = struct.pack("IB3x8s", can_id, len(data), padded_data)

            # 강화된 버퍼 보호 메커니즘
            max_retries = 5
            for attempt in range(max_retries):
                try:
                    # 전역 락으로 모든 CAN 명령을 순차 처리
                    with self._lock:
                        # 최소 전송 간격 보장
                        current_time = time.time()
                        time_since_last_send = current_time - self._last_send_time
                        if time_since_last_send < self._min_send_interval:
                            sleep_time = self._min_send_interval - time_since_last_send
                            debug_logger.debug(f"⏳ 최소 전송 간격 보장: {sleep_time*1000:.1f}ms 대기")
                            time.sleep(sleep_time)
                        
                        self.socket.send(frame)
                        self._last_send_time = time.time()
                    
                    debug_logger.debug(f"📤 전송: CAN ID=0x{can_id:03X}, 데이터={data.hex().upper()}")
                    return True
                    
                except OSError as e:
                    if e.errno == 105:  # No buffer space available
                        wait_time = 0.1 * (attempt + 1)  # 점진적 대기 시간
                        debug_logger.warning(f"⚠ CAN 버퍼 가득참, {wait_time:.1f}초 대기... (시도 {attempt + 1}/{max_retries})")
                        time.sleep(wait_time)
                        
                        if attempt == max_retries - 1:
                            debug_logger.error(f"❌ CAN 버퍼 오버플로우 - 최대 재시도 횟수 초과")
                            print(f"CAN 버퍼 오버플로우 (ID: 0x{can_id:03X}): 최대 재시도 횟수 초과")
                            return False
                    else:
                        raise
                        
        except Exception as e:
            print(f"CAN 전송 실패 (ID: 0x{can_id:03X}): {e}")
            debug_logger.error(f"❌ 전송 실패: CAN ID=0x{can_id:03X}, 오류={e}")
            return False

    def receive_frame(self, timeout: float = 0.1) -> Optional[tuple]:
        """
        CAN 프레임 수신

        Args:
            timeout: 수신 타임아웃 (초)

        Returns:
            (can_id, data) 튜플 또는 None
        """
        if not self.socket:
            return None

        try:
            old_timeout = self.socket.gettimeout()
            self.socket.settimeout(timeout)

            frame = self.socket.recv(16)
            can_id = struct.unpack("I", frame[:4])[0] & 0x1FFFFFFF
            dlc = frame[4]
            data = frame[8:8+dlc]

            self.socket.settimeout(old_timeout)
            return (can_id, data)
        except socket.timeout:
            return None
        except Exception as e:
            print(f"CAN 수신 오류: {e}")
            return None

    def register_callback(self, can_id: int, callback: Callable):
        """
        특정 CAN ID에 대한 콜백 등록

        Args:
            can_id: CAN ID
            callback: 콜백 함수 (can_id, data를 인자로 받음)
        """
        self.callbacks[can_id] = callback

    def unregister_callback(self, can_id: int):
        """콜백 등록 해제"""
        if can_id in self.callbacks:
            del self.callbacks[can_id]

    def start_receive_thread(self):
        """백그라운드 수신 스레드 시작"""
        if self.running:
            return

        self.running = True
        self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.receive_thread.start()

    def stop_receive_thread(self):
        """백그라운드 수신 스레드 중지"""
        self.running = False
        if self.receive_thread:
            self.receive_thread.join(timeout=1.0)

    def _receive_loop(self):
        """백그라운드 수신 루프"""
        debug_logger.info("="*60)
        debug_logger.info("CAN 수신 스레드 시작")
        debug_logger.info(f"등록된 콜백 ID: {[hex(cid) for cid in self.callbacks.keys()]}")
        debug_logger.info("="*60)

        while self.running:
            frame = self.receive_frame(timeout=0.1)
            if frame:
                can_id, data = frame
                debug_logger.debug(f"📥 수신: CAN ID=0x{can_id:03X}, 데이터={data.hex().upper()}, 길이={len(data)}")

                # 콜백 실행
                if can_id in self.callbacks:
                    debug_logger.info(f"✓ 콜백 호출: CAN ID=0x{can_id:03X}")
                    try:
                        self.callbacks[can_id](can_id, data)
                    except Exception as e:
                        print(f"콜백 실행 오류 (ID: 0x{can_id:03X}): {e}")
                        debug_logger.error(f"❌ 콜백 오류: CAN ID=0x{can_id:03X}, 오류={e}")
                else:
                    debug_logger.warning(f"⚠ 콜백 미등록: CAN ID=0x{can_id:03X}")

    def send_and_receive(self, can_id: int, data: bytes,
                        expected_response_id: int,
                        timeout: float = 0.5) -> Optional[bytes]:
        """
        동기식 송수신 (명령 전송 후 응답 대기)

        Args:
            can_id: 전송할 CAN ID
            data: 전송할 데이터
            expected_response_id: 예상 응답 CAN ID
            timeout: 응답 타임아웃

        Returns:
            응답 데이터 또는 None
        """
        # 버퍼 비우기
        while self.receive_frame(timeout=0.01):
            pass

        # 명령 전송
        if not self.send_frame(can_id, data):
            return None

        # 응답 대기
        import time
        start_time = time.time()
        while time.time() - start_time < timeout:
            frame = self.receive_frame(timeout=0.05)
            if frame:
                resp_id, resp_data = frame
                if resp_id == expected_response_id:
                    return resp_data

        return None
