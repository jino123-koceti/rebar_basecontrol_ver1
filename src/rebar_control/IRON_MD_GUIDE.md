# Iron-MD 무선 조종기 CAN 통신 가이드

## 📡 개요

Iron-MD는 **CAN 통신 기반** 무선 조종기입니다. 일반 Joystick과 달리 전용 CAN 프로토콜을 사용합니다.

## 🔌 하드웨어 연결

### 7핀 커넥터 (Connector SF1212/S7)

```
송신기(TX) 측:                수신기(RX) 측:
1: CAN_L (White)          3: CAN-H/RS232 TX (청색)
2: CAN_H (Green)          5: CAN-L/RS232 RX (보라)
3: NC                     4: POWER 24V_in (빨강)
4: Power GND (Black)      7: 0V(흑색)/GND(초록)
5: Power+ 24V (Red)
6: RS232_TX (Blue)
7: RS232_RX (Purple)
```

### CAN 인터페이스

- **통신 방식**: CAN 2.0
- **속도**: 125/250/500 Kbps (기본 250Kbps)
- **인터페이스**: can1 (RMD 모터는 can0)

## 📋 CAN 메시지 프로토콜

### 1. 조이스틱 데이터 (0x1E4 / 484 decimal)

**전송 주기**: 50ms

| Byte | 비트 | 이름 | 범위 | 설명 |
|------|------|------|------|------|
| 0 | 0-7 | AN1 | 0-255 | Joystick 1 (주행 전후) |
| 1 | 0-7 | AN2 | 0-255 | Joystick 2 (주행 좌우) |
| 2 | 0-7 | AN3 | 0-255 | Joystick 3 (X축) |
| 3 | 0-7 | AN4 | 0-255 | Joystick 4 (Z축) |
| 4-7 | - | - | - | 사용 안함 |

**조이스틱 값**:
- 중립: 127
- 최소: 0
- 최대: 255

### 2. 스위치 상태 (0x2E4 / 740 decimal)

**전송 주기**: 50ms

#### Byte 0: 메인 컨트롤
| 비트 | 이름 | 설명 |
|------|------|------|
| 0 | Start_Key | 시작 키 |
| 1 | Power_Key | 전원 키 |
| 2 | S13 | 엔진 시작 |
| 3 | S14 | 엔진 정지 |
| 6 | Emergency_Stop_Release | 0=정지, 1=해제 |
| 7 | Emergency_Stop_Active | 1=정지, 0=해제 |

#### Byte 1: 스위치 S00-S07
| 비트 | 스위치 | 철근 로봇 매핑 |
|------|--------|----------------|
| 0 | S06 | - |
| 1 | S07 | - |
| 4 | S02 | - |
| 5 | S03 | - |
| 6 | S01 | 트리거 해제 |
| 7 | S00 | 트리거 당김 |

#### Byte 2: 스위치 S08-S09
| 비트 | 스위치 | 철근 로봇 매핑 |
|------|--------|----------------|
| 0 | S08 | - |
| 1 | S09 | - |

#### Byte 3: 스위치 S17-S24
| 비트 | 스위치 | 철근 로봇 매핑 |
|------|--------|----------------|
| 0 | S23 | 그리퍼 열기 |
| 1 | S24 | 그리퍼 닫기 |
| 2 | S21 | 리프팅 상승 |
| 3 | S22 | 리프팅 하강 |
| 4 | S19 | Yaw 반시계 |
| 5 | S20 | Yaw 시계 |
| 6 | S17 | Y축 전진 |
| 7 | S18 | Y축 후진 |

#### Byte 4: 카운터 및 모드
| 비트 | 이름 | 설명 |
|------|------|------|
| 0-3 | Run_Counter | 수신 카운터 (0-15) |
| 4 | Cal_Mode | 0=일반, 1=교정 |

#### Byte 6: 연결 상태
| 비트 | 이름 | 설명 |
|------|------|------|
| 6 | TX_Connected | 1=송신기 연결됨 |
| 7 | TRX_Code | 1=TX, 0=RX |

#### Byte 7-8: 기기 정보
| Byte | 이름 | 설명 |
|------|------|------|
| 7 | Serial_Number | 시리얼 번호 |
| 8 | Device_ID | 기기 ID |

### 3. Heartbeat (0x764 / 1892 decimal)

**전송 주기**: 300ms

| Byte | 이름 | 값 | 설명 |
|------|------|-----|------|
| 0 | Heart_Bit | 0 | 항상 0 |

## 🎮 철근 로봇 조종기 매핑

### 아날로그 조이스틱
```
AN1 (Joystick 1) → 주행 전진/후진
AN2 (Joystick 2) → 주행 좌/우 회전
AN3 (Joystick 3) → X축 이동 (연속)
AN4 (Joystick 4) → Z축 이동 (연속)
```

### 디지털 스위치
```
[XYZ 스테이지]
S17 → Y축 전진 (스텝)
S18 → Y축 후진 (스텝)
S19 → Yaw 반시계 회전 (스텝)
S20 → Yaw 시계 회전 (스텝)

[리프팅]
S21 → 상승 (50mm)
S22 → 하강 (50mm)

[그리퍼]
S23 → 열기
S24 → 닫기

[결속 건]
S00 → 트리거 당김
S01 → 트리거 해제

[안전]
Emergency_Stop → 비상 정지 (하드웨어)
```

## 🚀 사용법

### 1. CAN 인터페이스 설정

```bash
# can1 인터페이스 활성화 (250 Kbps)
sudo ip link set can1 up type can bitrate 250000

# 확인
ip link show can1
```

### 2. 노드 실행

```bash
# 단독 실행
ros2 run rebar_control iron_md_teleop_node

# 전체 시스템 (full_system.launch.py에 포함됨)
ros2 launch rebar_control full_system.launch.py
```

### 3. CAN 메시지 모니터링

```bash
# CAN 메시지 확인
candump can1

# 조이스틱 데이터만
candump can1,1E4:7FF

# 스위치 상태만
candump can1,2E4:7FF
```

## 🔍 디버깅

### CAN 메시지 확인

```bash
# 실시간 모니터링
candump can1

# 예상 출력:
# can1  1E4   [8]  7F 7F 7F 7F 00 00 00 00  <- 조이스틱 중립
# can1  2E4   [8]  00 00 00 00 00 00 00 00  <- 스위치 모두 꺼짐
# can1  764   [8]  00 00 00 00 00 00 00 00  <- Heartbeat
```

### 송신기 연결 상태

```bash
# ROS2 토픽으로 확인 (구현 예정)
ros2 topic echo /iron_md/connection_status

# 또는 로그 확인
ros2 node list
ros2 node info /iron_md_teleop
```

### 조이스틱 값 테스트

```python
# can1에서 직접 수신 테스트
import can

bus = can.interface.Bus(channel='can1', bustype='socketcan', bitrate=250000)

while True:
    msg = bus.recv(timeout=1.0)
    if msg and msg.arbitration_id == 0x1E4:
        print(f"Joystick: AN1={msg.data[0]} AN2={msg.data[1]} "
              f"AN3={msg.data[2]} AN4={msg.data[3]}")
```

## ⚙️ 파라미터 조정

`config/iron_md.yaml`:

```yaml
iron_md_teleop:
  ros__parameters:
    can_interface: "can1"
    can_baudrate: 250000
    
    # 속도 제한
    max_linear_speed: 0.5    # m/s
    max_angular_speed: 1.0   # rad/s
    xyz_step_size: 0.01      # 10mm
    yaw_step_size: 0.1       # rad
    lifting_step_size: 0.05  # 50mm
    
    # 조이스틱 설정
    joystick_center: 127     # 중립값
    joystick_deadzone: 20    # 데드존 ±20
    
    # 트리거
    trigger_duration: 0.5    # 자동 해제 시간(초)
```

## 🛡️ 안전 기능

### 1. 송신기 연결 감시
- `TX_Connected` 비트 확인
- 연결 끊김 시 자동으로 모든 모터 정지

### 2. 하드웨어 비상 정지
- `Emergency_Stop_Active` 비트
- 하드웨어 레벨에서 즉시 정지

### 3. Heartbeat 모니터링
- 300ms 주기로 수신 확인
- 타임아웃 시 연결 끊김으로 처리 (구현 예정)

## 📦 필요한 패키지

```bash
# Python CAN 라이브러리
pip3 install python-can

# CAN 유틸리티
sudo apt install can-utils
```

## 🔧 트러블슈팅

### CAN 인터페이스 인식 안됨

```bash
# 사용 가능한 CAN 인터페이스 확인
ip link show | grep can

# can1이 없으면 USB-CAN 어댑터 확인
lsusb
dmesg | grep can
```

### 메시지 수신 안됨

```bash
# CAN 상태 확인
ip -details link show can1

# 재시작
sudo ip link set can1 down
sudo ip link set can1 up type can bitrate 250000

# 송신기 전원 확인 (24V)
```

### 조이스틱 응답 없음

1. candump으로 메시지 수신 확인
2. TX_Connected 비트 확인
3. 조이스틱 중립값 확인 (127)
4. 데드존 설정 확인

## 📄 참고 자료

- DBC 파일: `iron_md_joystick.dbc`
- 사양서: `IRON-MD_CAN_Specification.md`
- PDF 원본: `IRON-MD_일반형_CAN_8Char_Specification_20240329.pdf`
