# 🏗️ 철근 결속 로봇 시스템 아키텍처 설명

## 📋 전체 시스템 개요

철근 결속 로봇 시스템은 **계층화된 노드 구조**로 설계되어 있으며, 각 노드는 **단일 책임 원칙**에 따라 명확한 역할을 수행합니다.

```
┌─────────────────────────────────────────────────────────────┐
│                    사용자 입력 계층                         │
│  (Iron-MD 조종기, 키보드, 상위 제어 시스템)                │
└────────────────────┬────────────────────────────────────────┘
                     │
┌────────────────────▼────────────────────────────────────────┐
│                  ROS2 토픽 통신 계층                        │
│  (/cmd_vel, /joint_X/position, /gripper/*, /motor_*/*)     │
└────────────────────┬────────────────────────────────────────┘
                     │
┌────────────────────▼────────────────────────────────────────┐
│                 하드웨어 제어 계층                          │
│  (RMD 모터, Pololu, Seengrip, EZI-IO)                      │
└─────────────────────────────────────────────────────────────┘
```

---

## 🎯 계층별 노드 구조

### 1️⃣ **입력 제어 계층** (User Interface Layer)

조종기나 키보드 입력을 받아 **표준 ROS2 메시지로 변환**하는 계층입니다.

#### 📌 `iron_md_teleop_node.py` (Iron-MD 조종기)
**역할**: Iron-MD 무선 조종기의 CAN 신호를 ROS2 토픽으로 변환

**입력**:
- CAN 메시지 (조이스틱, 스위치, Heartbeat)
  - 0x1E4: 조이스틱 아날로그 값 (AN1~AN4)
  - 0x2E4: 스위치 상태 (S00~S24)
  - 0x764: Heartbeat (연결 확인)

**출력** (ROS2 토픽):
```python
/cmd_vel                    # 주행 제어 (Twist)
/joint_1/position           # 횡이동 (Float64[])
/joint_2/position           # X축 (Float64[])
/joint_3/position           # Y축 (Float64[])
/joint_4/position           # Z축 (Float64[])
/joint_5/position           # Yaw 회전 (Float64[])
/motor_0/vel                # 트리거 (Float32)
/gripper/position           # 그리퍼 위치 (Float32)
/emergency_stop             # 비상정지 (Bool)
```

**핵심 기능**:
1. **조이스틱 정규화**: 0-255 → -1.0~1.0 (데드존 ±20)
2. **엣지 트리거**: 3단 스위치 Rising Edge만 감지 (0→1)
3. **모드 전환**: Remote(S19) / Automatic(S20)
4. **작업 시퀀스**: S21(하강+그립), S22(트리거+상승)
5. **중복 방지**: 같은 값이면 발행 안 함 (통신 부하 감소)

**제어 주기**: 50ms (20Hz)

---

#### 📌 `teleop_node.py` (키보드 조종)
**역할**: 키보드 입력을 ROS2 토픽으로 변환 (개발/디버깅용)

**입력**: 키보드 (WASD, 화살표 등)

**출력**: 동일한 ROS2 토픽 (iron_md_teleop_node와 동일)

**용도**: 조종기 없이 테스트할 때 사용

---

### 2️⃣ **통합 제어 계층** (Middleware Layer)

ROS2 토픽을 받아 **하드웨어별 프로토콜로 변환**하는 계층입니다.

#### 📌 `position_control_node.py` (RMD 모터 통합 제어)
**역할**: 7개 RMD-X4 모터를 CAN 통신으로 제어

**입력** (ROS2 토픽):
```python
/cmd_vel                    # 주행 모터 (0x141, 0x142)
/joint_1/position           # 횡이동 모터 (0x143)
/joint_2/position           # X축 모터 (0x144)
/joint_3/position           # Y축 모터 (0x145)
/joint_4/position           # Z축 모터 (0x146)
/joint_5/position           # Yaw 모터 (0x147)
```

**출력**:
- **CAN 메시지**: RMD-X4 프로토콜 (위치/속도 명령)
- **ROS2 토픽**: `/joint_states`, `/motor_status` (피드백)

**핵심 기능**:
1. **이중 제어 모드**:
   - 주행 모터 (0x141, 0x142): **속도 제어** (rpm)
   - 관절 모터 (0x143~0x147): **위치 제어** (degree)

2. **좌표 변환**:
   - `/cmd_vel` (m/s) → rpm 변환
   - `/joint_X/position` (m) → degree 변환

3. **차동 제어**:
   ```python
   left_rpm = linear - angular * wheel_base / 2
   right_rpm = linear + angular * wheel_base / 2
   ```

4. **실시간 피드백**:
   - 10Hz로 모든 모터 상태 쿼리
   - 위치, 속도, 토크, 온도 모니터링

**제어 주기**: 
- 명령 전송: 이벤트 기반 (토픽 수신 즉시)
- 상태 피드백: 100ms (10Hz)

---

#### 📌 `pololu_node.py` (Pololu 액추에이터)
**역할**: Pololu Maestro로 결속 트리거 제어

**입력** (ROS2 토픽):
```python
/motor_0/vel                # 트리거 속도 (Float32: -1.0~1.0)
```

**출력**:
- **Serial 통신**: Pololu Protocol (Compact/MiniSSC)

**핵심 기능**:
1. **속도 변환**: -1.0~1.0 → -3200~3200 (Pololu 범위)
2. **타임아웃**: 명령 없으면 자동 정지 (1초)

**제어 주기**: 이벤트 기반

---

#### 📌 `seengrip_node.py` (Seengrip 그리퍼)
**역할**: Modbus RTU로 그리퍼 간격 제어

**입력** (ROS2 토픽):
```python
/gripper/position           # 간격 (Float32: 0~2000)
/gripper/command            # 명령 (Int32: Open/Close)
```

**출력**:
- **Modbus RTU**: 레지스터 쓰기 (0x06)

**핵심 기능**:
1. **위치 제어**: 0 (닫힘) ~ 2000 (열림)
2. **상태 모니터링**: 10Hz로 현재 위치 읽기

**제어 주기**: 
- 명령 전송: 이벤트 기반
- 상태 피드백: 100ms (10Hz)

---

### 3️⃣ **안전 감시 계층** (Safety Monitoring Layer)

#### 📌 `safety_monitor.py` (안전 감시)
**역할**: 리미트 센서 감시 및 비상 정지 처리

**입력** (ROS2 토픽):
```python
/limit_sensors/x_min        # X축 최소 리미트
/limit_sensors/x_max        # X축 최대 리미트
/limit_sensors/y_min        # Y축 최소 리미트
/limit_sensors/y_max        # Y축 최대 리미트
/limit_sensors/z_min        # Z축 최소 리미트
/limit_sensors/z_max        # Z축 최대 리미트
/emergency_stop             # 비상정지 버튼
```

**출력** (ROS2 토픽):
```python
/cmd_vel                    # 정지 명령
/joint_X/position           # 정지 명령 (모든 관절)
/motor_0/vel                # 정지 명령
```

**핵심 기능**:
1. **리미트 감지**: 센서 트리거 시 해당 축 정지
2. **비상정지**: 모든 모터 즉시 정지
3. **자동 복구**: 비상정지 해제 시 제어 재개

**모니터링 주기**: 100ms (10Hz)

---

#### 📌 `ezi_io_node.py` (EZI-IO 센서)
**역할**: Modbus TCP로 리미트 센서 상태 읽기

**입력**:
- **Modbus TCP**: EZI-IO PLC (192.168.0.2:502)

**출력** (ROS2 토픽):
```python
/limit_sensors/x_min        # Bool
/limit_sensors/x_max        # Bool
/limit_sensors/y_min        # Bool
/limit_sensors/y_max        # Bool
/limit_sensors/z_min        # Bool
/limit_sensors/z_max        # Bool
```

**핵심 기능**:
1. **입력 비트 읽기**: Modbus FC03 (Read Holding Registers)
2. **비트 파싱**: 6개 센서 상태 분리
3. **실시간 발행**: 변화 감지 시 즉시 발행

**모니터링 주기**: 50ms (20Hz)

---

## 🔄 데이터 흐름 (Data Flow)

### 예시 1: 조이스틱으로 주행하기

```
[Iron-MD 조종기]
     │
     │ AN3 조이스틱 움직임 (CAN 0x1E4)
     ▼
[iron_md_teleop_node]
     │
     │ 1. CAN 메시지 수신 (50ms)
     │ 2. 조이스틱 정규화 (0-255 → -1.0~1.0)
     │ 3. 데드존 체크 (±20)
     │ 4. 중복 발행 방지
     ▼
/cmd_vel (Twist)
     │ linear.x = 0.3 m/s
     │ angular.z = 0.0 rad/s
     ▼
[position_control_node]
     │
     │ 1. 토픽 수신
     │ 2. m/s → rpm 변환
     │ 3. 차동 제어 계산
     │    left_rpm = 0.3 * 60 / (π * 0.15) = 38 rpm
     │    right_rpm = 38 rpm
     │ 4. RMD-X4 프로토콜 패킹
     ▼
[CAN 버스 (0x141, 0x142)]
     │
     │ 0x141: [0xA2, 0x00, 0x00, 0x26, 0x00, 0x00, 0x00, 0x00]
     │ 0x142: [0xA2, 0x00, 0x00, 0x26, 0x00, 0x00, 0x00, 0x00]
     ▼
[RMD-X4 모터]
     │
     │ 무한궤도 회전 (38 rpm)
     ▼
[로봇 주행]
```

---

### 예시 2: S21 스위치로 작업 시퀀스

```
[Iron-MD 조종기]
     │
     │ S21 ON (CAN 0x2E4, Byte2 bit 1)
     ▼
[iron_md_teleop_node]
     │
     │ 1. CAN 메시지 수신
     │ 2. 스위치 상태 파싱
     │ 3. Rising Edge 감지 (0→1)
     │ 4. handle_work_sequence() 호출
     ▼
자동 시퀀스 실행:
     │
     ├─▶ /joint_4/position (Z축 -0.1m 하강)
     │        ▼
     │   [position_control_node]
     │        │ -0.1m → -360° 변환
     │        ▼
     │   [CAN 0x146: Z축 모터]
     │        ▼
     │   Z축 100mm 하강
     │
     ├─▶ 1초 대기
     │
     └─▶ /gripper/position (0 = 닫기)
              ▼
         [seengrip_node]
              │ Modbus 레지스터 쓰기
              ▼
         [Seengrip 그리퍼]
              ▼
         철근 파지 완료
```

---

### 예시 3: 리미트 센서 감지

```
[EZI-IO PLC]
     │
     │ X축 최대 리미트 센서 ON
     ▼
[ezi_io_node]
     │
     │ 1. Modbus TCP 읽기 (50ms 주기)
     │ 2. 레지스터 비트 파싱
     │ 3. 상태 변화 감지
     ▼
/limit_sensors/x_max (Bool: True)
     │
     ▼
[safety_monitor]
     │
     │ 1. 리미트 감지
     │ 2. X축 양의 방향 제어 차단
     │ 3. 경고 로그 출력
     ▼
/joint_2/position 명령 거부
     │
     ▼
[position_control_node]
     │
     │ X축 양의 방향 명령 무시
     ▼
X축 정지 (안전 보호)
```

---

## 🎨 노드 분리 설계 철학

### 1️⃣ **단일 책임 원칙** (Single Responsibility Principle)

각 노드는 **하나의 명확한 역할**만 수행합니다.

| 노드 | 책임 | 이유 |
|------|------|------|
| `iron_md_teleop_node` | CAN → ROS2 변환 | 조종기 프로토콜 독립 |
| `position_control_node` | ROS2 → RMD CAN | 모터 제어 로직 집중 |
| `pololu_node` | ROS2 → Pololu Serial | 액추에이터 독립 |
| `seengrip_node` | ROS2 → Modbus RTU | 그리퍼 독립 |
| `ezi_io_node` | Modbus TCP → ROS2 | 센서 독립 |
| `safety_monitor` | 안전 감시 | 안전 로직 집중 |

**장점**:
- 하나의 노드 수정이 다른 노드에 영향 없음
- 테스트 용이 (개별 노드 독립 실행 가능)
- 디버깅 간편 (문제 원인 쉽게 찾음)

---

### 2️⃣ **계층화** (Layered Architecture)

```
입력 계층 ──▶ 통합 제어 계층 ──▶ 하드웨어 계층
   ▲                              │
   └──────── 안전 감시 계층 ◀──────┘
```

**장점**:
- 조종기 교체 시: `iron_md_teleop_node`만 교체
- 모터 교체 시: `position_control_node`만 수정
- 상위 제어 시스템 연동: 입력 계층만 추가

---

### 3️⃣ **통신 프로토콜 캡슐화** (Protocol Encapsulation)

각 노드가 **하드웨어별 프로토콜을 캡슐화**합니다.

| 하드웨어 | 프로토콜 | 담당 노드 | 외부 인터페이스 |
|----------|----------|-----------|----------------|
| RMD-X4 | CAN (RMD-X4) | position_control_node | ROS2 토픽 |
| Pololu | Serial (Compact) | pololu_node | ROS2 토픽 |
| Seengrip | Modbus RTU | seengrip_node | ROS2 토픽 |
| EZI-IO | Modbus TCP | ezi_io_node | ROS2 토픽 |
| Iron-MD | CAN (Iron-MD) | iron_md_teleop_node | ROS2 토픽 |

**장점**:
- 다른 노드는 하드웨어 프로토콜 몰라도 됨
- ROS2 표준 메시지만 알면 통신 가능
- 하드웨어 변경 시 해당 노드만 수정

---

### 4️⃣ **이벤트 기반 설계** (Event-Driven)

대부분의 노드가 **이벤트 기반**으로 동작합니다.

```python
# position_control_node.py
def cmd_vel_callback(self, msg):
    """토픽 수신 즉시 처리 (이벤트 기반)"""
    linear = msg.linear.x
    angular = msg.angular.z
    # 즉시 CAN 메시지 전송
    self.send_velocity_command(linear, angular)
```

**장점**:
- CPU 사용량 최소화 (필요할 때만 동작)
- 응답성 향상 (명령 즉시 처리)
- CAN 버스 부하 감소 (불필요한 메시지 없음)

---

### 5️⃣ **상태 피드백 분리** (Feedback Separation)

제어 명령과 상태 피드백을 **분리된 타이머**로 처리합니다.

```python
# position_control_node.py
def __init__(self):
    # 제어 명령: 이벤트 기반
    self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
    
    # 상태 피드백: 주기적 (10Hz)
    self.status_timer = self.create_timer(0.1, self.publish_integrated_status)
```

**장점**:
- 제어와 모니터링 독립 실행
- 피드백 주기 조절 가능 (10Hz, 100Hz 등)
- 제어 지연 최소화

---

## 📊 노드별 통신 부하

| 노드 | 입력 주기 | 출력 주기 | CAN 부하 | 비고 |
|------|----------|----------|----------|------|
| `iron_md_teleop_node` | 50ms (20Hz) | 이벤트 | - | CAN2 수신만 |
| `position_control_node` | 이벤트 | 100ms (10Hz) | 16% | CAN0 양방향 |
| `pololu_node` | 이벤트 | - | - | Serial 통신 |
| `seengrip_node` | 이벤트 | 100ms (10Hz) | - | Modbus RTU |
| `ezi_io_node` | - | 50ms (20Hz) | - | Modbus TCP |
| `safety_monitor` | 100ms (10Hz) | 이벤트 | - | ROS2만 |

---

## 🔧 확장성 (Scalability)

### 새로운 조종기 추가
```python
# 새 노드: ps4_teleop_node.py
class PS4TeleopNode(Node):
    def __init__(self):
        # PS4 컨트롤러 입력 받기
        # 동일한 ROS2 토픽 발행 (/cmd_vel, /joint_X/position)
        pass
```
→ 기존 노드 수정 없음!

### 새로운 모터 추가
```python
# position_control_node.py 수정
motor_ids = [0x143, 0x144, 0x145, 0x146, 0x147, 0x148]  # 0x148 추가
joint_names = ['joint_1', ..., 'joint_6']

# iron_md_teleop_node.py 추가
self.joint6_pub = self.create_publisher(Float64MultiArray, '/joint_6/position', 10)
```
→ 2개 노드만 수정!

### 상위 제어 시스템 연동
```python
# higher_control_node.py (새 노드)
class HigherControlNode(Node):
    def __init__(self):
        # 작업 계획 생성
        # 동일한 ROS2 토픽 발행
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
```
→ 기존 시스템 그대로 사용!

---

## ✅ 최종 정리

### 노드 분리의 핵심 원칙

1. **입력 변환 노드**: 하드웨어 입력 → ROS2 메시지
2. **제어 노드**: ROS2 메시지 → 하드웨어 명령
3. **감시 노드**: 안전 상태 모니터링
4. **표준 인터페이스**: 모든 노드 간 ROS2 토픽 사용
5. **독립 실행**: 각 노드 개별 테스트 가능

### 통신 구조

```
Iron-MD ──CAN──▶ iron_md_teleop_node ──ROS2──▶ position_control_node ──CAN──▶ RMD 모터
                                      └─ROS2──▶ pololu_node ──Serial──▶ Pololu
                                      └─ROS2──▶ seengrip_node ──Modbus──▶ Seengrip
                                                        ▲
                                                        │
                                                      ROS2
                                                        │
ezi_io_node ──ROS2──▶ safety_monitor ─────────────────┘
      ▲
      │
   Modbus
      │
   EZI-IO PLC
```

이렇게 노드를 분리함으로써:
- ✅ 유지보수 용이
- ✅ 확장성 확보
- ✅ 테스트 간편
- ✅ 재사용성 향상
- ✅ 독립 개발 가능

각 노드가 명확한 역할만 수행하기 때문에, 시스템 전체를 이해하기 쉽고 문제 해결도 빠릅니다! 🎉
