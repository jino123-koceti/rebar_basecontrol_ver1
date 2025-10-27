# Iron-MD 조종기 통합 완료 ✅

## 🎯 변경 사항 요약

### 1. **새로운 노드 생성**
- `iron_md_teleop_node.py` - CAN 통신 기반 텔레옵 노드
- 기존 `teleop_node.py`는 보존 (일반 Joy용 참고)

### 2. **주요 차이점**

| 항목 | 기존 (teleop_node) | 신규 (iron_md_teleop_node) |
|------|-------------------|--------------------------|
| 입력 방식 | ROS2 Joy 토픽 | **CAN 메시지 직접 수신** |
| 통신 | USB Joystick | **CAN (can1, 250Kbps)** |
| 프로토콜 | sensor_msgs/Joy | **Iron-MD 전용 (0x1E4, 0x2E4)** |
| 의존성 | joy package | **python-can** |

### 3. **CAN 메시지 프로토콜**

#### 조이스틱 데이터 (0x1E4)
```
Byte 0: AN1 (0-255) → 주행 전후
Byte 1: AN2 (0-255) → 주행 좌우  
Byte 2: AN3 (0-255) → X축
Byte 3: AN4 (0-255) → Z축
```

#### 스위치 상태 (0x2E4)
```
Byte 0, bit 7: Emergency_Stop_Active
Byte 1, bit 7: S00 → 트리거 당김
Byte 1, bit 6: S01 → 트리거 해제
Byte 3, bit 0: S23 → 그리퍼 열기
Byte 3, bit 1: S24 → 그리퍼 닫기
Byte 3, bit 2: S21 → 리프팅 상승
Byte 3, bit 3: S22 → 리프팅 하강
Byte 3, bit 4: S19 → Yaw CCW
Byte 3, bit 5: S20 → Yaw CW
Byte 3, bit 6: S17 → Y축 전진
Byte 3, bit 7: S18 → Y축 후진
```

## 🔧 설정 방법

### 1. 의존성 설치
```bash
# Python CAN 라이브러리
pip3 install python-can

# CAN 유틸리티 (디버깅용)
sudo apt install can-utils
```

### 2. CAN 인터페이스 설정
```bash
# can0: RMD 모터 (1 Mbps)
sudo ip link set can0 up type can bitrate 1000000

# can1: Iron-MD 조종기 (250 Kbps) ← 새로 추가!
sudo ip link set can1 up type can bitrate 250000
```

### 3. 실행
```bash
# 빌드
cd ~/ros2_ws_backup
colcon build --packages-select rebar_control
source install/setup.bash

# 단독 실행
ros2 run rebar_control iron_md_teleop_node

# 또는 전체 시스템
ros2 launch rebar_control full_system.launch.py
```

## 🧪 테스트 방법

### 1. CAN 메시지 확인
```bash
# 조종기 메시지 모니터링
candump can1

# 예상 출력:
can1  1E4   [8]  7F 7F 7F 7F 00 00 00 00  ← 조이스틱 중립
can1  2E4   [8]  00 00 00 00 00 00 00 00  ← 스위치 꺼짐
can1  764   [8]  00 00 00 00 00 00 00 00  ← Heartbeat
```

### 2. 조이스틱 테스트
```bash
# 조이스틱을 움직이면:
can1  1E4   [8]  FF 7F 7F 7F 00 00 00 00  ← AN1=255 (전진)
can1  1E4   [8]  7F FF 7F 7F 00 00 00 00  ← AN2=255 (우회전)
```

### 3. 스위치 테스트
```bash
# S21 버튼 누르면:
can1  2E4   [8]  00 00 00 04 00 00 00 00  ← Byte 3, bit 2 = 1
```

## 📊 노드 동작 흐름

```
┌──────────────────────────────────────────┐
│   Iron-MD 송신기 (무선)                   │
└────────────┬─────────────────────────────┘
             │ 무선 2.4GHz
             ▼
┌──────────────────────────────────────────┐
│   Iron-MD 수신기 (CAN 출력)               │
└────────────┬─────────────────────────────┘
             │ CAN (250 Kbps)
             │ 0x1E4: 조이스틱 (50ms)
             │ 0x2E4: 스위치 (50ms)
             │ 0x764: Heartbeat (300ms)
             ▼
┌──────────────────────────────────────────┐
│   CAN1 Interface (Linux)                 │
└────────────┬─────────────────────────────┘
             │ python-can
             ▼
┌──────────────────────────────────────────┐
│   iron_md_teleop_node                    │
│   - CAN 메시지 수신 스레드                │
│   - 조이스틱 값 정규화 (0-255 → -1~1)    │
│   - 스위치 엣지 감지                      │
│   - ROS2 토픽 발행 (20Hz)                │
└────────────┬─────────────────────────────┘
             │ ROS2 Topics
             ├─ /cmd_vel
             ├─ /joint_*/position
             ├─ /motor_0/vel
             ├─ /gripper/position
             └─ /emergency_stop
             ▼
      [각 하드웨어 노드들]
```

## 🎮 조종기 매핑 다이어그램

```
     Iron-MD 조종기
┌─────────────────────────┐
│                         │
│   AN1 ↕ (Joystick 1)   │ → 주행 전후
│   AN2 ↔ (Joystick 2)   │ → 주행 좌우
│                         │
│   AN3 ↔ (Joystick 3)   │ → X축
│   AN4 ↕ (Joystick 4)   │ → Z축
│                         │
│   [S17] [S18]           │ → Y축 전/후
│   [S19] [S20]           │ → Yaw 좌/우
│                         │
│   [S21] [S22]           │ → 리프팅 상/하
│   [S23] [S24]           │ → 그리퍼 열기/닫기
│   [S00] [S01]           │ → 트리거 당김/해제
│                         │
│   🚨 Emergency_Stop     │ → 비상 정지
└─────────────────────────┘
```

## 📝 파일 구조

```
rebar_control/
├── rebar_control/
│   ├── iron_md_teleop_node.py    ← 새로 추가! (CAN 기반)
│   ├── teleop_node.py             (기존, Joy 기반)
│   ├── safety_monitor.py
│   └── __init__.py
├── config/
│   └── iron_md.yaml               ← 새로 추가!
├── launch/
│   └── full_system.launch.py      (업데이트됨)
├── IRON_MD_GUIDE.md               ← 새로 추가! (상세 가이드)
├── iron_md_joystick.dbc           (CAN DB 파일)
├── IRON-MD_CAN_Specification.md   (사양서)
└── README.md                      (업데이트됨)
```

## ⚠️ 주의사항

### 1. CAN 인터페이스 분리
- **can0**: RMD 모터 (1 Mbps)
- **can1**: Iron-MD 조종기 (250 Kbps)
- 반드시 **별도의 CAN 인터페이스** 사용!

### 2. 조이스틱 값 정규화
- 원본: 0-255 (중립: 127)
- 정규화: -1.0 ~ 1.0 (중립: 0.0)
- 데드존: ±20 (설정 가능)

### 3. 송신기 연결 확인
- `TX_Connected` 비트 감시
- 연결 끊김 시 자동으로 모든 모터 정지

### 4. 비상 정지 이중화
- 하드웨어: `Emergency_Stop_Active` 비트
- 소프트웨어: ROS2 `/emergency_stop` 토픽

## 🔍 디버깅 팁

### Python CAN 직접 테스트
```python
import can

# CAN 버스 열기
bus = can.interface.Bus(channel='can1', bustype='socketcan', bitrate=250000)

# 메시지 수신
while True:
    msg = bus.recv(timeout=1.0)
    if msg:
        print(f"ID: 0x{msg.arbitration_id:03X} Data: {msg.data.hex()}")
```

### 조이스틱 값 확인
```python
# 0x1E4 메시지 파싱
if msg.arbitration_id == 0x1E4:
    AN1, AN2, AN3, AN4 = msg.data[0:4]
    print(f"Joystick: AN1={AN1} AN2={AN2} AN3={AN3} AN4={AN4}")
```

## 📚 추가 문서

- **상세 가이드**: [IRON_MD_GUIDE.md](IRON_MD_GUIDE.md)
- **CAN 사양**: [IRON-MD_CAN_Specification.md](IRON-MD_CAN_Specification.md)
- **DBC 파일**: [iron_md_joystick.dbc](iron_md_joystick.dbc)

## ✅ 체크리스트

실행 전 확인사항:

- [ ] python-can 설치됨
- [ ] can1 인터페이스 활성화 (250 Kbps)
- [ ] candump으로 메시지 수신 확인
- [ ] TX_Connected 비트 확인 (송신기 연결)
- [ ] 조이스틱 중립값 127 확인
- [ ] 전체 시스템 빌드 완료

---

**모든 준비 완료!** 🎉
Iron-MD CAN 조종기로 철근 결속 로봇을 제어할 수 있습니다!
