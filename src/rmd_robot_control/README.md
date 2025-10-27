# RMD Robot Control Package

ROS2 기반 7모터 로봇 제어 패키지입니다. RMD-X4 액추에이터를 사용하여 CMD_VEL 제어와 위치제어를 제공합니다.

## 개요

이 패키지는 다음과 같은 기능을 제공합니다:

- **CMD_VEL 제어**: 0x141, 0x142 모터를 차동 구동으로 제어
- **위치제어**: 0x143, 0x144, 0x145, 0x146, 0x147 모터를 개별 위치제어
- **통합 상태 관리**: 모든 모터의 상태를 통합하여 관리
- **CAN 통신**: SocketCAN을 통한 RMD-X4 모터 통신
- **GUI 제어 인터페이스**: PyQt5 기반 직관적인 로봇 제어 GUI
- **통합 레이아웃**: 한 화면에서 모든 기능 확인 가능
- **동기제어**: CMD_VEL 명령 전송 버튼으로 안전한 동기 제어
- **개별 제어**: 위치제어에서 관절별 개별 명령 전송
- **실시간 모니터링**: 목표값과 현재값을 실시간으로 표시
- **안전 기능**: 비상 정지, 안전 모드, 제어 활성화/비활성화

## 패키지 구조

```
rmd_robot_control/
├── rmd_robot_control/
│   ├── __init__.py
│   ├── can_manager.py              # CAN 통신 매니저
│   ├── cmd_vel_control_node.py     # CMD_VEL 제어 노드
│   ├── position_control_node.py    # 위치제어 노드
│   ├── robot_control_node.py       # 통합 제어 노드
│   ├── robot_control_gui.py        # PyQt5 GUI 제어 인터페이스
│   ├── motor_test.py               # 테스트 노드
│   └── safe_motor_test.py          # 안전한 테스트 노드
├── config/
│   └── robot_control.yaml          # 설정 파일
├── launch/
│   ├── robot_control.launch.py     # 통합 런치 파일
│   ├── gui_control.launch.py       # GUI 제어 런치 파일
│   ├── cmd_vel_only.launch.py      # CMD_VEL만 실행
│   └── position_only.launch.py     # 위치제어만 실행
├── package.xml
├── setup.py
└── README.md
```

## 설치 및 빌드

### 의존성 설치

```bash
sudo apt update
sudo apt install ros-humble-joint-state-publisher
sudo apt install ros-humble-trajectory-msgs
sudo apt install ros-humble-control-msgs
sudo apt install python3-pyqt5
```

### 패키지 빌드

```bash
cd ~/ros2_ws
colcon build --packages-select rmd_robot_control
source install/setup.bash
```

## 사용법

### 1. GUI 제어 시스템 실행 (권장)

```bash
# GUI와 하위제어 시스템을 함께 실행
ros2 launch rmd_robot_control gui_control.launch.py

# GUI만 실행 (하위제어 노드 없이)
ros2 launch rmd_robot_control gui_control.launch.py gui_only:=true
```

### 2. 통합 제어 실행

```bash
ros2 launch rmd_robot_control robot_control.launch.py
```

### 3. CMD_VEL 제어만 실행

```bash
ros2 launch rmd_robot_control cmd_vel_only.launch.py
```

### 4. 위치제어만 실행

```bash
ros2 launch rmd_robot_control position_only.launch.py
```

### 5. 안전한 모터 테스트

```bash
# 통신 테스트 (모터 움직임 없음)
ros2 run rmd_robot_control safe_motor_test --ros-args -p test_mode:=communication

# 상태 읽기 테스트
ros2 run rmd_robot_control safe_motor_test --ros-args -p test_mode:=status_read

# 모터 ID 확인 테스트
ros2 run rmd_robot_control safe_motor_test --ros-args -p test_mode:=id_check
```

### 6. 모터 테스트 (주의: 실제 모터가 움직임)

```bash
# CMD_VEL 테스트
ros2 run rmd_robot_control motor_test --ros-args -p test_mode:=cmd_vel

# 위치제어 테스트
ros2 run rmd_robot_control motor_test --ros-args -p test_mode:=position

# 개별 관절 테스트
ros2 run rmd_robot_control motor_test --ros-args -p test_mode:=individual
```

## GUI 제어 인터페이스

### 통합 레이아웃 (한 화면에서 모든 기능 확인)

GUI는 탭 구조 대신 **한 화면에서 모든 기능을 확인할 수 있는 통합 레이아웃**을 제공합니다:

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│ 🚗 CMD_VEL 제어        │ 📊 상태 모니터링        │ ⚙️ 설정                    │
│ ┌─────────────────┐   │ ┌─────────────────┐   │ ┌─────────────────┐         │
│ │ 조이스틱 제어   │   │ │ 모터 상태       │   │ │ 안전 설정       │         │
│ │ [슬라이더들]    │   │ │ 연결 상태 표시  │   │ │ 안전 모드       │         │
│ │ [전송][정지]    │   │ │                 │   │ │ 속도 제한       │         │
│ │ 미리정의 동작   │   │ │ 실시간 로그     │   │ │                 │         │
│ │ [전진][후진]... │   │ │ (컴팩트)        │   │ │ 통신 설정       │         │
│ │ 상태 표시       │   │ │                 │   │ │ CAN: can2       │         │
│ └─────────────────┘   │ └─────────────────┘   │ └─────────────────┘         │
├─────────────────────────────────────────────────────────────────────────────────┤
│ 🎯 위치제어 (관절 제어) - 전체 너비                                               │
│ ┌─────────────────────────────────────────────────────────────────────────────┐ │
│ │ 관절1: 목표:45° 현재:42.3° [스핀박스] [전송]                              │ │
│ │ 관절2: 목표:-30° 현재:-28.7° [스핀박스] [전송]                            │ │
│ │ 관절3: 목표:90° 현재:89.2° [스핀박스] [전송]                              │ │
│ │ 관절4: 목표:-45° 현재:-44.1° [스핀박스] [전송]                            │ │
│ │ 관절5: 목표:0° 현재:0.5° [스핀박스] [전송]                                │ │
│ │ 미리정의 포즈: [홈][준비][그립][확장]                                      │ │
│ │ 궤적 제어: [텍스트 영역] [궤적 실행]                                      │ │
│ └─────────────────────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────────────────┘
```

### 주요 기능

1. **🚗 CMD_VEL 제어 (동기제어)**
   - 조이스틱 스타일 선속도/각속도 제어 슬라이더
   - **"CMD_VEL 명령 전송"** 버튼으로 동기 전송
   - **"정지"** 버튼으로 즉시 정지
   - 미리 정의된 동작 (전진, 후진, 좌회전, 우회전, 정지)
   - 실시간 바퀴 속도 모니터링

2. **🎯 위치제어 (개별 제어)**
   - **슬라이더 제거**: 정확한 각도 입력을 위한 스핀박스만 사용
   - **목표값 표시**: 초록색으로 표시되는 설정된 각도
   - **현재값 표시**: 파란색으로 표시되는 실제 모터 위치
   - **개별 전송 버튼**: 각 관절마다 파란색 "전송" 버튼
   - 미리 정의된 포즈 (홈, 준비, 그립, 확장)
   - 궤적 제어 (CSV 형식 입력)

3. **📊 상태 모니터링**
   - 모든 모터의 실시간 상태 표시
   - 연결 상태 및 통신 상태 모니터링
   - 컴팩트한 실시간 로그 표시

4. **⚙️ 설정**
   - 안전 모드 활성화/비활성화
   - 최대 속도 제한 설정
   - 통신 설정 정보 (CAN: can2, 모터: 0x141-0x147, 속도: 1Mbps)

5. **🛡️ 안전 기능**
   - 비상 정지 버튼 (빨간색)
   - 제어 활성화/비활성화 토글
   - 안전 모드 설정

### GUI 사용법

1. **시작**: `ros2 launch rmd_robot_control gui_control.launch.py`
2. **안전 확인**: 오른쪽 패널의 비상 정지 버튼이 보이는지 확인
3. **제어 활성화**: "제어 활성화" 버튼 클릭
4. **CMD_VEL 제어**: 
   - 슬라이더로 속도 설정 → "CMD_VEL 명령 전송" 버튼 클릭
   - 또는 미리 정의된 동작 버튼 사용
5. **위치제어**: 
   - 스핀박스로 각도 입력 → 개별 "전송" 버튼 클릭
   - 또는 미리 정의된 포즈 버튼 사용
6. **비상 정지**: 문제 발생 시 빨간색 비상 정지 버튼 클릭

## 토픽

### 구독 토픽

- `/cmd_vel` (geometry_msgs/Twist): 차동 구동 명령
- `/joint_trajectory` (trajectory_msgs/JointTrajectory): 관절 궤적 명령

### 발행 토픽

- `/joint_states` (sensor_msgs/JointState): 통합된 관절 상태
- `/motor_status` (std_msgs/Float64MultiArray): 모터 상태 정보

### 개별 모터 상태 토픽

- `motor_0x141_rpm` (std_msgs/Float32): 왼쪽 바퀴 RPM
- `motor_0x142_rpm` (std_msgs/Float32): 오른쪽 바퀴 RPM
- `motor_0x143_position` (std_msgs/Float32): 관절 1 위치
- `motor_0x144_position` (std_msgs/Float32): 관절 2 위치
- `motor_0x145_position` (std_msgs/Float32): 관절 3 위치
- `motor_0x146_position` (std_msgs/Float32): 관절 4 위치
- `motor_0x147_position` (std_msgs/Float32): 관절 5 위치

## 파라미터

### CMD_VEL 제어 노드

- `can_interface`: CAN 인터페이스 이름 (기본값: "can2")
- `left_motor_id`: 왼쪽 모터 ID (기본값: 0x141)
- `right_motor_id`: 오른쪽 모터 ID (기본값: 0x142)
- `wheel_radius`: 바퀴 반지름 (기본값: 0.1m)
- `wheel_base`: 바퀴 간 거리 (기본값: 0.5m)
- `max_linear_vel`: 최대 선속도 (기본값: 1.0m/s)
- `max_angular_vel`: 최대 각속도 (기본값: 1.0rad/s)

### 위치제어 노드

- `can_interface`: CAN 인터페이스 이름 (기본값: "can2")
- `motor_ids`: 모터 ID 리스트 (기본값: [0x143, 0x144, 0x145, 0x146, 0x147])
- `joint_names`: 관절 이름 리스트
- `max_position`: 최대 위치 (기본값: 1800.0도)
- `min_position`: 최소 위치 (기본값: -1800.0도)
- `max_velocity`: 최대 속도 (기본값: 100.0도/초)
- `position_tolerance`: 위치 허용 오차 (기본값: 1.0도)

## 모터 ID 매핑

| 모터 ID | 제어 방식 | 용도 |
|---------|-----------|------|
| 0x141   | CMD_VEL    | 왼쪽 바퀴 |
| 0x142   | CMD_VEL    | 오른쪽 바퀴 |
| 0x143   | 위치제어   | 관절 1 |
| 0x144   | 위치제어   | 관절 2 |
| 0x145   | 위치제어   | 관절 3 |
| 0x146   | 위치제어   | 관절 4 |
| 0x147   | 위치제어   | 관절 5 |

## RMD-X4 프로토콜

이 패키지는 RMD-X4 모터의 표준 CAN 프로토콜을 사용합니다:

- **명령 ID**: 0x140 + 모터 번호
- **응답 ID**: 0x240 + 모터 번호
- **속도 명령**: 0xA2 (RPM 단위)
- **위치 명령**: 0xA4 (도 단위)
- **상태 읽기**: 0x9C

## 문제 해결

### CAN 인터페이스 설정

```bash
# CAN 인터페이스 활성화
sudo ip link set can2 up type can bitrate 1000000

# CAN 인터페이스 상태 확인
ip link show can2
```

### 모터 통신 확인

```bash
# CAN 메시지 모니터링
candump can2

# 특정 모터 ID 확인
candump can2 | grep "141\|142\|143\|144\|145\|146\|147"
```

## 라이선스

MIT License

## 참고 자료

- [MyActuator RMD-X4 프로토콜 문서](https://github.com/2b-t/myactuator_rmd_ros)
- [RMD-X4 CAN 프로토콜](https://www.myactuator.com/)
