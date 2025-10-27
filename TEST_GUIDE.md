# 🧪 Iron-MD 텔레옵 노트북 테스트 가이드

노트북 `can0`에서 Iron-MD 조종기 신호만 받아서 텔레옵 노드가 제대로 동작하는지 확인합니다.

## 📋 테스트 환경

- **CAN 인터페이스**: `can0` (노트북)
  - 실제 장비: `can2`
- **연결 장비**: Iron-MD 조종기 수신기만
- **모터**: 없음 (터미널 출력으로 확인)
- **디버그 모드**: 활성화

## 🔧 사전 준비

### 1. CAN 인터페이스 설정

```bash
# CAN 인터페이스 활성화 (250Kbps)
sudo ip link set can0 type can bitrate 250000
sudo ip link set can0 up

# CAN 상태 확인
ip -details link show can0
```

### 2. Iron-MD 조종기 수신기 연결

- USB-CAN 어댑터에 Iron-MD 수신기 연결
- 조종기 전원 ON
- 연결 LED 확인

### 3. CAN 메시지 확인

```bash
# 조종기 메시지 수신되는지 확인
candump can0

# 확인할 메시지:
# - 0x1E4: 조이스틱 데이터 (50ms 주기)
# - 0x2E4: 스위치 상태 (50ms 주기)
# - 0x764: Heartbeat (300ms 주기)
```

예상 출력:
```
can0  1E4   [4]  7F 7F 7F 7F        # 조이스틱 중립 (127,127,127,127)
can0  2E4   [8]  00 00 00 00 00 00 40 00  # 스위치 OFF, TX 연결됨
can0  764   [8]  00 00 00 00 00 00 00 00  # Heartbeat
```

## 🚀 테스트 실행

### 방법 1: 런치 파일로 실행 (권장)

```bash
# 빌드
cd ~/ros2_ws_backup
colcon build --packages-select rebar_control
source install/setup.bash

# 테스트 런치 실행
ros2 launch rebar_control test_teleop.launch.py
```

### 방법 2: 직접 실행

```bash
# 파라미터 파일로 실행
ros2 run rebar_control iron_md_teleop \
  --ros-args --params-file src/rebar_control/config/test_params.yaml

# 또는 커맨드라인 파라미터
ros2 run rebar_control iron_md_teleop \
  --ros-args \
  -p can_interface:=can0 \
  -p debug_mode:=true
```

## 📊 예상 출력

### 초기 시작 화면

```
[INFO] [iron_md_teleop]: Iron-MD Teleop node started (CAN: can0)
[INFO] [iron_md_teleop]: 🔍 DEBUG MODE: 터미널 출력 상세화
[INFO] [iron_md_teleop]: 
╔════════════════════════════════════════════════════════════════════╗
║         철근 결속 로봇 Iron-MD 조종기 매핑 (CAN)                  ║
╠════════════════════════════════════════════════════════════════════╣
...
```

### 1초마다 자동 출력 (디버그 모드)

```
┌─────────────────────────────────────────────────────────────────────┐
│                  🎮 Iron-MD 조종기 상태 (DEBUG)                    │
├─────────────────────────────────────────────────────────────────────┤
│ 연결 상태: ✅ 송신기 연결됨
│ 제어 모드: 🎮 Remote Control
│ 비상정지: ✅ 정상
│
│ [조이스틱 값 - Raw / 정규화]
│   AN1 (X축):   127 → +0.00
│   AN2 (Y축):   127 → +0.00
│   AN3 (주행):  127 → +0.00
│   AN4 (예비):  127 → +0.00
│
│ [현재 위치]
│   하부체 횡이동: +0.000 m
│   상부체 X축:   +0.000 m
│   상부체 Y축:   +0.000 m
│   상부체 Z축:   +0.000 m
│   공구 Yaw:    +0.0°
│
│ [스위치 상태]
│   모드: S19=ON S20=off
│   횡이동: S17=off S18=off
│   작업: S21=off S22=off
│   Yaw: S23=off S24=off
│   트리거: S00=off S01=off
└─────────────────────────────────────────────────────────────────────┘
```

## 🎯 테스트 시나리오

### 1. 조이스틱 테스트

#### AN3 (주행 전후진)
```
동작: AN3 조이스틱을 앞으로 밀기 (255)
예상 출력:
  📊 조이스틱: AN1=127 AN2=127 AN3=255 AN4=127
  🚗 주행 → /cmd_vel: linear=-0.50 m/s, angular=+0.00 rad/s
```

#### AN1 (X축)
```
동작: AN1 조이스틱을 좌측으로 이동 (0)
예상 출력:
  📐 X축 → /joint_2/position: +0.0100 m (delta: +0.0100)
  (연속 이동하면 계속 증가)
```

#### AN2 (Y축)
```
동작: AN2 조이스틱을 우측으로 이동 (255)
예상 출력:
  📐 Y축 → /joint_3/position: -0.0100 m (delta: -0.0100)
```

### 2. 모드 전환 테스트

#### Remote → Automatic
```
동작: S19-S20 스위치를 하단으로 내림 (S20 ON)
예상 출력:
  🤖 Automatic Control 모드 (상위제어 대기)
상태창:
  제어 모드: 🤖 Automatic Control
```

### 3. 횡이동 테스트

#### S17 (양의 방향)
```
동작: S17 스위치 올렸다가 중립 복귀
예상 출력:
  ➡️  횡이동 +50mm (위치: 0.050m)
  📍 LATERAL: 0.050 m
상태창:
  하부체 횡이동: +0.050 m
```

#### S18 (음의 방향)
```
동작: S18 스위치 내렸다가 중립 복귀
예상 출력:
  ⬅️  횡이동 -50mm (위치: 0.000m)
상태창:
  하부체 횡이동: +0.000 m
```

### 4. 작업 시퀀스 테스트

#### S21 (그립 준비)
```
동작: S21 스위치 올렸다가 중립 복귀
예상 출력:
  🔽 작업 시퀀스: Z축 하강 → 그리퍼 닫기
  📍 Z: -0.100 m
  (그리퍼 위치 0 발행)
```

#### S22 (결속 실행)
```
동작: S22 스위치 내렸다가 중립 복귀
예상 출력:
  🔧 작업 시퀀스: 트리거 → 그리퍼 열기 → Z축 상승
  🔫 트리거 당김!
  (0.5초 후)
  🔫 트리거 해제
  📍 Z: +0.000 m
```

### 5. Yaw 회전 테스트

#### S23 (+30도)
```
동작: S23 스위치 올렸다가 중립 복귀
예상 출력:
  ↻  Yaw +30° (위치: 30.0°)
상태창:
  공구 Yaw: +30.0°
```

#### S24 (-30도)
```
동작: S24 스위치 내렸다가 중립 복귀
예상 출력:
  ↺  Yaw -30° (위치: 0.0°)
상태창:
  공구 Yaw: +0.0°
```

## 📡 ROS2 토픽 모니터링

다른 터미널에서 실시간 토픽 확인:

```bash
# 주행 명령 확인
ros2 topic echo /cmd_vel

# X축 위치 확인
ros2 topic echo /joint_2/position

# Y축 위치 확인
ros2 topic echo /joint_3/position

# Z축 위치 확인
ros2 topic echo /joint_4/position

# Yaw 위치 확인
ros2 topic echo /joint_5/position

# 그리퍼 위치 확인
ros2 topic echo /gripper/position

# 트리거 확인
ros2 topic echo /motor_0/vel
```

## 🔍 문제 해결

### CAN 메시지가 안 들어옴

```bash
# CAN 인터페이스 확인
ip link show can0
# UP 상태여야 함

# 비트레이트 확인
ip -details link show can0
# bitrate 250000

# 수동으로 재설정
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 250000
sudo ip link set can0 up
```

### 송신기 연결 안됨 (TX_Connected = 0)

```bash
# candump로 0x2E4 메시지 확인
candump can0 | grep 2E4

# Byte 6, Bit 6이 1이어야 함
# 예: can0  2E4   [8]  00 00 00 00 00 00 40 00
#                                         ^^
#                                         0x40 = 0b01000000
#                                                  ^ Bit 6
```

조치:
- 조종기 전원 확인
- 수신기 연결 확인
- 페어링 다시 시도

### 조이스틱 값이 127에서 안 벗어남

```bash
# candump로 0x1E4 확인
candump can0 | grep 1E4

# 조이스틱 움직일 때 값 변경 확인
```

조치:
- 조이스틱 실제로 움직이기
- 데드존 범위 확인 (127±20)

### 스위치가 계속 실행됨 (엣지 트리거 안됨)

원인: 스위치가 중립으로 복귀하지 않음

조치:
- 스위치를 올리거나 내린 후 **반드시 중립으로 복귀**
- candump로 스위치 비트 확인

### 디버그 출력이 안 보임

```bash
# debug_mode 확인
ros2 param get /iron_md_teleop debug_mode

# true로 설정
ros2 param set /iron_md_teleop debug_mode true
```

## ✅ 테스트 체크리스트

```
연결 확인
[ ] CAN 인터페이스 활성화 (can0)
[ ] Iron-MD 수신기 연결
[ ] candump로 메시지 수신 확인
[ ] TX_Connected = 1

조이스틱 테스트
[ ] AN3: 주행 전후진 (linear velocity)
[ ] AN1: X축 이동 (joint_2)
[ ] AN2: Y축 이동 (joint_3)
[ ] 데드존 동작 확인 (127±20)

모드 전환 테스트
[ ] S19: Remote 모드
[ ] S20: Automatic 모드

횡이동 테스트
[ ] S17: +50mm (1회 실행)
[ ] S18: -50mm (1회 실행)
[ ] 중립 복귀 확인

작업 시퀀스 테스트
[ ] S21: Z하강 + 그리퍼 닫기
[ ] S22: 트리거 + 그리퍼 열기 + Z상승
[ ] 시퀀스 순서 확인

Yaw 회전 테스트
[ ] S23: +30도
[ ] S24: -30도

안전 기능 테스트
[ ] Emergency Stop 동작
[ ] TX 연결 끊김 감지
[ ] 비상 정지 해제

디버그 출력 확인
[ ] 1초마다 상태 출력
[ ] 조이스틱 값 실시간 표시
[ ] 위치 정보 누적 확인
[ ] 스위치 상태 표시
```

## 📝 테스트 결과 기록

```
날짜: ___________
테스터: ___________

조이스틱 동작:
- AN3 (주행): [ ] OK  [ ] NG  비고: ___________
- AN1 (X축):  [ ] OK  [ ] NG  비고: ___________
- AN2 (Y축):  [ ] OK  [ ] NG  비고: ___________

스위치 동작:
- S19-S20 (모드):    [ ] OK  [ ] NG  비고: ___________
- S17-S18 (횡이동):  [ ] OK  [ ] NG  비고: ___________
- S21-S22 (작업):    [ ] OK  [ ] NG  비고: ___________
- S23-S24 (Yaw):     [ ] OK  [ ] NG  비고: ___________

안전 기능:
- 비상 정지: [ ] OK  [ ] NG  비고: ___________
- TX 감시:   [ ] OK  [ ] NG  비고: ___________

종합 평가: [ ] 통과  [ ] 불합격
```

## 🚀 실제 장비 배포 시

테스트 완료 후 실제 장비에 배포할 때:

1. **CAN 인터페이스 변경**
   ```yaml
   # config/params.yaml 수정
   can_interface: 'can2'  # can0 → can2
   ```

2. **디버그 모드 비활성화**
   ```yaml
   debug_mode: false  # true → false
   ```

3. **빌드 및 배포**
   ```bash
   colcon build --packages-select rebar_control
   source install/setup.bash
   ```

4. **전체 시스템 실행**
   ```bash
   ros2 launch rebar_control full_system.launch.py
   ```
