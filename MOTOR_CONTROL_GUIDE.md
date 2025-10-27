# Pololu & Seengrip Gripper 제어 가이드

## 개요
ROS2 워크스페이스에서 Pololu 리니어 액추에이터와 Seengrip Gripper를 제어하는 방법

---

## 1. Pololu 리니어 액추에이터 (LM2219F-6024)

### 하드웨어 연결
- **장치**: Pololu Simple Motor Controller 18v7
- **통신 방식**: USB 직접 연결
- **포트**: `/dev/ttyACM0`
- **전원**: 24V
- **제어 방식**: SmcCmd (USB를 통한 직접 제어)

### 설정 확인

#### 1) SmcCenter GUI 실행
```bash
cd /home/test/ros2_ws/src/smc_linux
mono SmcCenter &
```

#### 2) 필수 설정 항목
SmcCenter GUI에서 다음 설정을 확인/변경:

**Input Settings 탭:**
- Input mode: `Serial/USB`
- Mixing mode: `None`

**Motor Settings 탭:**
- Max. speed: `3200` (Forward/Reverse)
- Max. acceleration: `1000` 이상 (빠른 가속을 원할 경우)
- Max. deceleration: `1000` 이상

**Advanced Settings 탭:**
- Safe start 비활성화 체크

설정 후 반드시 **Apply Settings** 클릭!

### 사용법

#### 1) Kill switch 해제
```bash
cd /home/test/ros2_ws/src/smc_linux
./SmcCmd --resume
```

#### 2) 왕복 운동 테스트 실행
```bash
cd /home/test/ros2_ws
python3 test_pololu_smccmd.py
```

#### 3) 테스트 코드 동작
- **전진**: 1초 동안 100% 속도 (3200)
- **정지**: 0.3초
- **후진**: 1초 동안 100% 속도 (-3200)
- **정지**: 0.3초
- 무한 반복

#### 4) 중지
```
Ctrl + C
```

### 수동 제어 명령

#### 속도 설정
```bash
cd /home/test/ros2_ws/src/smc_linux

# 전진 (50% 속도)
./SmcCmd --speed 1600

# 후진 (50% 속도)
./SmcCmd --speed -1600

# 정지
./SmcCmd --speed 0

# 완전 정지
./SmcCmd --stop
```

#### 상태 확인
```bash
./SmcCmd --status
```

---

## 2. Seengrip Gripper

### 하드웨어 연결
- **장치**: Seengrip Optimum Gripper
- **통신 방식**: RS485 (Modbus RTU)
- **어댑터**: USB2RS485
- **포트**: `/dev/ttyUSB0`
- **Baudrate**: `115200`
- **Device ID**: `1`

### 필요한 파일
```bash
cd /home/test/ros2_ws/src
# Navifra.zip에서 추출한 파일들:
# - seengrip_code.py (레지스터 정의)
# - simplemodbus.py (Modbus 통신 라이브러리)
```

### 사용법

#### 1) 시리얼 포트 권한 설정
```bash
sudo chmod 666 /dev/ttyUSB0
```

#### 2) 왕복 운동 테스트 실행
```bash
cd /home/test/ros2_ws
python3 test_seengrip_simplemodbus.py
```

#### 3) 테스트 코드 동작
- **Homing**: 초기 위치로 이동 (3초)
- **Close**: Position 2000으로 닫기 (2초)
- **Open**: Position 0으로 열기 (2초)
- 무한 반복

#### 4) 중지
```
Ctrl + C
```

### Position 범위
- **0**: 완전히 열림
- **2000**: 닫힘 (조정 가능)
- **속도**: 0~1000 (기본값: 500)

---

## 3. 동시 제어 시 주의사항

### 포트 구분
- **Pololu**: `/dev/ttyACM0` (USB 직접)
- **Seengrip**: `/dev/ttyUSB0` (RS485-USB)

### 테스트 전환
Pololu → Seengrip으로 전환:
```bash
pkill -f test_pololu
sleep 1
python3 test_seengrip_simplemodbus.py
```

Seengrip → Pololu로 전환:
```bash
pkill -f test_seengrip
sleep 1
cd /home/test/ros2_ws/src/smc_linux && ./SmcCmd --resume
cd /home/test/ros2_ws && python3 test_pololu_smccmd.py
```

---

## 4. 파일 위치

### Pololu 관련
```
/home/test/ros2_ws/
├── src/smc_linux/
│   ├── SmcCmd          # 명령줄 도구
│   └── SmcCenter       # GUI 도구
└── test_pololu_smccmd.py  # 왕복 테스트 코드
```

### Seengrip 관련
```
/home/test/ros2_ws/
├── src/
│   ├── seengrip_code.py        # 레지스터 정의
│   ├── simplemodbus.py         # Modbus 라이브러리
│   └── Navifra.zip            # 원본 파일
└── test_seengrip_simplemodbus.py  # 왕복 테스트 코드
```

---

## 5. 트러블슈팅

### Pololu 문제

#### "Limit/kill switch active" 에러
```bash
cd /home/test/ros2_ws/src/smc_linux
./SmcCmd --resume
```

#### 모터가 안 움직임
1. SmcCenter에서 Input mode가 `Serial/USB`인지 확인
2. Safe start가 비활성화되어 있는지 확인
3. Max acceleration/deceleration 값 확인

#### VIN 에러
- 24V 전원이 연결되어 있는지 확인
- `./SmcCmd --status`로 VIN 전압 확인 (24000mV 근처여야 함)

### Seengrip 문제

#### "Comm Error Code -2006" (응답 없음)
- Gripper 전원이 켜져 있는지 확인
- RS485 연결 확인 (A, B 단자)
- 포트가 `/dev/ttyUSB0`인지 확인

#### 잘못된 포트
```bash
ls -l /dev/ttyUSB* /dev/ttyACM*
```
위 명령으로 포트 확인

---

## 6. 코드 커스터마이징

### Pololu 속도/시간 변경
`test_pololu_smccmd.py` 파일 수정:
```python
# Forward at 100% speed
set_speed(3200)    # 속도 변경: 0~3200
time.sleep(1.0)    # 전진 시간 변경

# Stop
set_speed(0)
time.sleep(0.3)    # 정지 시간 변경
```

### Seengrip Position/속도 변경
`test_seengrip_simplemodbus.py` 파일 수정:
```python
# Close gripper
mb.WriteReg(1, MB_GOAL_SPD, 500)   # 속도: 0~1000
mb.WriteReg(1, MB_GOAL_POS, 2000)  # 위치: 0~2000 이상
time.sleep(2)  # 동작 시간

# Open gripper  
mb.WriteReg(1, MB_GOAL_SPD, 500)   # 속도
mb.WriteReg(1, MB_GOAL_POS, 0)     # 위치: 0=완전 열림
time.sleep(2)  # 동작 시간
```

---

## 7. 요약

### Pololu 빠른 시작
```bash
cd /home/test/ros2_ws/src/smc_linux
./SmcCmd --resume
cd /home/test/ros2_ws
python3 test_pololu_smccmd.py
```

### Seengrip 빠른 시작
```bash
sudo chmod 666 /dev/ttyUSB0
cd /home/test/ros2_ws
python3 test_seengrip_simplemodbus.py
```

---

**작성일**: 2025-10-24  
**ROS2**: Humble  
**워크스페이스**: `/home/test/ros2_ws`
