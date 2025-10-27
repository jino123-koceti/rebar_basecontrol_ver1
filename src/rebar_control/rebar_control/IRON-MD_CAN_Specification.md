# IRON-MD_일반형_CAN_8Char_Specification_20240329

*PDF 파일에서 자동 변환됨: IRON-MD_일반형_CAN_8Char_Specification_20240329.pdf*

---

## 페이지 1

IRON-MD수신기7PinConnector IRON-MD송신기측7PinConnector
6 : USB-DN(백색)
7 : 0V(흑색)/GND(초록)
1 2 6 : RS232_TX (Blue) -수신기TX 5 : CAN-L/RS232RX(보라)
7 : RS232_RX (Purple) -수신기RX
1 : USB-5V_in(빨강)
3 4 5 5 : Power+(24V) (Red) 4 : POWER 24V_in(빨강)
4 : Power GND(0V) (Black) 2 : USB-DP(노랑)
3 : NC 3 : CAN-H/RS232 TX(청색)
6 7 2 : CAN_H (Green)
Connector SF1212/S7
1 : CAN_L (White)
Wireless Remote Control 20240329
Name DAS_99_MD
CAN Identifier (Hex) 0x1E4
Data Driection TX data
Baudrate 125/250/500 [Kbps]
Repetition Rate (msec) 50
Byte Bit Name Range Description
1 1~8 AN1 255~0 Joystick 1 1
2 1~8 AN2 255~0 Joystick 2 2
3 1~8 AN3 255~0 Joystick 3 3
4 1~8 AN4 255~0 Joystick 4 4
5 - -- not use
6 - -- not use
7 - -- not use
8 - -- not use

### 표 (페이지 1)

**표 1:**

| Wireless Remote Control 20240329 |  |  |  |  |  |
| --- | --- | --- | --- | --- | --- |
| Name |  |  | DAS_99_MD |  |  |
| CAN Identifier (Hex) |  |  | 0x1E4 |  |  |
| Data Driection |  |  | TX data |  |  |
| Baudrate |  |  | 125/250/500 [Kbps] |  |  |
| Repetition Rate (msec) |  |  | 50 |  |  |
| Byte | Bit | Name | Range | Description |  |
| 1 | 1~8 | AN1 | 255~0 | Joystick 1 | 1 |
| 2 | 1~8 | AN2 | 255~0 | Joystick 2 | 2 |
| 3 | 1~8 | AN3 | 255~0 | Joystick 3 | 3 |
| 4 | 1~8 | AN4 | 255~0 | Joystick 4 | 4 |
| 5 |  | - | -- | not use |  |
| 6 |  | - | -- | not use |  |
| 7 |  | - | -- | not use |  |
| 8 |  | - | -- | not use |  |

---

## 페이지 2

Wireless Remote Control 20240329
Name DAS_99_MD
CAN Identifier (Hex) 0x2E4
Data Driection TX data
Baudrate 125/250/500 [Kbps]
Repetition Rate (msec) 50
Byte Bit Name Range Description 참고
1[LSB] Start key Start Key 16
2 Power Key power on key 15
3 S13 Engine Start 13
4 S14 Engine Stop 14
1
5 Not use not use 99
6 Not use not use 99
7 Emergency Stop 0 : 비상 정지, 1 : 비상정지 해제 26
8[MSB] Emergency Stop 1 : 비상 정지, 0 : 비상정지 해제 12
1[LSB] S06 Switch S06 6
2 S07 Switch S07 7
3 Not use 사용 안함
4 Not use 사용 안함
2
5 S02 Switch S02 2
6 S03 Switch S03 3
7 S01 Switch S01 1
8[MSB] S00 Switch S00 0
1[LSB] S08 Switch S08 8
2 S09 Switch S09 9
3 Not use 사용 안함
4 Not use 사용 안함
3
5 Not use 사용 안함
6 Not use 사용 안함
7 Not use 사용 안함
8[MSB] Not use 사용 안함
1[LSB] S23 Switch S23 23
2 S24 Switch S24 24
3 S21 Switch S21 21
4 S22 Switch S22 22
4
5 S19 Switch S19 19
6 S20 Switch S20 20
7 S17 Switch S17 17
8[MSB] S18 Switch S18 18
1~4 Run Counter 0~16 수신된 신호의 counter
5 5 Cal FG 0/1 조정 mode 일때 1 / normal mode 0
6~8 Not use 사용 안함
1~8 Not use 사용 안함
6 7 송신기 연결 0/1 1: 송신기와 연결 상태
8 TRX code 0/1 1: Transmitter, 0: Receiver
7 1~8 Serial_no 255~0 기기번호 (참고용)
8 1~8 Device ID 255~0 기기번호 (참고용)

### 표 (페이지 2)

**표 1:**

| Wireless Remote Control 20240329 |  |  |  |  |  |
| --- | --- | --- | --- | --- | --- |
| Name |  |  | DAS_99_MD |  |  |
| CAN Identifier (Hex) |  |  | 0x2E4 |  |  |
| Data Driection |  |  | TX data |  |  |
| Baudrate |  |  | 125/250/500 [Kbps] |  |  |
| Repetition Rate (msec) |  |  | 50 |  |  |
| Byte | Bit | Name | Range | Description | 참고 |
| 1 | 1[LSB] | Start key |  | Start Key | 16 |
|  | 2 | Power Key |  | power on key | 15 |
|  | 3 | S13 |  | Engine Start | 13 |
|  | 4 | S14 |  | Engine Stop | 14 |
|  | 5 | Not use |  | not use | 99 |
|  | 6 | Not use |  | not use | 99 |
|  | 7 | Emergency Stop |  | 0 : 비상 정지, 1 : 비상정지 해제 | 26 |
|  | 8[MSB] | Emergency Stop |  | 1 : 비상 정지, 0 : 비상정지 해제 | 12 |
| 2 | 1[LSB] | S06 |  | Switch S06 | 6 |
|  | 2 | S07 |  | Switch S07 | 7 |
|  | 3 | Not use |  | 사용 안함 |  |
|  | 4 | Not use |  | 사용 안함 |  |
|  | 5 | S02 |  | Switch S02 | 2 |
|  | 6 | S03 |  | Switch S03 | 3 |
|  | 7 | S01 |  | Switch S01 | 1 |
|  | 8[MSB] | S00 |  | Switch S00 | 0 |
| 3 | 1[LSB] | S08 |  | Switch S08 | 8 |
|  | 2 | S09 |  | Switch S09 | 9 |
|  | 3 | Not use |  | 사용 안함 |  |
|  | 4 | Not use |  | 사용 안함 |  |
|  | 5 | Not use |  | 사용 안함 |  |
|  | 6 | Not use |  | 사용 안함 |  |
|  | 7 | Not use |  | 사용 안함 |  |
|  | 8[MSB] | Not use |  | 사용 안함 |  |
| 4 | 1[LSB] | S23 |  | Switch S23 | 23 |
|  | 2 | S24 |  | Switch S24 | 24 |
|  | 3 | S21 |  | Switch S21 | 21 |
|  | 4 | S22 |  | Switch S22 | 22 |
|  | 5 | S19 |  | Switch S19 | 19 |
|  | 6 | S20 |  | Switch S20 | 20 |
|  | 7 | S17 |  | Switch S17 | 17 |
|  | 8[MSB] | S18 |  | Switch S18 | 18 |
| 5 | 1~4 | Run Counter | 0~16 | 수신된 신호의 counter |  |
|  | 5 | Cal FG | 0/1 | 조정 mode 일때 1 / normal mode 0 |  |
|  | 6~8 | Not use |  | 사용 안함 |  |
| 6 | 1~8 | Not use |  | 사용 안함 |  |
|  | 7 | 송신기 연결 | 0/1 | 1: 송신기와 연결 상태 |  |
|  | 8 | TRX code | 0/1 | 1: Transmitter, 0: Receiver |  |
| 7 | 1~8 | Serial_no | 255~0 | 기기번호 (참고용) |  |
| 8 | 1~8 | Device ID | 255~0 | 기기번호 (참고용) |  |

---

## 페이지 3

Wireless Remote Control 20240329
Name DAS_99_MD
CAN Identifier (Hex) 0x764
Data Driection TX data
Baudrate 125/250/500 [Kbps]
Repetition Rate (msec) 300
Byte Bit Name Range Description 참고
1 1~8 Heart bit 0 Heart bit, 항상 0 출력

### 표 (페이지 3)

**표 1:**

| Wireless Remote Control 20240329 |  |  |  |  |  |
| --- | --- | --- | --- | --- | --- |
| Name |  |  | DAS_99_MD |  |  |
| CAN Identifier (Hex) |  |  | 0x764 |  |  |
| Data Driection |  |  | TX data |  |  |
| Baudrate |  |  | 125/250/500 [Kbps] |  |  |
| Repetition Rate (msec) |  |  | 300 |  |  |
| Byte | Bit | Name | Range | Description | 참고 |
| 1 | 1~8 | Heart bit | 0 | Heart bit, 항상 0 출력 |  |

---

## 페이지 4

Wireless Remote Control 20240329
Name DAS_99_MD
CAN Identifier (Hex) 0x364
Data Driection RX data - Option - notuse
Baudrate 125 Kbps
Repetition Rate (msec) >100
Byte Bit Name Range Description 참고
1 1~8 Line Index 0~15 lcd 표시 문자열 라인 번호(12 이상은 무시 됨)
2 1~8 Data Index 0~10 lcd 문자열 위치번호 (8 이상은 무시 됨)
3 1~8 Data[Index+0]
4 1~8 Data[Index+1]
5 1~8 Data[Index+2]
6 1~8 Data[Index+3]
7 1~8 Data[Index+4]
8 1~8 Data[Index+5]
LCD 표시창 8char
-Page전환: S13 사용
P0 Page1[line 0~line3]
ABCD_001 P1 Page2[line 4~line7]
WT : 250kg P2
ABCD_001
Page3[line 8~line11]
L 120.7m
ABCD_002
ABCD_001
ABCD_003 Page표시
ABCD_003
ABCD_002
ABCD_004
ABCD_003
ABCD_004

### 표 (페이지 4)

**표 1:**

| Wireless Remote Control 20240329 |  |  |  |  |  |
| --- | --- | --- | --- | --- | --- |
| Name |  |  | DAS_99_MD |  |  |
| CAN Identifier (Hex) |  |  | 0x364 |  |  |
| Data Driection |  |  | RX data - Option - notuse |  |  |
| Baudrate |  |  | 125 Kbps |  |  |
| Repetition Rate (msec) |  |  | >100 |  |  |
| Byte | Bit | Name | Range | Description | 참고 |
| 1 | 1~8 | Line Index | 0~15 | lcd 표시 문자열 라인 번호(12 이상은 무시 됨) |  |
| 2 | 1~8 | Data Index | 0~10 | lcd 문자열 위치번호 (8 이상은 무시 됨) |  |
| 3 | 1~8 | Data[Index+0] |  |  |  |
| 4 | 1~8 | Data[Index+1] |  |  |  |
| 5 | 1~8 | Data[Index+2] |  |  |  |
| 6 | 1~8 | Data[Index+3] |  |  |  |
| 7 | 1~8 | Data[Index+4] |  |  |  |
| 8 | 1~8 | Data[Index+5] |  |  |  |

**표 2:**

| P0
ABCD_001
WT : 250kg
AB
L 120.7m
AB
ABCD_003
AB
AB |  |
| --- | --- |
|  | 01
kg
AB
AB
03
AB
AB |

---

