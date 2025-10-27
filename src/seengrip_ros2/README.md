# Seengrip ROS2 Driver

ROS2 Humble driver for **Seengrip Optimum Gripper** via Modbus RTU (RS485).

## Features

- ✅ ROS2 Humble compatible
- ✅ Modbus RTU communication
- ✅ Position control (0-2000 or 0.0-1.0 normalized)
- ✅ Speed control
- ✅ Home command
- ✅ Real-time status publishing
- ✅ Fault monitoring

## Hardware Setup

### Connections
- **Device**: Seengrip Optimum Gripper
- **Communication**: RS485 (Modbus RTU)
- **Adapter**: USB2RS485
- **Default Port**: `/dev/ttyACM0` or `/dev/ttyUSB0`
- **Baudrate**: 115200
- **Slave ID**: 1

### Permissions
```bash
sudo chmod 666 /dev/ttyACM0
# or add user to dialout group
sudo usermod -a -G dialout $USER
# then logout and login again
```

## Installation

```bash
cd ~/ros2_ws/src
# Package already in workspace
cd ~/ros2_ws
colcon build --packages-select seengrip_ros2
source install/setup.bash
```

## Usage

### Basic Launch

```bash
ros2 launch seengrip_ros2 seengrip.launch.py
```

### Custom Serial Port

```bash
ros2 launch seengrip_ros2 seengrip.launch.py serial_port:=/dev/ttyUSB0
```

### Control Commands

#### Set Position (Normalized 0.0-1.0)
```bash
# Open gripper (0.0)
ros2 topic pub /gripper/position std_msgs/msg/Float32 "data: 0.0"

# Close gripper (1.0)
ros2 topic pub /gripper/position std_msgs/msg/Float32 "data: 1.0"

# Half closed (0.5)
ros2 topic pub /gripper/position std_msgs/msg/Float32 "data: 0.5"
```

#### Command Gripper
```bash
# Home gripper
ros2 topic pub /gripper/command std_msgs/msg/Int32 "data: 3"

# Quick open (custom command 100)
ros2 topic pub /gripper/command std_msgs/msg/Int32 "data: 100"

# Quick close (custom command 101)
ros2 topic pub /gripper/command std_msgs/msg/Int32 "data: 101"

# Stop
ros2 topic pub /gripper/command std_msgs/msg/Int32 "data: 1"
```

### Monitor Status

```bash
# Watch gripper position
ros2 topic echo /gripper/joint_states

# Watch faults
ros2 topic echo /gripper/fault
```

## Topics

### Subscribed
- `/gripper/position` (std_msgs/Float32): Target position 0.0-1.0
- `/gripper/command` (std_msgs/Int32): Direct commands

### Published
- `/gripper/joint_states` (sensor_msgs/JointState): Current position @10Hz
- `/gripper/fault` (std_msgs/Int32): Fault status

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `serial_port` | string | `/dev/ttyACM0` | Serial port device |
| `baudrate` | int | 115200 | Serial baudrate |
| `slave_id` | int | 1 | Modbus slave ID |
| `default_speed` | int | 500 | Default gripper speed (0-1000) |
| `open_position` | int | 0 | Fully open position |
| `close_position` | int | 2000 | Fully closed position |

## Commands Reference

| Command | Value | Description |
|---------|-------|-------------|
| RESET | 0 | Reset gripper |
| STOP | 1 | Stop motion |
| HOME | 3 | Home gripper |
| OPEN | 100 | Quick open (custom) |
| CLOSE | 101 | Quick close (custom) |

## Troubleshooting

### Cannot open serial port
```bash
# Check port exists
ls -l /dev/ttyACM* /dev/ttyUSB*

# Check permissions
sudo chmod 666 /dev/ttyACM0

# Or add user to dialout group (permanent)
sudo usermod -a -G dialout $USER
```

### No response from gripper
1. Check physical RS485 connections
2. Verify baudrate (115200)
3. Verify slave ID (default: 1)
4. Check power supply to gripper

### Gripper doesn't move
1. Run home command first: `ros2 topic pub /gripper/command std_msgs/msg/Int32 "data: 3"`
2. Wait 3 seconds for homing to complete
3. Then send position commands

## Integration Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32

class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller')
        self.pos_pub = self.create_publisher(Float32, '/gripper/position', 10)
        self.cmd_pub = self.create_publisher(Int32, '/gripper/command', 10)
    
    def home(self):
        msg = Int32()
        msg.data = 3  # HOME
        self.cmd_pub.publish(msg)
    
    def open(self):
        msg = Float32()
        msg.data = 0.0
        self.pos_pub.publish(msg)
    
    def close(self):
        msg = Float32()
        msg.data = 1.0
        self.pos_pub.publish(msg)
```

## License

MIT
