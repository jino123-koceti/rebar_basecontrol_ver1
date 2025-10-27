# Pololu ROS2 Driver

ROS2 Humble driver for **Pololu Simple Motor Controller** (LM2219F-6024-24V-10mm linear actuator).

Ported from [matwilso/pololu_ros](https://github.com/matwilso/pololu_ros) (ROS1) to ROS2.

## Features

- ✅ ROS2 Humble compatible
- ✅ Daisy chain support (multiple motors on one serial port)
- ✅ Simple Float32 velocity command interface (-1.0 to 1.0)
- ✅ Launch file with configurable parameters
- ✅ Python 3 (no Python 2 dependencies)

## Dependencies

```bash
sudo apt install python3-serial
```

## Installation

```bash
cd ~/ros2_ws/src
# Package already created in workspace
cd ~/ros2_ws
colcon build --packages-select pololu_ros2
source install/setup.bash
```

## Usage

### Basic Launch

```bash
ros2 launch pololu_ros2 pololu.launch.py
```

### Custom Serial Port

```bash
ros2 launch pololu_ros2 pololu.launch.py serial_port:=/dev/ttyUSB1
```

### Send Velocity Commands

```bash
# Motor 0 forward at 50% speed
ros2 topic pub /motor_0/vel std_msgs/msg/Float32 "data: 0.5"

# Motor 1 backward at 30% speed
ros2 topic pub /motor_1/vel std_msgs/msg/Float32 "data: -0.3"

# Stop motor 2
ros2 topic pub /motor_2/vel std_msgs/msg/Float32 "data: 0.0"
```

## Configuration

Edit `config/pololu.yaml`:

```yaml
pololu_node:
  ros__parameters:
    serial_port: "/dev/ttyUSB0"
    baudrate: 9600
    motor_ids: [0, 1, 2]  # Device numbers (0-127)
    motor_topics: ["motor_0/vel", "motor_1/vel", "motor_2/vel"]
```

**Important:** Configure motor device numbers using [Pololu Simple Motor Controller software](https://www.pololu.com/docs/0J44/3).

## Topics

### Subscribed

- `/motor_0/vel` (std_msgs/Float32) - Velocity command for motor 0 (-1.0 to 1.0)
- `/motor_1/vel` (std_msgs/Float32) - Velocity command for motor 1 (-1.0 to 1.0)
- `/motor_2/vel` (std_msgs/Float32) - Velocity command for motor 2 (-1.0 to 1.0)

## Protocol

Uses **Pololu Protocol** for serial communication:
- Baudrate: 9600 (default)
- Speed range: -3200 to 3200 (internal)
- ROS interface: -1.0 to 1.0 (scaled automatically)

## Troubleshooting

### Permission Denied on /dev/ttyUSB0

```bash
sudo chmod 666 /dev/ttyUSB0
# Or permanently:
sudo usermod -a -G dialout $USER
# Then logout/login
```

### Motor Not Responding

1. Check device number matches configuration
2. Verify serial port connection: `ls -l /dev/ttyUSB*`
3. Test with Pololu software first
4. Check motor error LED status

## License

MIT License (original repository)

## Credits

- Original ROS1 version: [Matthew Wilson](https://github.com/matwilso/pololu_ros)
- ROS2 port: Based on matwilso's work
