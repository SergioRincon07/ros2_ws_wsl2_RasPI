# ROS2 Cross-Distro Communication Demo

This project demonstrates communication between a Linux PC and a Raspberry Pi using ROS2 Humble. It includes a publisher node running on the PC and a subscriber node running on the Raspberry Pi, utilizing a custom message type.

## Project Structure

```
ros2_ws_wsl2_RasPI
├── src
│   ├── cross_distro_demo
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   ├── setup.py
│   │   ├── cross_distro_demo
│   │   │   ├── __init__.py
│   │   │   ├── pc_publisher.py
│   │   │   ├── pi_subscriber.py
│   │   │   └── message_relay.py
│   │   ├── launch
│   │   │   ├── pc_launch.py
│   │   │   └── pi_launch.py
│   │   └── config
│   │       └── params.yaml
│   └── custom_interfaces
│       ├── package.xml
│       ├── CMakeLists.txt
│       └── msg
│           └── CrossDistroMessage.msg
├── install
├── build
├── log
└── README.md
```

## Setup Instructions

1. **Install ROS2 Humble** on both your Linux PC and Raspberry Pi. Follow the official installation guide for your respective operating systems.

2. **Create a ROS2 workspace** on both devices:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```

3. **Clone this repository** into the `src` directory of your workspace:
   ```bash
   git clone <repository-url> cross_distro_demo
   ```

4. **Build the workspace**:
   ```bash
   cd ~/ros2_ws
   colcon build
   ```

5. **Source the setup files**:
   ```bash
   source install/setup.bash
   ```

## Usage

### On the PC

1. Launch the publisher node:
   ```bash
   ros2 launch cross_distro_demo pc_launch.py
   ```

### On the Raspberry Pi

1. Launch the subscriber node:
   ```bash
   ros2 launch cross_distro_demo pi_launch.py
   ```

## Custom Message

The project uses a custom message type defined in `src/custom_interfaces/msg/CrossDistroMessage.msg`. Ensure that both devices have the custom message built and sourced.

## Notes

- Ensure that both devices are on the same network and can communicate with each other.
- Adjust QoS settings in `params.yaml` if necessary to optimize communication based on your network conditions.