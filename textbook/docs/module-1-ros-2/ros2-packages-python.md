---
title: ROS2 Python Package Development
sidebar_position: 3
---

# ROS2 Python Package Development

## Introduction

ROS2 packages are the fundamental building blocks of ROS2-based applications. They provide a standardized way to organize code, data, and configuration files to promote reusability and maintainability. In the context of humanoid robotics, packages allow for modular development of complex systems where different components—such as perception, control, and actuation—can be developed and maintained independently.

A ROS2 package typically contains:

- **Nodes**: Executable programs that perform specific functions
- **Message definitions**: Custom data structures for communication
- **Launch files**: Configuration for starting multiple nodes simultaneously
- **Configuration files**: Parameters and settings for different environments
- **Tests**: Unit and integration tests to ensure reliability
- **Documentation**: Usage instructions and API documentation

In humanoid robotics applications, packages might include modules for joint control, sensor processing, walking pattern generation, or high-level behavior management. This modular approach allows teams to develop different subsystems in parallel and integrate them seamlessly.

**Key Terms:**
- **Package**: A filesystem organization of ROS2 code
- **ament**: The build system used by ROS2 for package management
- **colcon**: The meta-build tool that orchestrates the build process

## Package Structure and Organization for Humanoid Robotics

A well-structured ROS2 package follows a standard organization that promotes consistency and maintainability. For humanoid robotics applications, this structure becomes particularly important as these systems often involve multiple complex subsystems.

### Basic Package Structure

```
my_humanoid_package/
├── CMakeLists.txt          # Build configuration (if using C++)
├── package.xml             # Package metadata and dependencies
├── setup.py                # Python package configuration
├── setup.cfg               # Installation configuration
├── resource/               # Resource files (meshes, textures)
├── launch/                 # Launch files for different configurations
├── config/                 # Parameter files
├── src/                    # Python source files
│   └── my_humanoid_package/
│       ├── __init__.py
│       ├── joint_controller.py
│       └── sensor_processor.py
├── test/                   # Test files
└── README.md               # Package documentation
```

### Humanoid-Specific Considerations

When organizing packages for humanoid robotics, consider the following principles:

1. **Functional Decomposition**: Group related functionality together (e.g., all walking-related code in a walking package)
2. **Hardware Abstraction**: Separate hardware-specific code from algorithmic code
3. **Safety Boundaries**: Critical safety functions should be in separate, well-tested packages
4. **Reusability**: Design packages to be reusable across different humanoid platforms

## Package Dependencies and Setup Files

Managing dependencies is crucial in complex humanoid robotics systems where multiple packages need to work together. ROS2 uses the `package.xml` file to declare dependencies and the `setup.py` file to specify Python-specific requirements.

### package.xml Structure

The `package.xml` file contains metadata about the package and its dependencies:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>humanoid_control</name>
  <version>1.0.0</version>
  <description>Humanoid robot control package</description>
  <maintainer email="developer@example.com">Robotics Developer</maintainer>
  <license>Apache-2.0</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>message_runtime</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### setup.py Configuration

The `setup.py` file configures the Python package and specifies entry points:

```python
from setuptools import setup
from glob import glob
import os

package_name = 'humanoid_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robotics Developer',
    maintainer_email='developer@example.com',
    description='Humanoid robot control package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_controller = humanoid_control.joint_controller:main',
            'walking_controller = humanoid_control.walking_controller:main',
        ],
    },
)
```

## Python Code Examples for Package Creation

Here's an example of a simple humanoid robot joint controller package:

### Joint Controller Node

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import math

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')

        # Joint state publisher
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Joint command subscriber
        self.cmd_sub = self.create_subscription(
            Float64MultiArray,
            'joint_commands',
            self.joint_cmd_callback,
            10
        )

        # Timer for joint state publishing
        self.timer = self.create_timer(0.05, self.publish_joint_states)  # 20Hz

        # Initialize joint positions
        self.joint_positions = [0.0] * 12  # 12 joints for example humanoid
        self.joint_names = [
            'head_pan', 'head_tilt',
            'left_shoulder', 'left_elbow', 'left_wrist',
            'right_shoulder', 'right_elbow', 'right_wrist',
            'left_hip', 'left_knee', 'left_ankle',
            'right_hip', 'right_knee', 'right_ankle'
        ]

        self.get_logger().info('Joint Controller initialized')

    def joint_cmd_callback(self, msg):
        """Handle joint command messages"""
        if len(msg.data) == len(self.joint_positions):
            self.joint_positions = list(msg.data)
        else:
            self.get_logger().warn(f'Command has {len(msg.data)} joints, expected {len(self.joint_positions)}')

    def publish_joint_states(self):
        """Publish current joint states"""
        msg = JointState()
        msg.name = self.joint_names
        msg.position = self.joint_positions
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Humanoid-Specific Package Examples

In humanoid robotics, packages often need to handle complex interactions between multiple subsystems. Here's an example of a walking pattern generation package:

### Walking Controller Package

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Bool
import numpy as np

class WalkingController(Node):
    def __init__(self):
        super().__init__('walking_controller')

        # Command subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.walk_enable_sub = self.create_subscription(
            Bool,
            'walk_enable',
            self.walk_enable_callback,
            10
        )

        # Publishers for joint commands
        self.trajectory_pub = self.create_publisher(Float64MultiArray, 'joint_trajectory', 10)

        # Walking parameters
        self.is_walking = False
        self.step_height = 0.05  # 5cm step height
        self.step_length = 0.3   # 30cm step length
        self.step_duration = 1.0 # 1 second per step

        self.get_logger().info('Walking Controller initialized')

    def cmd_vel_callback(self, msg):
        """Handle walking velocity commands"""
        if self.is_walking:
            # Generate walking pattern based on velocity
            self.generate_walking_pattern(msg.linear.x, msg.angular.z)

    def walk_enable_callback(self, msg):
        """Enable/disable walking"""
        self.is_walking = msg.data
        if not self.is_walking:
            self.stop_walking()

    def generate_walking_pattern(self, linear_vel, angular_vel):
        """Generate walking pattern based on desired velocity"""
        # Simplified walking pattern generation
        # In a real implementation, this would use inverse kinematics
        # and dynamic balance control
        pass

    def stop_walking(self):
        """Stop current walking motion"""
        # Publish zero joint commands to stop
        stop_msg = Float64MultiArray()
        stop_msg.data = [0.0] * 14  # 14 joints for humanoid
        self.trajectory_pub.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)
    node = WalkingController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## Package Structure Diagram

The following diagram illustrates the standard structure of a ROS2 package for humanoid robotics:

```
                    ROS2 Package Structure
                    ======================

    my_humanoid_package/                    # Package root directory
    ├── CMakeLists.txt                      # Build configuration
    ├── package.xml                         # Package metadata/dependencies
    ├── setup.py                           # Python package configuration
    ├── setup.cfg                          # Installation configuration
    ├── resource/                          # Resource files (meshes, etc.)
    │   └── my_humanoid_package            # Resource files by package name
    ├── launch/                            # Launch files for different configs
    │   ├── head_controller.launch.py      # Launch head control system
    │   ├── walking_controller.launch.py   # Launch walking control system
    │   └── full_body.launch.py            # Launch complete robot control
    ├── config/                            # Parameter files
    │   ├── head_params.yaml               # Head control parameters
    │   ├── walking_params.yaml            # Walking control parameters
    │   └── joint_limits.yaml              # Joint limit definitions
    ├── src/                               # Source code directory
    │   └── my_humanoid_package/           # Python package directory
    │       ├── __init__.py                # Python package initialization
    │       ├── joint_controller.py        # Joint control implementation
    │       ├── walking_controller.py      # Walking pattern generation
    │       └── sensor_processor.py        # Sensor data processing
    ├── test/                              # Test files
    │   ├── test_joint_controller.py       # Unit tests for joint controller
    │   └── test_walking_controller.py     # Unit tests for walking controller
    └── README.md                          # Package documentation

Key Components:
- package.xml: Declares dependencies and package metadata
- setup.py: Configures Python package installation
- launch/: Contains launch files to start multiple nodes
- config/: Stores parameter configurations
- src/: Contains the actual Python source code
```

This structure promotes modularity and reusability in humanoid robotics applications.

## Hands-on Exercise: Creating a Humanoid Robot Package

### Exercise Objective
Create a ROS2 package for a simple humanoid robot head controller that can pan and tilt to follow objects.

### Implementation Steps

1. **Create the package structure** using `ros2 pkg create --build-type ament_python head_tracker`
2. **Implement a node** that subscribes to object detection messages and controls head movement
3. **Create a launch file** to start your node with appropriate parameters
4. **Test your package** in simulation or on a physical robot

### Requirements for Your Package
- Use appropriate message types for object detection and joint control
- Include parameter configuration for head movement limits
- Implement safety checks to prevent dangerous head movements
- Add logging for debugging purposes

### Sample Solution Approach
Consider how you would structure your package to handle:
- Object detection message processing
- Head movement control with velocity limiting
- Joint limit checking for safety
- Parameter configuration for different robot platforms

## Summary

ROS2 package development is fundamental to building modular, maintainable humanoid robotics applications. By following proper package structure and dependency management practices, you can create reusable components that integrate seamlessly into larger humanoid robot systems.

The modular approach enables teams to develop different subsystems in parallel and promotes code reuse across different humanoid platforms. Proper package organization also facilitates testing, debugging, and maintenance of complex humanoid robotics applications.

---

*This section covered the fundamentals of ROS2 package development with specific focus on humanoid robotics applications. The concepts learned here will be essential when we explore Digital Twin implementations in Module 2 and Isaac Sim in Module 3, where similar package structures are used for simulation environments.*

*Cross-reference: The package development principles covered in this section form the foundation for creating simulation packages in upcoming modules.*