---
title: ROS2 Python Package Development
sidebar_position: 4
---


# ROS2 Python Package Development

## Introduction

**ROS2 packages** are the fundamental building blocks of ROS2-based applications that provide a standardized way to organize code, data, and configuration files to promote reusability and maintainability. In the context of humanoid robotics, packages enable modular development of complex systems where different components—such as perception, control, actuation, and safety—can be developed, tested, and maintained independently.

A well-structured ROS2 package contains several key components that work together to create robust humanoid robot systems:

- **Nodes**: Executable programs that perform specific functions and communicate with other components
- **Message definitions**: Custom data structures for communication between nodes
- **Launch files**: Configuration for starting multiple nodes simultaneously with coordinated parameters
- **Configuration files**: Parameters and settings for different operational environments and robot configurations
- **Tests**: Unit and integration tests to ensure reliability and safety in humanoid applications
- **Documentation**: Usage instructions, API documentation, and safety guidelines

In humanoid robotics applications, packages serve as modular subsystems that can include specialized modules for joint control, sensor processing, walking pattern generation, balance control, manipulation, or high-level behavior management. This modular approach allows development teams to work on different subsystems in parallel while ensuring seamless integration and maintaining safety standards.

The **rclpy** Python client library provides the interface for creating ROS2 nodes in Python, offering the same capabilities as the C++ equivalent while leveraging Python's ease of development and extensive ecosystem of scientific computing libraries.

**Key Terms:**
- **Package**: A filesystem organization of ROS2 code that encapsulates related functionality
- **ament**: The build system used by ROS2 for package management and dependency resolution
- **colcon**: The meta-build tool that orchestrates the build process across multiple packages
- **rclpy**: The Python client library for ROS2 that provides the standard interface for Python programs
- **Console scripts**: Executable entry points defined in setup.py that become command-line tools
- **Resource index**: The system that tracks available packages and their resources

## Advanced Package Structure and Organization for Humanoid Robotics

A well-structured ROS2 package for humanoid robotics follows a comprehensive organization that promotes consistency, maintainability, and safety. For humanoid systems, this structure becomes critically important as these systems involve multiple complex subsystems that must operate safely and reliably.

### Comprehensive Package Structure

```
humanoid_control_package/
├── CMakeLists.txt                        # Build configuration (if using C++ components)
├── package.xml                           # Package metadata and dependencies
├── setup.py                              # Python package configuration
├── setup.cfg                             # Installation configuration
├── pyproject.toml                        # Python project configuration (PEP 621)
├── requirements.txt                      # Python dependencies beyond setup.py
├── resource/                             # Resource files (meshes, textures, models)
│   └── humanoid_control_package          # Resource files by package name
├── launch/                               # Launch files for different configurations
│   ├── head_controller.launch.py         # Launch head control system
│   ├── walking_controller.launch.py      # Launch walking control system
│   ├── manipulation_controller.launch.py # Launch manipulation system
│   ├── full_body.launch.py               # Launch complete robot control
│   └── safety_monitor.launch.py          # Launch safety monitoring system
├── config/                               # Parameter files for different modes
│   ├── head_params.yaml                  # Head control parameters and limits
│   ├── walking_params.yaml               # Walking control parameters and gait settings
│   ├── manipulation_params.yaml          # Manipulation control parameters
│   ├── joint_limits.yaml                 # Joint limit definitions and safety bounds
│   ├── balance_params.yaml               # Balance control parameters
│   └── safety_params.yaml                # Safety system parameters
├── src/                                  # Python source code directory
│   └── humanoid_control_package/         # Main Python package directory
│       ├── __init__.py                   # Python package initialization
│       ├── controllers/                  # Control algorithms and implementations
│       │   ├── joint_controller.py       # Joint position/velocity control
│       │   ├── walking_controller.py     # Bipedal locomotion control
│       │   ├── balance_controller.py     # Balance and stability control
│       │   └── manipulation_controller.py# Arm and hand manipulation control
│       ├── utils/                        # Utility functions and helpers
│       │   ├── safety_checks.py          # Safety validation functions
│       │   ├── kinematics.py             # Forward/inverse kinematics
│       │   ├── transforms.py             # Coordinate system transformations
│       │   └── logging.py                # Custom logging utilities
│       ├── interfaces/                   # Custom message/service/action definitions
│       │   ├── msg/                      # Custom message definitions
│       │   ├── srv/                      # Custom service definitions
│       │   └── action/                   # Custom action definitions
│       └── nodes/                        # Node implementations
│           ├── head_tracker_node.py      # Head tracking and control node
│           ├── walking_node.py           # Walking pattern generation node
│           ├── safety_monitor_node.py    # Safety monitoring node
│           └── behavior_manager_node.py  # High-level behavior management
├── test/                                 # Test files for validation
│   ├── unit/                             # Unit tests for individual components
│   │   ├── test_joint_controller.py      # Tests for joint controller
│   │   ├── test_kinematics.py            # Tests for kinematics functions
│   │   └── test_safety_checks.py         # Tests for safety functions
│   ├── integration/                      # Integration tests for subsystems
│   │   ├── test_walking_integration.py   # Integration test for walking system
│   │   └── test_full_body_integration.py # Integration test for complete system
│   ├── performance/                      # Performance and stress tests
│   │   ├── test_real_time_performance.py # Real-time performance tests
│   │   └── test_safety_response.py       # Safety system response tests
│   └── fixtures/                         # Test fixtures and mock data
├── scripts/                              # Standalone scripts for utilities
│   ├── calibrate_joints.py               # Joint calibration script
│   ├── check_safety_limits.py            # Safety limit validation script
│   └── generate_urdf.py                  # URDF generation utility
├── docs/                                 # Package documentation
│   ├── api.md                            # API documentation
│   ├── safety_manual.md                  # Safety guidelines and procedures
│   └── troubleshooting.md                # Common issues and solutions
└── README.md                             # Package overview and usage
```

### Humanoid-Specific Organizational Principles

When organizing packages for humanoid robotics, consider these critical principles that ensure safety, reliability, and maintainability:

1. **Functional Decomposition**: Group related functionality together while maintaining clear boundaries between subsystems (e.g., all walking-related code in a walking package, all safety-related code in safety packages)

2. **Hardware Abstraction**: Strictly separate hardware-specific code from algorithmic code using abstraction layers that can be easily swapped for different robot platforms

3. **Safety Boundaries**: Critical safety functions must be in separate, well-tested packages with independent validation and monitoring

4. **Reusability**: Design packages to be reusable across different humanoid platforms with configurable parameters and minimal dependencies

5. **Testability**: Organize code to enable comprehensive unit, integration, and system-level testing of all safety-critical functions

6. **Real-time Considerations**: Structure packages to support real-time performance requirements for control systems

## Package Dependencies and Configuration Management

Managing dependencies in complex humanoid robotics systems requires careful attention to safety, performance, and maintainability. ROS2 uses the `package.xml` file to declare dependencies and the `setup.py` file to specify Python-specific requirements.

### Advanced package.xml Configuration

The `package.xml` file contains comprehensive metadata about the package and its dependencies, including safety and performance considerations:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>humanoid_control</name>
  <version>2.1.0</version>
  <description>Humanoid robot control package with safety monitoring and real-time capabilities</description>
  <maintainer email="safety@robotics-institute.org">Humanoid Robotics Safety Team</maintainer>
  <license>Apache-2.0</license>
  <url type="website">https://robotics-institute.org/humanoid-control</url>
  <author email="dev@robotics-institute.org">Humanoid Development Team</author>

  <buildtool_depend>ament_cmake_python</buildtool_depend>
  <buildtool_depend>ament_copyright</buildtool_depend>
  <buildtool_depend>ament_flake8</buildtool_depend>
  <buildtool_depend>ament_pep257</buildtool_depend>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>
  <depend>control_msgs</depend>
  <depend>trajectory_msgs</depend>

  <exec_depend>python3-numpy</exec_depend>
  <exec_depend>python3-scipy</exec_depend>
  <exec_depend>python3-yaml</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>
  <test_depend>launch_testing</test_depend>
  <test_depend>launch_testing_ament_cmake</test_depend>

  <export>
    <build_type>ament_python</build_type>
    <rosdoc2>config.yaml</rosdoc2>
  </export>
</package>
```

### Comprehensive setup.py Configuration

The `setup.py` file configures the Python package with safety considerations and comprehensive entry points:

```python
from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'humanoid_control'

setup(
    name=package_name,
    version='2.1.0',
    packages=find_packages(exclude=['test*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'models'), glob('models/*.urdf')),
    ],
    install_requires=[
        'setuptools',
        'numpy>=1.19.0',
        'scipy>=1.7.0',
        'pyyaml>=5.4.0',
        'transforms3d>=0.3.1'
    ],
    zip_safe=True,
    maintainer='Humanoid Robotics Safety Team',
    maintainer_email='safety@robotics-institute.org',
    description='Humanoid robot control package with safety monitoring and real-time capabilities',
    license='Apache-2.0',
    tests_require=['pytest', 'pytest-cov', 'pytest-timeout'],
    python_requires='>=3.8',
    entry_points={
        'console_scripts': [
            # Control nodes
            'joint_controller = humanoid_control.nodes.joint_controller_node:main',
            'walking_controller = humanoid_control.nodes.walking_node:main',
            'balance_controller = humanoid_control.nodes.balance_node:main',
            'manipulation_controller = humanoid_control.nodes.manipulation_node:main',

            # Safety and monitoring nodes
            'safety_monitor = humanoid_control.nodes.safety_monitor_node:main',
            'system_diagnostics = humanoid_control.nodes.diagnostics_node:main',

            # Utility scripts
            'calibrate_joints = humanoid_control.scripts.calibrate_joints:main',
            'check_safety_limits = humanoid_control.scripts.check_safety_limits:main',
            'generate_gait = humanoid_control.scripts.generate_gait:main',
        ],
    },
    classifiers=[
        'Development Status :: 5 - Production/Stable',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
        'Topic :: Scientific/Engineering',
        'Topic :: Software Development :: Libraries :: Python Modules',
    ],
    keywords=['ros2', 'humanoid', 'robotics', 'control', 'safety'],
    project_urls={
        'Bug Reports': 'https://github.com/robotics-institute/humanoid-control/issues',
        'Source': 'https://github.com/robotics-institute/humanoid-control',
        'Documentation': 'https://robotics-institute.org/humanoid-control/docs',
    },
)
```

## Advanced Python Code Examples for Humanoid Packages

Here's an example of a comprehensive humanoid robot control package with safety monitoring and real-time capabilities:

### Advanced Joint Controller with Safety Features

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from control_msgs.msg import JointTrajectoryControllerState
import numpy as np
import threading
from typing import List, Dict, Optional
import time

class SafeJointController(Node):
    """
    Advanced joint controller with comprehensive safety features for humanoid robots.
    Includes joint limit checking, velocity limiting, and real-time safety monitoring.
    """

    def __init__(self):
        super().__init__('safe_joint_controller')

        # Safety parameters
        self.joint_limits = {
            'head_pan': {'min': -1.57, 'max': 1.57, 'vel_max': 1.0},
            'head_tilt': {'min': -0.785, 'max': 0.785, 'vel_max': 0.8},
            'left_shoulder': {'min': -2.0, 'max': 2.0, 'vel_max': 2.0},
            'left_elbow': {'min': 0.0, 'max': 2.5, 'vel_max': 2.5},
            # Add more joints as needed
        }

        # Initialize joint states
        self.current_positions = {}
        self.target_positions = {}
        self.last_update_time = self.get_clock().now()

        # Safety monitoring
        self.emergency_stop = False
        self.safety_violation = False
        self.safety_lock = threading.Lock()

        # QoS profiles for different data types
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST
        )

        control_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # Publishers
        self.joint_state_pub = self.create_publisher(
            JointState, 'joint_states', sensor_qos
        )

        self.controller_state_pub = self.create_publisher(
            JointTrajectoryControllerState, 'controller_state', control_qos
        )

        # Subscribers
        self.joint_cmd_sub = self.create_subscription(
            Float64MultiArray,
            'joint_commands',
            self.joint_cmd_callback,
            control_qos
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            'hardware/joint_states',
            self.hardware_joint_state_callback,
            sensor_qos
        )

        # Timer for safety checks and state publishing
        self.control_timer = self.create_timer(0.01, self.control_loop)  # 100Hz
        self.safety_timer = self.create_timer(0.05, self.safety_check)   # 20Hz

        # Initialize joint names
        self.joint_names = list(self.joint_limits.keys())
        self.current_positions = {name: 0.0 for name in self.joint_names}
        self.target_positions = {name: 0.0 for name in self.joint_names}

        self.get_logger().info('Safe Joint Controller initialized with safety monitoring')

    def joint_cmd_callback(self, msg: Float64MultiArray):
        """Handle joint command messages with safety validation"""
        if len(msg.data) != len(self.joint_names):
            self.get_logger().error(
                f'Command has {len(msg.data)} joints, expected {len(self.joint_names)}'
            )
            return

        with self.safety_lock:
            if self.emergency_stop:
                self.get_logger().warn('Joint commands ignored due to emergency stop')
                return

            # Validate and apply commands with safety limits
            for i, joint_name in enumerate(self.joint_names):
                target_pos = float(msg.data[i])

                # Check position limits
                limits = self.joint_limits[joint_name]
                if target_pos < limits['min'] or target_pos > limits['max']:
                    self.get_logger().warn(
                        f'Joint {joint_name} position {target_pos} exceeds limits '
                        f'[{limits["min"]}, {limits["max"]}]'
                    )
                    # Clamp to safe limits
                    target_pos = max(limits['min'], min(limits['max'], target_pos))

                self.target_positions[joint_name] = target_pos

        self.last_update_time = self.get_clock().now()

    def hardware_joint_state_callback(self, msg: JointState):
        """Update current joint states from hardware"""
        for i, name in enumerate(msg.name):
            if name in self.current_positions:
                self.current_positions[name] = msg.position[i]

    def control_loop(self):
        """Main control loop running at 100Hz"""
        if self.emergency_stop:
            # Publish zero commands to stop all joints safely
            self.publish_safe_commands()
            return

        # Calculate control commands with smooth interpolation
        current_time = self.get_clock().now()
        dt = (current_time - self.last_update_time).nanoseconds / 1e9

        with self.safety_lock:
            # Interpolate towards target positions
            commands = Float64MultiArray()
            for joint_name in self.joint_names:
                current_pos = self.current_positions.get(joint_name, 0.0)
                target_pos = self.target_positions[joint_name]

                # Simple PD control with velocity limiting
                error = target_pos - current_pos
                vel_limit = self.joint_limits[joint_name]['vel_max']

                # Limit velocity change
                max_change = vel_limit * 0.01  # 100Hz control
                limited_error = max(-max_change, min(max_change, error))

                new_pos = current_pos + limited_error
                commands.data.append(new_pos)

        # Publish joint commands
        cmd_pub = self.create_publisher(Float64MultiArray, 'hardware/joint_commands', 10)
        cmd_pub.publish(commands)

        # Publish current joint states
        self.publish_joint_states()

    def safety_check(self):
        """Perform safety checks at 20Hz"""
        with self.safety_lock:
            # Check for joint limit violations
            for joint_name, current_pos in self.current_positions.items():
                if joint_name in self.joint_limits:
                    limits = self.joint_limits[joint_name]
                    if (current_pos < limits['min'] - 0.1 or
                        current_pos > limits['max'] + 0.1):
                        self.safety_violation = True
                        self.get_logger().error(
                            f'SAFETY VIOLATION: Joint {joint_name} at position {current_pos} '
                            f'outside safe limits [{limits["min"]}, {limits["max"]}]'
                        )

            # Check for communication timeouts
            time_since_update = self.get_clock().now() - self.last_update_time
            if time_since_update.nanoseconds > 2e9:  # 2 seconds
                self.safety_violation = True
                self.get_logger().error('SAFETY VIOLATION: No joint commands received for 2 seconds')

    def publish_joint_states(self):
        """Publish current joint states"""
        msg = JointState()
        msg.name = list(self.current_positions.keys())
        msg.position = list(self.current_positions.values())
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Calculate velocities approximately
        msg.velocity = [0.0] * len(msg.position)  # Simplified
        msg.effort = [0.0] * len(msg.position)    # Simplified

        self.joint_state_pub.publish(msg)

    def publish_safe_commands(self):
        """Publish safe zero commands to stop all joints"""
        # Implementation would send zero velocity commands to all joints
        pass

def main(args=None):
    rclpy.init(args=args)
    node = SafeJointController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down safe joint controller...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Humanoid-Specific Package Architecture and Design Patterns

Humanoid robotics packages often need to handle complex interactions between multiple subsystems with strict safety and real-time requirements. Here's an example of a comprehensive walking pattern generation package with safety integration:

### Advanced Walking Controller with Safety and Balance

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from geometry_msgs.msg import Twist, Pose, Vector3
from std_msgs.msg import Bool, Float64MultiArray
from sensor_msgs.msg import Imu, JointState
from humanoid_msgs.action import WalkToPose
from builtin_interfaces.msg import Duration as BuiltinDuration
import numpy as np
from scipy import signal
import threading
from typing import Tuple, List, Optional
import math

class AdvancedWalkingController(Node):
    """
    Advanced walking controller for humanoid robots with balance control,
    obstacle avoidance, and comprehensive safety monitoring.
    """

    def __init__(self):
        super().__init__('advanced_walking_controller')

        # Walking parameters
        self.step_height = 0.05  # 5cm step height
        self.step_length = 0.3   # 30cm step length
        self.step_duration = 1.0 # 1 second per step
        self.step_width = 0.2    # 20cm step width (distance between feet)

        # Balance parameters
        self.balance_margin = 0.05  # 5cm safety margin for balance
        self.zmp_tolerance = 0.02   # 2cm tolerance for Zero Moment Point

        # State variables
        self.is_walking = False
        self.is_balanced = True
        self.current_pose = Pose()
        self.imu_data = None
        self.joint_states = None

        # Safety flags
        self.emergency_stop = False
        self.balance_lost = False
        self.obstacle_detected = False

        # Threading lock for safety
        self.safety_lock = threading.Lock()

        # Action server for walking
        self._walk_action_server = ActionServer(
            self,
            WalkToPose,
            'walk_to_pose',
            execute_callback=self.execute_walk_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

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

        # Sensor subscribers for balance and safety
        self.imu_sub = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            10
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        # Publishers
        self.trajectory_pub = self.create_publisher(
            Float64MultiArray, 'joint_trajectory', 10
        )

        self.balance_pub = self.create_publisher(
            Bool, 'balance_status', 10
        )

        # Timers for walking control and safety
        self.walk_timer = self.create_timer(0.02, self.walk_control_loop)  # 50Hz walking control
        self.safety_timer = self.create_timer(0.05, self.safety_monitor)    # 20Hz safety check

        self.get_logger().info('Advanced walking controller initialized with safety systems')

    def goal_callback(self, goal_request):
        """Accept or reject walking goals based on safety status"""
        with self.safety_lock:
            if self.emergency_stop or not self.is_balanced:
                self.get_logger().warn('Rejecting walking goal due to safety conditions')
                return GoalResponse.REJECT
            return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Handle walking goal cancellation"""
        self.get_logger().info('Walking goal cancellation requested')
        return CancelResponse.ACCEPT

    def cmd_vel_callback(self, msg: Twist):
        """Handle velocity commands for walking"""
        if self.is_walking and not self.emergency_stop:
            # Process velocity command for walking
            self.process_velocity_command(msg)

    def walk_enable_callback(self, msg: Bool):
        """Enable or disable walking capability"""
        with self.safety_lock:
            if msg.data:
                if self.is_balanced and not self.emergency_stop:
                    self.is_walking = True
                    self.get_logger().info('Walking enabled')
                else:
                    self.get_logger().warn('Cannot enable walking - safety conditions not met')
            else:
                self.is_walking = False
                self.stop_walking()
                self.get_logger().info('Walking disabled')

    def imu_callback(self, msg: Imu):
        """Process IMU data for balance monitoring"""
        self.imu_data = msg
        # Calculate orientation and acceleration for balance
        orientation = msg.orientation
        # Extract roll, pitch, yaw for balance calculations
        # (simplified - would use proper quaternion to euler conversion)

    def joint_state_callback(self, msg: JointState):
        """Process joint state data"""
        self.joint_states = msg
        # Update current joint positions for walking calculations

    def execute_walk_callback(self, goal_handle):
        """Execute walking action with feedback and safety monitoring"""
        self.get_logger().info('Starting walking action execution')

        target_pose = goal_handle.request.target_pose
        walk_speed = goal_handle.request.speed

        # Initialize feedback and result
        feedback_msg = WalkToPose.Feedback()
        result = WalkToPose.Result()

        try:
            # Generate walking trajectory
            trajectory = self.generate_walking_trajectory(target_pose, walk_speed)

            if not trajectory:
                goal_handle.abort()
                result.success = False
                result.message = 'Failed to generate valid walking trajectory'
                return result

            # Execute walking with safety monitoring
            for step_idx, step in enumerate(trajectory):
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('Walking action canceled by user')
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'Walking action canceled'
                    return result

                # Check safety conditions
                with self.safety_lock:
                    if self.emergency_stop or not self.is_balanced:
                        self.get_logger().error('Safety violation during walking - stopping')
                        goal_handle.abort()
                        result.success = False
                        result.message = 'Safety violation during walking'
                        return result

                # Execute single walking step
                success = self.execute_walking_step(step)

                if not success:
                    goal_handle.abort()
                    result.success = False
                    result.message = 'Failed to execute walking step'
                    return result

                # Publish feedback
                feedback_msg.current_pose = self.current_pose
                feedback_msg.progress = float(step_idx) / len(trajectory)
                feedback_msg.remaining_distance = self.calculate_remaining_distance(target_pose)

                goal_handle.publish_feedback(feedback_msg)

                # Small delay between steps
                time.sleep(0.02)

            # Check if successfully reached target
            if self.is_at_target_pose(target_pose):
                goal_handle.succeed()
                result.success = True
                result.message = 'Successfully walked to target pose'
            else:
                goal_handle.abort()
                result.success = False
                result.message = 'Failed to reach target pose within tolerance'

        except Exception as e:
            self.get_logger().error(f'Error during walking execution: {str(e)}')
            goal_handle.abort()
            result.success = False
            result.message = f'Walking execution failed: {str(e)}'

        return result

    def generate_walking_trajectory(self, target_pose: Pose, speed: float) -> List[Pose]:
        """Generate walking trajectory to reach target pose"""
        # Simplified trajectory generation
        # In practice, this would involve path planning, inverse kinematics,
        # and balance control calculations

        # Calculate number of steps needed
        distance = self.calculate_distance_to_pose(target_pose)
        num_steps = max(1, int(distance / self.step_length))

        trajectory = []
        for i in range(num_steps):
            # Generate intermediate poses along the path
            fraction = float(i + 1) / num_steps
            intermediate_pose = self.interpolate_pose(self.current_pose, target_pose, fraction)
            trajectory.append(intermediate_pose)

        return trajectory

    def execute_walking_step(self, target_pose: Pose) -> bool:
        """Execute a single walking step with balance control"""
        try:
            # Generate joint commands for the walking step
            joint_commands = self.calculate_step_joints(target_pose)

            if joint_commands:
                cmd_msg = Float64MultiArray()
                cmd_msg.data = joint_commands
                self.trajectory_pub.publish(cmd_msg)
                return True
            else:
                return False
        except Exception as e:
            self.get_logger().error(f'Error executing walking step: {str(e)}')
            return False

    def calculate_step_joints(self, target_pose: Pose) -> Optional[List[float]]:
        """Calculate joint positions for a walking step"""
        # This would involve inverse kinematics and balance calculations
        # Simplified for this example
        return [0.0] * 14  # Placeholder for 14 joints

    def calculate_distance_to_pose(self, target_pose: Pose) -> float:
        """Calculate distance to target pose"""
        dx = target_pose.position.x - self.current_pose.position.x
        dy = target_pose.position.y - self.current_pose.position.y
        return math.sqrt(dx*dx + dy*dy)

    def interpolate_pose(self, start_pose: Pose, end_pose: Pose, fraction: float) -> Pose:
        """Interpolate between two poses"""
        result = Pose()
        result.position.x = start_pose.position.x + fraction * (end_pose.position.x - start_pose.position.x)
        result.position.y = start_pose.position.y + fraction * (end_pose.position.y - start_pose.position.y)
        result.position.z = start_pose.position.z + fraction * (end_pose.position.z - start_pose.position.z)
        return result

    def is_at_target_pose(self, target_pose: Pose) -> bool:
        """Check if robot is at target pose within tolerance"""
        tolerance = 0.1  # 10cm tolerance
        distance = self.calculate_distance_to_pose(target_pose)
        return distance <= tolerance

    def calculate_remaining_distance(self, target_pose: Pose) -> float:
        """Calculate remaining distance to target"""
        return self.calculate_distance_to_pose(target_pose)

    def walk_control_loop(self):
        """Main walking control loop running at 50Hz"""
        if not self.is_walking or self.emergency_stop:
            return

        # Implement walking pattern generation and execution
        # This would involve real-time gait generation and balance control
        pass

    def safety_monitor(self):
        """Monitor safety conditions at 20Hz"""
        with self.safety_lock:
            # Check balance based on IMU data
            if self.imu_data:
                # Calculate balance metrics from IMU data
                # Check if robot is within safe balance margins
                pass

            # Check for emergency conditions
            if self.balance_lost or self.obstacle_detected:
                self.emergency_stop = True
                self.stop_walking()
                self.get_logger().error('EMERGENCY STOP: Safety condition violated')

    def stop_walking(self):
        """Safely stop all walking motion"""
        stop_msg = Float64MultiArray()
        stop_msg.data = [0.0] * 14  # Zero all joint commands
        self.trajectory_pub.publish(stop_msg)
        self.is_walking = False

    def process_velocity_command(self, cmd_vel: Twist):
        """Process velocity command for walking"""
        # Convert twist command to walking pattern
        # This would involve gait generation based on desired velocity
        pass

def main(args=None):
    rclpy.init(args=args)
    node = AdvancedWalkingController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down walking controller...')
        node.stop_walking()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Package Development Best Practices and Safety Patterns

The following diagram illustrates the comprehensive architecture of a safe, real-time humanoid robot package:

```
                    Humanoid Robot Package Architecture
                    ===================================

+-------------------+    +-------------------+    +-------------------+
|   External        |    |   Package         |    |   Internal        |
|   Interfaces      |    |   Core            |    |   Components      |
|                   |    |                   |    |                   |
| • ROS2 Topics     |<-->| • Node Base       |<-->| • Control         |
| • ROS2 Services   |    | • Action Servers  |    |   Algorithms      |
| • ROS2 Actions    |    | • Lifecycle Mgmt  |    | • Safety Checks   |
| • Parameter       |    | • QoS Management  |    | • State Machines  |
|   Server          |    | • TF Management   |    | • Logging         |
+-------------------+    +-------------------+    +-------------------+
         |                        |                        |
         | ROS2 Messages          | Internal APIs          | Direct Function
         |----------------------->|----------------------->|----------------->
         | Services               | Component Interfaces   | Calls
         |<-----------------------|<-----------------------|<-----------------

+-------------------+    +-------------------+    +-------------------+
|   Safety &        |    |   Configuration   |    |   Testing &       |
|   Monitoring      |    |   Management      |    |   Validation      |
|                   |    |                   |    |                   |
| • Emergency Stop  |<-->| • Parameter       |<-->| • Unit Tests      |
| • Balance Mon.    |    |   Validation      |    | • Integration     |
| • Limit Checking  |    | • Dynamic Reconf. |    |   Tests           |
| • Collision Det.  |    | • YAML Configs    |    | • Performance     |
| • Health Checks   |    | • Runtime Params  |    |   Tests           |
+-------------------+    +-------------------+    +-------------------+

Safety Layers:
- Hardware Safety: Physical safety mechanisms and emergency stops
- Software Safety: Runtime safety checks and validation
- Communication Safety: Message validation and timeout handling
- Algorithm Safety: Control limit enforcement and stability checks

Real-time Considerations:
- Deterministic execution paths
- Memory allocation avoidance during control loops
- Priority-based task scheduling
- Minimal communication latency
```

## Hands-on Exercise: Complete Humanoid Control Package Development

### Exercise Objective
Develop a complete humanoid robot control package that integrates advanced safety features, real-time control capabilities, and comprehensive testing for a specific humanoid robot subsystem.

### Scenario Description
Create a humanoid manipulation control package that enables safe arm and hand control for object interaction tasks. The package must include:

1. **Real-time Control**: 1kHz control loop for precise manipulation
2. **Safety Monitoring**: Force/torque limit checking and collision avoidance
3. **State Management**: Proper state transitions for different manipulation modes
4. **Testing Framework**: Comprehensive unit and integration tests
5. **Documentation**: Complete API documentation and safety guidelines

### Implementation Requirements

**1. Package Structure:**
- Follow the comprehensive package structure outlined above
- Include proper separation of concerns with dedicated modules
- Implement safety-critical components in isolated modules

**2. Safety Features:**
- Force/torque limit checking for each joint
- Collision detection and avoidance
- Emergency stop capabilities
- Safe position homing procedures

**3. Control Algorithms:**
- Inverse kinematics for end-effector positioning
- Force control for compliant manipulation
- Trajectory generation with smooth motion profiles
- Grasp planning and execution

**4. Testing Coverage:**
- 90%+ code coverage for safety-critical functions
- Integration tests for complete manipulation sequences
- Performance tests for real-time requirements
- Safety scenario tests for emergency conditions

### Sample Implementation Approach

```python
# manipulation_controller.py
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from manipulation_msgs.action import GraspObject, MoveToPose
import numpy as np
from scipy.spatial.transform import Rotation as R

class ManipulationController(Node):
    def __init__(self):
        super().__init__('manipulation_controller')

        # Initialize manipulation-specific components
        # Implement safety checks and validation
        # Set up real-time control parameters
        pass

def main(args=None):
    rclpy.init(args=args)
    node = ManipulationController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

### Assessment Criteria
- Proper implementation of safety features and validation
- Real-time performance meeting humanoid robot requirements
- Comprehensive testing with high coverage of safety-critical code
- Proper documentation and safety guidelines
- Clean, maintainable code structure following ROS2 conventions

## Summary

ROS2 package development is fundamental to building modular, maintainable, and safe humanoid robotics applications. By following proper package structure, dependency management, and safety design patterns, you can create reusable components that integrate seamlessly into larger humanoid robot systems while maintaining the strict safety and real-time requirements that humanoid robots demand.

The modular approach enables development teams to work on different subsystems in parallel while ensuring comprehensive testing, validation, and safety monitoring across all components. Proper package organization also facilitates maintenance, debugging, and evolution of complex humanoid robotics applications over time.

---

*This section covered comprehensive ROS2 package development with specific focus on safety, real-time performance, and best practices for humanoid robotics applications. The concepts of structured package organization, safety integration, and real-time considerations form the foundation for developing reliable humanoid robot systems. The next section will explore launch files and parameter management for coordinating complex humanoid robot systems.*

*Cross-reference: The package development principles covered in this section directly apply to simulation packages in Module 2, where similar safety and real-time considerations must be maintained in digital twin environments. The testing and validation approaches established here are essential for verifying simulation-to-reality transfer in humanoid robotics applications.*