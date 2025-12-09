---
title: Hands-on Exercises and Integration
sidebar_position: 7
---

# Hands-on Exercises and Integration

## Introduction to Integrated Exercises

This comprehensive section provides hands-on exercises that combine all the ROS2 concepts learned throughout this module, specifically applied to advanced humanoid robotics scenarios. These exercises are designed to reinforce understanding through practical application, develop problem-solving skills, and prepare students for simulation modules that follow.

The exercises progress systematically from basic single-concept tasks to complex multi-concept challenges that mirror real-world humanoid robot development scenarios. Each exercise builds upon the foundational knowledge of ROS2 architecture, Python package development, launch files, parameter management, and URDF modeling covered in previous sections, with emphasis on safety, real-time performance, and system integration.

Students will work with simulated humanoid robot systems, implementing sophisticated control strategies, sensor processing pipelines, and coordinated movement patterns. The exercises emphasize the integration aspects that are crucial for humanoid robot operation, where multiple subsystems must work together seamlessly while maintaining safety and performance requirements.

**Key Terms:**
- **Integration Exercise**: A comprehensive task that combines multiple ROS2 concepts and subsystems
- **Multi-concept Task**: An exercise requiring knowledge and application from several learning domains
- **Progressive Difficulty**: Exercises that build in complexity from basic to advanced levels
- **System Integration**: The process of combining multiple subsystems into a cohesive working system
- **Safety-Critical Implementation**: Development approaches that prioritize safety in all aspects

## Exercise 1: Advanced Humanoid Head Control System Integration

### Exercise Objective
Implement a complete humanoid robot head control system that integrates nodes, topics, services, and actions in a coordinated manner with comprehensive safety monitoring and real-time performance optimization.

### Scenario Description
You are tasked with creating a humanoid robot head control system that can:
- **Track objects** using camera data with real-time processing (topics with QoS optimization)
- **Calibrate sensors** when requested with safety validation (services with parameter validation)
- **Execute complex head movement sequences** with smooth trajectories (actions with feedback and cancellation)
- **Coordinate with other robot systems** while maintaining safety constraints (integration with monitoring)

### Implementation Requirements

**1. Node Architecture:**
- Create a `head_tracker` node that subscribes to camera image topics with optimized QoS settings
- Create a `head_controller` node that publishes joint commands with safety validation
- Create a `calibration_service` node that handles calibration requests with safety checks
- Implement a `safety_monitor` node that monitors head movement constraints

**2. Topic Communication with QoS Optimization:**
- Subscribe to `/camera/image_raw` for object detection with sensor QoS profile
- Subscribe to `/joint_states` for current head position feedback with default QoS
- Subscribe to `/imu/data` for balance monitoring with reliable QoS
- Publish to `/head_controller/position_commands` with control QoS for safety-critical commands
- Publish to `/head_controller/trajectory_commands` for smooth movement trajectories

**3. Service Integration with Safety:**
- Implement a `/head_calibration` service that performs head sensor calibration with safety validation
- Implement a `/head_safety_check` service that validates head position limits
- Return success/failure status with detailed diagnostic information
- Include timeout and error handling for all service calls

**4. Action Implementation with Real-time Performance:**
- Create a `/head_scan_pattern` action that executes a systematic scanning pattern
- Provide detailed feedback on scan progress and safety status
- Allow cancellation of the scan with safe termination procedures
- Implement trajectory smoothing for smooth motion execution

### Advanced Implementation Structure

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import Image, JointState, Imu
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger, SetBool
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration
import threading
import numpy as np
from typing import List, Tuple, Optional
import time

class AdvancedHeadController(Node):
    """
    Advanced head controller with safety monitoring and real-time performance.
    Implements comprehensive safety checks and optimized communication patterns.
    """

    def __init__(self):
        super().__init__('advanced_head_controller')

        # Safety parameters and constraints
        self.head_limits = {
            'pan': {'min': -1.57, 'max': 1.57, 'vel_max': 1.0},
            'tilt': {'min': -0.785, 'max': 0.785, 'vel_max': 0.8}
        }

        # Current state tracking
        self.current_positions = {'pan': 0.0, 'tilt': 0.0}
        self.target_positions = {'pan': 0.0, 'tilt': 0.0}
        self.safety_status = True
        self.emergency_stop = False

        # Callback groups for thread safety
        self.sensor_callback_group = MutuallyExclusiveCallbackGroup()
        self.control_callback_group = MutuallyExclusiveCallbackGroup()

        # QoS profiles for different data types
        sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        control_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # Publishers
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray, '/head_controller/position_commands', control_qos
        )

        self.trajectory_cmd_pub = self.create_publisher(
            Float64MultiArray, '/head_controller/trajectory_commands', control_qos
        )

        self.safety_status_pub = self.create_publisher(
            SetBool, '/head_safety_status', 10
        )

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback,
            sensor_qos, callback_group=self.sensor_callback_group
        )

        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback,
            sensor_qos, callback_group=self.sensor_callback_group
        )

        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback,
            sensor_qos, callback_group=self.sensor_callback_group
        )

        # Services
        self.calibration_srv = self.create_service(
            Trigger, '/head_calibration', self.calibration_callback,
            callback_group=self.control_callback_group
        )

        self.safety_check_srv = self.create_service(
            SetBool, '/head_safety_check', self.safety_check_callback,
            callback_group=self.control_callback_group
        )

        # Action server
        self.scan_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/head_scan_pattern',
            execute_callback=self.execute_scan_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.control_callback_group
        )

        # Timers for safety monitoring (100Hz)
        self.safety_timer = self.create_timer(
            0.01, self.safety_monitor, callback_group=self.control_callback_group
        )

        # Control timer (50Hz)
        self.control_timer = self.create_timer(
            0.02, self.control_loop, callback_group=self.control_callback_group
        )

        self.get_logger().info('Advanced head controller initialized with safety monitoring')

    def image_callback(self, msg: Image):
        """Process camera images for object detection and tracking."""
        # Implementation would include image processing for head tracking
        pass

    def joint_state_callback(self, msg: JointState):
        """Update current joint positions from joint state messages."""
        for i, name in enumerate(msg.name):
            if name in ['head_pan', 'head_tilt']:
                self.current_positions[name.replace('head_', '')] = msg.position[i]

    def imu_callback(self, msg: Imu):
        """Process IMU data for balance and orientation monitoring."""
        # Implementation would include balance calculations
        pass

    def calibration_callback(self, request, response):
        """Handle head calibration service requests with safety validation."""
        if self.emergency_stop:
            response.success = False
            response.message = 'Emergency stop active - calibration not allowed'
            return response

        if not self.validate_safety():
            response.success = False
            response.message = 'Safety validation failed - cannot calibrate'
            return response

        # Perform calibration sequence
        self.perform_calibration()

        response.success = True
        response.message = 'Head calibration completed successfully'
        return response

    def safety_check_callback(self, request, response):
        """Validate head position and safety constraints."""
        is_safe = self.validate_safety()
        response.success = is_safe
        response.message = f'Head safety check: {"PASSED" if is_safe else "FAILED"}'
        return response

    def goal_callback(self, goal_request):
        """Accept or reject scan goals based on safety status."""
        if self.emergency_stop or not self.validate_safety():
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject cancel requests."""
        return CancelResponse.ACCEPT

    def execute_scan_callback(self, goal_handle):
        """Execute head scanning action with safety monitoring and feedback."""
        self.get_logger().info('Executing head scan pattern...')

        # Generate scan trajectory
        trajectory = self.generate_scan_trajectory()

        # Initialize feedback and result
        feedback_msg = FollowJointTrajectory.Feedback()
        result = FollowJointTrajectory.Result()

        try:
            for step_idx, step in enumerate(trajectory):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                    result.error_string = 'Scan canceled by user'
                    return result

                # Check safety constraints
                if not self.validate_safety():
                    goal_handle.abort()
                    result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                    result.error_string = 'Safety violation during scan'
                    return result

                # Execute scan step
                success = self.execute_scan_step(step)

                if not success:
                    goal_handle.abort()
                    result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
                    result.error_string = 'Failed to execute scan step'
                    return result

                # Publish feedback
                feedback_msg.actual.positions = list(self.current_positions.values())
                feedback_msg.desired.positions = list(self.target_positions.values())
                feedback_msg.progress = float(step_idx) / len(trajectory)

                goal_handle.publish_feedback(feedback_msg)

                time.sleep(0.05)  # Control timing

            # Complete scan successfully
            goal_handle.succeed()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            result.error_string = 'Head scan completed successfully'

        except Exception as e:
            self.get_logger().error(f'Error during head scan: {str(e)}')
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
            result.error_string = f'Scan execution failed: {str(e)}'

        return result

    def safety_monitor(self):
        """Monitor safety conditions at 100Hz."""
        if not self.validate_safety():
            self.emergency_stop = True
            self.publish_safety_status(False)
            self.get_logger().error('SAFETY VIOLATION: Emergency stop activated')
        else:
            self.publish_safety_status(True)

    def control_loop(self):
        """Main control loop running at 50Hz."""
        if self.emergency_stop:
            self.publish_safe_commands()
            return

        # Implement control logic for smooth head movement
        commands = self.calculate_control_commands()
        if commands:
            self.joint_cmd_pub.publish(commands)

    def validate_safety(self) -> bool:
        """Validate all safety constraints."""
        # Check joint limits
        for joint_name, pos in self.current_positions.items():
            limits = self.head_limits[joint_name]
            if pos < limits['min'] or pos > limits['max']:
                return False

        # Check velocity limits
        # Check balance based on IMU data
        # Check for other safety conditions

        return True

    def publish_safety_status(self, is_safe: bool):
        """Publish safety status to monitoring systems."""
        msg = SetBool.Request()
        msg.data = is_safe
        self.safety_status_pub.publish(msg)

    def publish_safe_commands(self):
        """Publish safe zero commands to stop head movement."""
        cmd_msg = Float64MultiArray()
        cmd_msg.data = [0.0, 0.0]  # Zero all head joints
        self.joint_cmd_pub.publish(cmd_msg)

    def generate_scan_trajectory(self) -> List[Tuple[float, float]]:
        """Generate head scanning trajectory."""
        # Implementation would generate scanning pattern
        return [(0.0, 0.0)]  # Placeholder

    def execute_scan_step(self, step: Tuple[float, float]) -> bool:
        """Execute a single scan step."""
        # Implementation would execute head movement
        return True  # Placeholder

    def calculate_control_commands(self) -> Optional[Float64MultiArray]:
        """Calculate control commands for smooth movement."""
        # Implementation would calculate smooth trajectories
        return None  # Placeholder

    def perform_calibration(self):
        """Perform head calibration sequence."""
        # Implementation would calibrate head sensors
        pass

def main(args=None):
    rclpy.init(args=args)
    node = AdvancedHeadController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down advanced head controller...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercise 2: Comprehensive Humanoid Walking Control Package

### Exercise Objective
Create a complete ROS2 package for humanoid walking control that includes advanced safety features, real-time performance optimization, and comprehensive parameter validation with launch file coordination.

### Implementation Requirements

**1. Complete Package Structure:**
```
humanoid_walking/
├── CMakeLists.txt                    # Build configuration
├── package.xml                       # Package metadata and dependencies
├── setup.py                          # Python package configuration
├── pyproject.toml                    # Project configuration (PEP 621)
├── resource/                         # Resource files
│   └── humanoid_walking              # Package resources
├── launch/                           # Launch files for different configurations
│   ├── walking_system.launch.py      # Main walking system launch
│   ├── walking_with_monitoring.launch.py # Launch with safety monitoring
│   └── walking_calibration.launch.py # Launch for calibration mode
├── config/                           # Configuration files
│   ├── walking_params.yaml           # Walking parameters and gait settings
│   ├── balance_params.yaml           # Balance control parameters
│   └── safety_params.yaml            # Safety validation parameters
├── src/                              # Source code
│   └── humanoid_walking/
│       ├── __init__.py
│       ├── controllers/              # Control algorithms
│       │   ├── walking_controller.py # Main walking control
│       │   ├── balance_controller.py # Balance control
│       │   └── gait_generator.py     # Gait pattern generation
│       ├── nodes/                    # ROS2 nodes
│       │   ├── walking_node.py       # Walking control node
│       │   ├── balance_node.py       # Balance monitoring node
│       │   └── safety_node.py        # Safety validation node
│       └── utils/                    # Utility functions
│           ├── kinematics.py         # Kinematic calculations
│           ├── transforms.py         # Coordinate transformations
│           └── validation.py         # Parameter validation
├── test/                             # Test files
│   ├── unit/                         # Unit tests
│   ├── integration/                  # Integration tests
│   └── performance/                  # Performance tests
└── scripts/                          # Utility scripts
    ├── validate_params.py            # Parameter validation script
    └── calibrate_gait.py             # Gait calibration script
```

**2. Advanced Node Implementation:**
- Implement a `walking_controller` node with real-time gait generation
- Create a `balance_controller` node with ZMP (Zero Moment Point) calculation
- Develop a `safety_validator` node that monitors walking safety
- Include proper parameter validation and dynamic reconfiguration

**3. Launch File Coordination:**
- Create a main launch file that coordinates all walking subsystems
- Implement parameter validation before node startup
- Include safety monitoring and emergency procedures
- Support different operational modes (walking, calibration, standby)

**4. Parameter Configuration with Safety:**
- Create comprehensive YAML parameter files with safety validation
- Implement dynamic parameter reconfiguration
- Include walking gait parameters, balance thresholds, and safety limits
- Validate parameters against physical constraints

### Advanced Parameter File Example

```yaml
# config/walking_params.yaml
/**:  # Global parameters
  ros__parameters:
    use_sim_time: false
    log_level: "INFO"
    enable_diagnostics: true

# Walking controller parameters
walking_controller:
  ros__parameters:
    control_frequency: 200  # Hz
    max_walking_speed: 0.5  # m/s
    step_height: 0.05       # meters
    step_length: 0.3        # meters
    step_duration: 1.0      # seconds
    step_width: 0.2         # meters (distance between feet)
    zmp_margin: 0.05        # meters safety margin
    max_tilt_angle: 0.52    # radians (30 degrees)
    balance_threshold: 0.02 # meters
    gait_type: "dual_support"
    step_adjustment_enabled: true
    max_step_adjustment: 0.05  # meters

    # Joint control parameters
    joint_control:
      position_gain: 100.0
      velocity_gain: 10.0
      effort_limit: 200.0

    # Safety limits
    safety_limits:
      max_joint_velocity: 2.0  # rad/s
      max_joint_effort: 100.0  # Nm
      balance_timeout: 0.5     # seconds

# Balance controller parameters
balance_controller:
  ros__parameters:
    control_frequency: 100  # Hz
    zmp_tolerance: 0.02     # meters
    com_tracking_gain: 50.0
    imu_filter_cutoff: 10.0 # Hz
    balance_recovery_enabled: true
    recovery_threshold: 0.1  # meters

# Safety validation parameters
safety_validator:
  ros__parameters:
    emergency_stop_timeout: 0.1  # seconds
    joint_limit_safety_margin: 0.05  # radians
    balance_threshold: 0.1     # meters
    max_tilt_angle: 0.52       # radians
    collision_detection_enabled: true
    minimum_safe_distance: 0.5 # meters
```

## Exercise 3: URDF Integration with Control System and Simulation

### Exercise Objective
Integrate a humanoid robot URDF model with control systems and simulation environments, implementing comprehensive kinematic validation, collision detection, and real-time performance monitoring.

### Implementation Requirements

**1. Advanced URDF Model:**
- Create a complete humanoid URDF with 20+ links and appropriate joint limits
- Implement proper inertial properties for dynamic simulation
- Include collision and visual properties optimized for performance
- Add transmission definitions for ROS control integration

**2. Robot State Integration:**
- Implement a robot state publisher with optimized performance
- Create forward and inverse kinematics solvers
- Develop collision detection and self-collision avoidance
- Integrate with TF2 for coordinate transformations

**3. Control System Integration:**
- Create joint trajectory controllers for smooth motion
- Implement safety validation based on URDF limits
- Develop real-time kinematic solvers
- Add simulation compatibility with Gazebo

### Advanced URDF Integration Example

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import ColorRGBA
import numpy as np
import threading
from typing import Dict, List, Tuple
import time

class AdvancedRobotStatePublisher(Node):
    """
    Advanced robot state publisher with kinematic validation,
    collision detection, and real-time performance optimization.
    """

    def __init__(self):
        super().__init__('advanced_robot_state_publisher')

        # Robot model data
        self.joint_limits = {}
        self.link_masses = {}
        self.link_inertias = {}
        self.kinematic_chains = {}

        # Joint state management
        self.current_joint_states = {}
        self.target_joint_states = {}
        self.joint_velocities = {}

        # Performance monitoring
        self.update_rate = 100  # Hz
        self.last_update_time = time.time()

        # Publishers
        self.joint_state_pub = self.create_publisher(
            JointState, 'joint_states', 10
        )

        self.marker_pub = self.create_publisher(
            MarkerArray, 'robot_markers', 10
        )

        # Subscribers
        self.joint_cmd_sub = self.create_subscription(
            JointState, 'joint_commands', self.joint_command_callback, 10
        )

        # Transform broadcasters
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # Timers for different update rates
        self.state_timer = self.create_timer(
            1.0/self.update_rate, self.publish_robot_state
        )

        self.performance_timer = self.create_timer(
            1.0, self.performance_monitor  # 1 Hz performance monitoring
        )

        # Initialize robot model (in practice, load from URDF)
        self.initialize_robot_model()

        self.get_logger().info(
            f'Advanced robot state publisher initialized at {self.update_rate}Hz'
        )

    def initialize_robot_model(self):
        """Initialize robot model data from URDF or other sources."""
        # In practice, this would parse URDF file and extract information
        # For this example, we'll use placeholder values
        self.joint_limits = {
            'left_hip_joint': {'min': -1.57, 'max': 1.57},
            'left_knee_joint': {'min': 0.0, 'max': 2.0},
            'right_hip_joint': {'min': -1.57, 'max': 1.57},
            'right_knee_joint': {'min': 0.0, 'max': 2.0},
            # Add more joints as needed
        }

        # Initialize current states
        for joint_name in self.joint_limits.keys():
            self.current_joint_states[joint_name] = 0.0
            self.target_joint_states[joint_name] = 0.0
            self.joint_velocities[joint_name] = 0.0

    def joint_command_callback(self, msg: JointState):
        """Handle joint command messages with validation."""
        for i, joint_name in enumerate(msg.name):
            if joint_name in self.joint_limits:
                # Validate joint limits
                target_pos = msg.position[i] if i < len(msg.position) else 0.0
                limits = self.joint_limits[joint_name]

                if target_pos < limits['min'] or target_pos > limits['max']:
                    # Clamp to safe limits
                    clamped_pos = max(limits['min'], min(limits['max'], target_pos))
                    self.get_logger().warn(
                        f'Joint {joint_name} command {target_pos} '
                        f'clamped to safe limits [{limits["min"]}, {limits["max"]}]'
                    )
                    target_pos = clamped_pos

                self.target_joint_states[joint_name] = target_pos

    def calculate_forward_kinematics(self) -> Dict[str, np.ndarray]:
        """Calculate forward kinematics for all links."""
        # Implementation would calculate link positions based on joint angles
        # This is a simplified placeholder
        transforms = {}

        # Calculate transforms for each link based on joint positions
        for joint_name, position in self.current_joint_states.items():
            # Calculate transformation matrix (simplified)
            transform = np.eye(4)
            transform[2, 3] = position  # Simple z-axis movement

            link_name = joint_name.replace('_joint', '_link')
            transforms[link_name] = transform

        return transforms

    def detect_collisions(self) -> List[Tuple[str, str]]:
        """Detect potential collisions between robot links."""
        # Implementation would check for link-link collisions
        # This is a simplified placeholder
        collisions = []

        # Check for self-collisions based on current configuration
        # In practice, this would use geometric collision detection

        return collisions

    def publish_robot_state(self):
        """Publish robot state with transforms and markers."""
        current_time = self.get_clock().now()

        # Create joint state message
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = current_time.to_msg()
        joint_state_msg.header.frame_id = 'base_link'

        # Add joint names and positions
        for joint_name, position in self.current_joint_states.items():
            joint_state_msg.name.append(joint_name)
            joint_state_msg.position.append(position)

            # Calculate velocity (simplified)
            velocity = self.joint_velocities[joint_name]
            joint_state_msg.velocity.append(velocity)

            # Calculate effort (placeholder)
            joint_state_msg.effort.append(0.0)

        # Publish joint states
        self.joint_state_pub.publish(joint_state_msg)

        # Calculate forward kinematics
        link_transforms = self.calculate_forward_kinematics()

        # Publish transforms
        for link_name, transform_matrix in link_transforms.items():
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = 'base_link'
            t.child_frame_id = link_name

            # Extract translation and rotation from transformation matrix
            t.transform.translation.x = transform_matrix[0, 3]
            t.transform.translation.y = transform_matrix[1, 3]
            t.transform.translation.z = transform_matrix[2, 3]

            # Convert rotation matrix to quaternion (simplified)
            # In practice, proper matrix-to-quaternion conversion would be used
            t.transform.rotation.w = 1.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0

            self.tf_broadcaster.sendTransform(t)

        # Check for collisions
        collisions = self.detect_collisions()
        if collisions:
            self.get_logger().warn(f'Detected {len(collisions)} potential collisions')

        # Update current positions toward targets with smooth interpolation
        self.interpolate_joint_positions()

    def interpolate_joint_positions(self):
        """Smoothly interpolate joint positions toward targets."""
        interpolation_factor = 0.1  # Adjust for desired smoothness

        for joint_name in self.current_joint_states.keys():
            current_pos = self.current_joint_states[joint_name]
            target_pos = self.target_joint_states[joint_name]

            # Simple linear interpolation
            new_pos = current_pos + interpolation_factor * (target_pos - current_pos)
            self.current_joint_states[joint_name] = new_pos

            # Calculate velocity
            self.joint_velocities[joint_name] = (
                (target_pos - current_pos) * self.update_rate * interpolation_factor
            )

    def performance_monitor(self):
        """Monitor and log performance metrics."""
        current_time = time.time()
        dt = current_time - self.last_update_time

        # Calculate actual update rate
        actual_rate = 1.0 / dt if dt > 0 else 0

        # Log performance if below expected rate
        if actual_rate < self.update_rate * 0.9:  # 10% tolerance
            self.get_logger().warn(
                f'Performance warning: Actual rate {actual_rate:.1f}Hz '
                f'below expected {self.update_rate}Hz'
            )

        self.last_update_time = current_time

        # Log performance metrics
        self.get_logger().debug(f'Robot state publisher running at {actual_rate:.1f}Hz')

def main(args=None):
    rclpy.init(args=args)
    node = AdvancedRobotStatePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down advanced robot state publisher...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercise 4: Complete Humanoid Robot Integration System

### Exercise Objective
Implement a complete humanoid robot system that integrates all concepts from this module with advanced safety features, real-time performance optimization, and comprehensive monitoring capabilities.

### Scenario: Humanoid Assistant Robot
Create a complete system where a humanoid robot serves as an assistant that can:
1. **Navigate** to a person using walking control (action-based navigation)
2. **Greet** the person with coordinated head and arm movements (multi-joint control)
3. **Engage** in simple interaction tasks (service-based commands)
4. **Maintain** safety throughout all operations (comprehensive safety monitoring)

### System Architecture Requirements

**1. Core Subsystems:**
- **Navigation System**: Walking control with obstacle avoidance and path planning
- **Interaction System**: Head tracking, arm manipulation, and gesture control
- **Safety System**: Continuous monitoring of all safety parameters
- **Coordination System**: Behavior management and task scheduling

**2. Communication Architecture:**
- **Real-time topics** for sensor data and control commands
- **Services** for high-level commands and system status
- **Actions** for long-running behaviors and complex tasks
- **Parameters** for dynamic configuration and safety limits

**3. Safety and Validation:**
- **Pre-motion validation** of all movement plans
- **Real-time safety monitoring** of joint limits and balance
- **Emergency stop procedures** with safe position recovery
- **Collision avoidance** with humans and environment

### Complete System Implementation Checklist

**Design Phase:**
- [ ] Define complete system architecture with all subsystems
- [ ] Design communication patterns between subsystems
- [ ] Specify safety requirements and validation procedures
- [ ] Plan launch file structure for coordinated startup

**Implementation Phase:**
- [ ] Implement navigation system with walking control
- [ ] Create interaction system with head/arm coordination
- [ ] Develop safety monitoring and validation
- [ ] Build coordination system for behavior management

**Integration Phase:**
- [ ] Create comprehensive launch files
- [ ] Implement parameter validation and management
- [ ] Develop system monitoring and diagnostics
- [ ] Test safety procedures and emergency responses

**Validation Phase:**
- [ ] Verify real-time performance requirements
- [ ] Test safety system under various conditions
- [ ] Validate communication patterns and data flow
- [ ] Document system behavior and limitations

### Assessment Criteria

**Technical Implementation:**
- Proper integration of all ROS2 communication patterns (nodes, topics, services, actions)
- Correct implementation of safety monitoring and validation systems
- Appropriate use of QoS settings for different data types
- Real-time performance meeting humanoid robot requirements

**System Design:**
- Clean, modular architecture following ROS2 best practices
- Proper separation of concerns between different subsystems
- Comprehensive error handling and graceful degradation
- Efficient resource utilization and performance optimization

**Safety and Reliability:**
- Comprehensive safety validation at all system levels
- Proper emergency stop and recovery procedures
- Collision avoidance and human safety considerations
- Robust error handling and system recovery

**Documentation and Testing:**
- Clear code documentation and API documentation
- Comprehensive unit and integration tests
- Performance benchmarks and validation results
- Safety analysis and risk assessment documentation

## Progressive Exercise Structure for Skill Development

### Beginner Level (Foundation Skills)
- **Exercise 1**: Create a simple publisher/subscriber pair for joint control
- **Exercise 2**: Implement a basic service for joint calibration
- **Exercise 3**: Create a simple action server for single-joint movement

### Intermediate Level (Integration Skills)
- **Exercise 4**: Integrate multiple nodes with proper parameter configuration
- **Exercise 5**: Implement coordinated movement of 2-3 joints
- **Exercise 6**: Add basic safety validation to movement commands

### Advanced Level (System Skills)
- **Exercise 7**: Create a complete subsystem with launch file coordination
- **Exercise 8**: Implement real-time control with performance optimization
- **Exercise 9**: Add comprehensive safety monitoring and validation

### Expert Level (Complete System)
- **Exercise 10**: Full humanoid robot system integration
- **Exercise 11**: Multi-modal operation with different behavioral modes
- **Exercise 12**: Complete safety-critical system with emergency procedures

## Summary

This comprehensive exercises module has provided extensive hands-on experience with ROS2 concepts specifically applied to advanced humanoid robotics applications. Students have learned to:

- **Integrate multiple ROS2 communication patterns** (topics, services, actions) in complex systems with safety considerations
- **Develop properly structured Python packages** for humanoid systems with real-time performance requirements
- **Create coordinated launch files** for complex robot systems with parameter validation and safety checks
- **Incorporate URDF models** into control systems with kinematic validation and collision detection
- **Design progressive exercises** that build complexity gradually from basic to expert levels
- **Implement comprehensive safety systems** with real-time monitoring and emergency procedures
- **Optimize performance** for real-time humanoid robot control applications

These advanced skills form the foundation for sophisticated robotics applications covered in subsequent modules, including simulation environments, digital twins, and advanced control strategies. The emphasis on safety, real-time performance, and system integration prepares students for professional humanoid robotics development.

---

*This section combined all ROS2 concepts with specific focus on advanced humanoid robotics applications, safety-critical implementation, and real-time performance optimization. The comprehensive exercises and integration patterns covered here will be essential when exploring Digital Twin implementations in Module 2 and Isaac Sim in Module 3, where integrated, safe, and high-performance systems are used extensively.*

*Cross-reference: The integration principles, safety patterns, and real-time performance considerations covered in this section form the foundation for creating complex, safe, and reliable simulation environments in upcoming modules. The system architecture and validation approaches established here are directly applicable to simulation-to-reality transfer scenarios.*