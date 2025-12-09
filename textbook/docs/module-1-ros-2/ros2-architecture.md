---
title: ROS2 Architecture Fundamentals
sidebar_position: 2
---

# ROS2 Architecture Fundamentals

## Introduction

**ROS2** (Robot Operating System 2) is a flexible framework for writing robot software that serves as the communication backbone for complex robotic systems. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms. In the context of **humanoid robotics**, ROS2 provides the critical communication infrastructure that allows different subsystems of a humanoid robot—such as perception, planning, control, and actuation—to work together seamlessly.

The architecture of ROS2 represents a significant evolution from ROS1, addressing many of its predecessor's limitations through the adoption of **DDS (Data Distribution Service)** as the underlying middleware. This change provides better support for real-time systems, security, and distributed computing—essential features for humanoid robots that operate in dynamic environments with humans.

ROS2's architecture is fundamentally designed around a distributed computing model where multiple processes (nodes) communicate through a publish-subscribe pattern, service calls, and action interfaces. This architecture is particularly well-suited for humanoid robots, which require coordination between numerous subsystems that must operate in real-time while maintaining safety and reliability.

**Key Terms:**
- **Node**: A process that performs computation and serves as the fundamental unit of a ROS2 program
- **Topic**: A named bus over which nodes exchange messages using a publish/subscribe pattern
- **Service**: A synchronous request/response communication pattern between nodes
- **Action**: A communication pattern for long-running tasks that provides feedback and can be canceled
- **DDS**: Data Distribution Service, the middleware that enables message passing in ROS2
- **RMW**: ROS Middleware, the abstraction layer that interfaces with DDS implementations

## Deep Dive into Nodes: The Building Blocks of ROS2

A **node** is the fundamental unit of computation in ROS2, representing an executable process that performs specific tasks within the robot system. In humanoid robotics applications, nodes serve as modular components that can be developed, tested, and maintained independently while contributing to the overall robot behavior.

### Node Characteristics and Lifecycle

Nodes in ROS2 have a well-defined lifecycle that includes creation, configuration, activation, and cleanup phases. This lifecycle management is crucial for humanoid robots that require reliable startup and shutdown procedures to ensure safety.

```python
import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from sensor_msgs.msg import JointState
import time

class HumanoidJointController(LifecycleNode):
    def __init__(self):
        super().__init__('humanoid_joint_controller')
        self.get_logger().info('Humanoid Joint Controller node initialized')

    def on_configure(self, state: LifecycleState):
        """Configure the node and initialize resources"""
        self.get_logger().info('Configuring humanoid joint controller')
        # Initialize joint control resources
        self.joint_publisher = self.create_publisher(JointState, 'joint_states', 10)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState):
        """Activate the node and start operations"""
        self.get_logger().info('Activating humanoid joint controller')
        # Start control loops
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState):
        """Deactivate the node and stop operations safely"""
        self.get_logger().info('Deactivating humanoid joint controller')
        # Stop control loops safely
        return super().on_deactivate(state)

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidJointController()

    # Transition through lifecycle states
    node.trigger_configure()
    node.trigger_activate()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.trigger_deactivate()
        node.destroy_node()
        rclpy.shutdown()
```

### Humanoid-Specific Node Categories

In humanoid robotics, nodes are typically organized by function and can be categorized as follows:

**Control Nodes:**
- Joint controllers for individual limbs
- Balance and stability controllers
- Walking pattern generators
- Manipulation controllers

**Perception Nodes:**
- Camera image processing
- IMU data interpretation
- Object detection and recognition
- SLAM (Simultaneous Localization and Mapping)

**Planning Nodes:**
- Path planning for navigation
- Trajectory generation for movements
- High-level behavior planning
- Motion planning for manipulation

**System Nodes:**
- State estimation and filtering
- System monitoring and diagnostics
- Safety and emergency handling
- Communication and coordination

### Node Communication and Coordination

Nodes in a humanoid robot system must coordinate effectively to achieve complex behaviors. The following diagram illustrates the communication patterns between different node categories in a humanoid robot:

```
                    Humanoid Robot Node Architecture
                    =================================

+---------------------+       +---------------------+       +---------------------+
|   Perception        |       |   Planning &        |       |   Control &         |
|   Nodes             |<----->|   Decision          |<----->|   Actuation         |
|                     |       |   Nodes             |       |   Nodes             |
| • Camera Processing |       | • Behavior Planner  |       | • Joint Controllers |
| • IMU Processing    |<----->| • Path Planner      |<----->| • Balance Control   |
| • Object Detection  |       | • Trajectory Gen    |       | • Walking Control   |
| • SLAM              |       | • State Estimator   |       | • Safety Systems    |
+---------------------+       +---------------------+       +---------------------+
         |                           |                           |
         | Sensor Data               | Commands & Plans          | Joint Commands
         |-------------------------->|-------------------------->|
         |<--------------------------|<--------------------------|
         | Processed Perception      | Feedback & Status         | Execution Status

+---------------------+       +---------------------+       +---------------------+
|   System & Safety   |       |   Human Interface   |       |   Simulation &      |
|   Nodes             |<----->|   Nodes             |<----->|   Visualization     |
|                     |       |                     |       |   Nodes             |
| • System Monitor    |<----->| • Command Interface |<----->| • RViz Visualization|
| • Safety Manager    |       | • Teleoperation     |       | • Gazebo Interface  |
| • Diagnostics       |       | • Speech Interface  |       | • Logging           |
+---------------------+       +---------------------+       +---------------------+

Communication Patterns:
- Topics: Continuous data streams (sensor fusion, state estimation)
- Services: Synchronous operations (calibration, emergency stops)
- Actions: Long-running behaviors (walking, manipulation, navigation)
```

## Topics: Asynchronous Communication for Real-time Systems

**Topics** in ROS2 implement a publish/subscribe communication pattern that is essential for real-time humanoid robot systems. Publishers send messages to a topic, and subscribers receive messages from a topic without direct coupling between them. This decoupling allows for flexible system design and is particularly valuable for humanoid robots that require multiple systems to access the same data simultaneously.

### Topic Communication Characteristics

Topics in ROS2 have several key characteristics that make them suitable for humanoid robotics:

- **Asynchronous**: Publishers and subscribers don't need to be synchronized in time
- **Many-to-many**: Multiple publishers can send to a topic, and multiple subscribers can receive from it
- **Typed**: Messages have defined schemas for type safety
- **QoS-configurable**: Quality of Service settings allow tuning for different requirements

### Quality of Service (QoS) for Humanoid Systems

Quality of Service (QoS) settings are crucial for humanoid robots where different data streams have different requirements:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# High-frequency sensor data (IMU, joint encoders) - need reliability and low latency
sensor_qos = QoSProfile(
    depth=10,  # Small buffer to minimize latency
    reliability=ReliabilityPolicy.RELIABLE,  # Must not lose sensor data
    history=HistoryPolicy.KEEP_LAST,  # Only keep most recent messages
    durability=DurabilityPolicy.VOLATILE  # Don't need to keep for late-joining subscribers
)

# Control commands - critical for safety
control_qos = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    durability=DurabilityPolicy.VOLATILE
)

# Logging and diagnostics - can be best-effort
diagnostic_qos = QoSProfile(
    depth=100,  # Larger buffer for logging
    reliability=ReliabilityPolicy.BEST_EFFORT,  # Can afford to lose some diagnostic messages
    history=HistoryPolicy.KEEP_ALL,  # Keep all diagnostic messages
    durability=DurabilityPolicy.VOLATILE
)

class HumanoidSensorProcessor(Node):
    def __init__(self):
        super().__init__('humanoid_sensor_processor')

        # Subscribe to high-frequency sensor data with appropriate QoS
        self.imu_sub = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            sensor_qos
        )

        # Publish processed sensor data
        self.state_pub = self.create_publisher(
            RobotState,
            'robot_state',
            sensor_qos
        )

        self.get_logger().info('Humanoid sensor processor initialized with QoS settings')

    def imu_callback(self, msg):
        """Process IMU data for balance control"""
        # Process IMU data for balance and orientation
        processed_state = self.process_imu_data(msg)
        self.state_pub.publish(processed_state)
```

### Common Topic Patterns in Humanoid Robotics

**Sensor Data Topics:**
- `/joint_states`: Current positions, velocities, and efforts of all joints
- `/imu/data`: Inertial measurement unit data for balance and orientation
- `/camera/image_raw`: Raw camera images for perception
- `/scan`: LIDAR data for environment mapping
- `/tf` and `/tf_static`: Transform data for coordinate system relationships

**Control Command Topics:**
- `/joint_group_position_controller/commands`: Position commands for joint groups
- `/cmd_vel`: Velocity commands for base movement
- `/target_positions`: Desired joint positions for specific behaviors

**System State Topics:**
- `/robot_state`: High-level robot state information
- `/battery_state`: Power system status
- `/diagnostics`: System health information

## Services: Synchronous Communication for Critical Operations

**Services** provide a synchronous request/response communication pattern that is essential for operations that need to complete before the client can continue. In humanoid robotics, services are used for critical operations that require guaranteed completion and response.

### Service Implementation for Humanoid Safety

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger, SetBool
from humanoid_msgs.srv import CalibrateJoint, EmergencyStop

class HumanoidSafetyManager(Node):
    def __init__(self):
        super().__init__('humanoid_safety_manager')

        # Emergency stop service - critical for safety
        self.emergency_stop_srv = self.create_service(
            EmergencyStop,
            'emergency_stop',
            self.emergency_stop_callback
        )

        # Calibration service for joint zeroing
        self.calibrate_srv = self.create_service(
            CalibrateJoint,
            'calibrate_joint',
            self.calibrate_joint_callback
        )

        # System enable/disable service
        self.enable_srv = self.create_service(
            SetBool,
            'enable_robot',
            self.enable_robot_callback
        )

        self.robot_enabled = False
        self.get_logger().info('Humanoid safety manager initialized')

    def emergency_stop_callback(self, request, response):
        """Handle emergency stop request"""
        self.get_logger().warn('EMERGENCY STOP REQUESTED')

        # Immediately stop all joint controllers
        self.stop_all_joints()

        # Set robot to safe state
        self.robot_enabled = False

        response.success = True
        response.message = 'Emergency stop executed successfully'
        return response

    def calibrate_joint_callback(self, request, response):
        """Handle joint calibration request"""
        if not self.robot_enabled:
            response.success = False
            response.message = 'Robot not enabled, cannot calibrate'
            return response

        # Perform calibration sequence for specified joint
        success = self.perform_calibration(request.joint_name)

        response.success = success
        response.message = f'Joint {request.joint_name} calibration: {"SUCCESS" if success else "FAILED"}'
        return response

    def enable_robot_callback(self, request, response):
        """Enable or disable robot operations"""
        if request.data:
            # Safety checks before enabling
            if self.system_check_ok():
                self.robot_enabled = True
                response.success = True
                response.message = 'Robot enabled successfully'
            else:
                response.success = False
                response.message = 'System check failed, cannot enable robot'
        else:
            # Disable robot safely
            self.robot_enabled = False
            self.stop_all_joints()
            response.success = True
            response.message = 'Robot disabled safely'

        return response

    def system_check_ok(self):
        """Perform safety checks before enabling robot"""
        # Check battery levels, joint limits, etc.
        return True  # Simplified for example

    def stop_all_joints(self):
        """Send stop commands to all joint controllers"""
        # Implementation would send zero velocity commands to all joints
        pass

    def perform_calibration(self, joint_name):
        """Perform calibration sequence for specified joint"""
        # Implementation would move joint to calibration position
        return True  # Simplified for example
```

## Actions: Managing Long-Running Behaviors

**Actions** are designed for long-running tasks that provide feedback during execution and can be canceled. They are particularly valuable for humanoid robotics tasks that take significant time to complete and require monitoring of progress.

### Action Architecture for Humanoid Behaviors

```python
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from humanoid_msgs.action import WalkToPose, ManipulateObject
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
import threading
import time

class HumanoidWalkingController(Node):
    def __init__(self):
        super().__init__('humanoid_walking_controller')

        # Action server for walking to a pose
        self._walk_action_server = ActionServer(
            self,
            WalkToPose,
            'walk_to_pose',
            execute_callback=self.execute_walk_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        # Publishers for joint commands during walking
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray,
            'joint_trajectory',
            10
        )

        self.get_logger().info('Humanoid walking controller initialized')

    def goal_callback(self, goal_request):
        """Accept or reject goal based on current state"""
        # Check if robot is in safe state to accept walking goal
        if self.is_robot_safe_to_walk():
            return GoalResponse.ACCEPT
        else:
            return GoalResponse.REJECT

    def cancel_callback(self, goal_handle):
        """Accept or reject cancel request"""
        self.get_logger().info('Received cancel request for walking action')
        return CancelResponse.ACCEPT

    def execute_walk_callback(self, goal_handle):
        """Execute the walking action with feedback"""
        self.get_logger().info('Executing walking action...')

        # Get target pose from goal
        target_pose = goal_handle.request.target_pose
        walk_speed = goal_handle.request.speed

        # Initialize feedback message
        feedback_msg = WalkToPose.Feedback()
        result = WalkToPose.Result()

        try:
            # Generate walking pattern to target pose
            walk_pattern = self.generate_walk_pattern(target_pose, walk_speed)

            # Execute walking with feedback
            for step_idx, step in enumerate(walk_pattern):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'Walking action canceled'
                    return result

                # Execute single walking step
                self.execute_walking_step(step)

                # Publish feedback
                feedback_msg.current_pose = self.get_current_pose()
                feedback_msg.progress = float(step_idx) / len(walk_pattern)
                feedback_msg.remaining_distance = self.calculate_remaining_distance(target_pose)

                goal_handle.publish_feedback(feedback_msg)

                # Sleep for step timing
                time.sleep(0.1)

            # Check if successfully reached target
            if self.is_at_target_pose(target_pose):
                goal_handle.succeed()
                result.success = True
                result.message = 'Successfully walked to target pose'
            else:
                goal_handle.abort()
                result.success = False
                result.message = 'Failed to reach target pose'

        except Exception as e:
            self.get_logger().error(f'Error during walking: {str(e)}')
            goal_handle.abort()
            result.success = False
            result.message = f'Walking failed: {str(e)}'

        return result

    def generate_walk_pattern(self, target_pose, speed):
        """Generate walking pattern to reach target pose"""
        # Simplified walking pattern generation
        # In practice, this would involve inverse kinematics and balance control
        return [Pose() for _ in range(10)]  # Placeholder

    def execute_walking_step(self, step):
        """Execute a single walking step"""
        # Send joint commands for the walking step
        cmd_msg = Float64MultiArray()
        # Populate with joint angles for the step
        self.joint_cmd_pub.publish(cmd_msg)

    def is_robot_safe_to_walk(self):
        """Check if robot is in safe state for walking"""
        return True  # Simplified for example

    def get_current_pose(self):
        """Get current robot pose"""
        return Pose()  # Simplified for example

    def calculate_remaining_distance(self, target_pose):
        """Calculate remaining distance to target"""
        return 0.0  # Simplified for example

    def is_at_target_pose(self, target_pose):
        """Check if robot reached target pose within tolerance"""
        return True  # Simplified for example
```

## DDS Middleware: The Foundation of ROS2 Communication

The **DDS (Data Distribution Service)** middleware is the foundation that enables ROS2's distributed communication architecture. DDS provides the underlying infrastructure for discovery, serialization, and transportation of messages between nodes, whether they're running on the same computer or distributed across a network.

### DDS Configuration for Humanoid Robotics

DDS configuration is crucial for humanoid robots that often have strict real-time requirements and safety considerations:

```xml
<!-- DDS configuration profile for humanoid robot -->
<dds>
    <profiles>
        <!-- Profile for high-priority control messages -->
        <participant profile="humanoid_control_participant">
            <rtps>
                <name>HumanoidControlParticipant</name>
                <builtin>
                    <discovery_config>
                        <lease_duration>
                            <sec>10</sec>
                        </lease_duration>
                    </discover_config>
                </builtin>
                <userTransports>
                    <transport_id>shm_transport</transport_id>
                </userTransports>
                <useBuiltinTransports>false</useBuiltinTransports>
            </rtps>
        </participant>

        <!-- QoS profile for control data -->
        <data_writer profile="control_qos">
            <qos>
                <reliability>
                    <kind>RELIABLE_RELIABILITY_QOS</kind>
                </reliability>
                <durability>
                    <kind>VOLATILE_DURABILITY_QOS</kind>
                </durability>
                <history>
                    <kind>KEEP_LAST_HISTORY_QOS</kind>
                    <depth>1</depth>
                </history>
                <publish_mode>
                    <kind>ASYNCHRONOUS_PUBLISH_MODE</kind>
                </publish_mode>
            </qos>
        </data_writer>
    </profiles>
</dds>
```

## Safety and Reliability in Humanoid ROS2 Systems

Humanoid robots operating in human environments require robust safety and reliability mechanisms. The ROS2 architecture provides several features that support these requirements:

### Safety Architecture Pattern

```
                    Humanoid Robot Safety Architecture
                    ==================================

+-------------------+    +-------------------+    +-------------------+
|   Primary         |    |   Safety          |    |   Emergency       |
|   Control Nodes   |<-->|   Monitor         |<-->|   Systems         |
|                   |    |                   |    |                   |
| • Joint Control   |    | • State Monitor   |    | • Emergency Stop  |
| • Balance Control |    | • Limit Checker   |    | • Power Off       |
| • Behavior Exec   |    | • Collision Det   |    | • Safe Position   |
+-------------------+    +-------------------+    +-------------------+
         |                       |                       |
         | Control Commands      | System State          | Safety Commands
         |--------------------->|--------------------->|------------------>|
         | Status Feedback       | Safety Status         | Emergency Actions
         |<---------------------|<---------------------|<------------------|

+-------------------+    +-------------------+    +-------------------+
|   Redundant       |    |   Diagnostic      |    |   Logging &       |
|   Systems         |    |   Systems         |    |   Monitoring      |
|                   |    |                   |    |                   |
| • Backup Control  |    | • Health Checks   |    | • Performance     |
| • Fallback Modes  |    | • Error Detection |    | • Data Recording  |
| • Safe States     |    | • Calibration     |    | • System Status   |
+-------------------+    +-------------------+    +-------------------+

Safety Layers:
- Hardware Safety: Physical safety mechanisms and limits
- Software Safety: Safety nodes and limit checking
- Operational Safety: Procedures and protocols
```

## Hands-on Exercise: Complete Humanoid Robot Architecture Implementation

### Exercise Objective
Implement a complete humanoid robot architecture that integrates all ROS2 communication patterns (nodes, topics, services, and actions) for a walking and manipulation task.

### Scenario Description
Design and implement a humanoid robot system that can:
1. Receive navigation goals via an action interface
2. Use sensor data from topics to perceive the environment
3. Request object manipulation services when reaching a destination
4. Coordinate multiple subsystems safely and efficiently

### Implementation Requirements

**1. Node Structure:**
- `navigation_manager`: Coordinates navigation and manipulation tasks
- `sensor_fusion`: Processes sensor data from multiple sources
- `walking_controller`: Handles bipedal locomotion
- `manipulation_controller`: Handles arm and hand movements
- `safety_monitor`: Monitors system health and enforces safety limits

**2. Topic Configuration:**
- Use appropriate QoS settings for different data types
- Implement proper message types for humanoid-specific data
- Include transform (TF) data for coordinate system management

**3. Service Implementation:**
- Create a manipulation service that can be called after navigation
- Implement safety services for emergency stops and system checks
- Include calibration services for joint zeroing

**4. Action Development:**
- Implement a navigation action that includes walking and obstacle avoidance
- Create a manipulation action for object interaction tasks
- Include proper feedback and cancellation mechanisms

### Sample Implementation Structure

```python
# navigation_manager.py
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose
from humanoid_msgs.action import NavigateToPose
from humanoid_msgs.srv import ManipulateObject
from sensor_msgs.msg import JointState, Imu

class NavigationManager(Node):
    def __init__(self):
        super().__init__('navigation_manager')

        # Action client for navigation
        self.nav_action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )

        # Service client for manipulation
        self.manip_srv_client = self.create_client(
            ManipulateObject, 'manipulate_object'
        )

        # Subscriptions for sensor data
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10
        )

        self.imu_sub = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, 10
        )

        self.get_logger().info('Navigation manager initialized')

    def navigate_and_manipulate(self, target_pose, object_pose):
        """Navigate to target and manipulate object"""
        # Send navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.target_pose = target_pose

        self.nav_action_client.wait_for_server()
        future = self.nav_action_client.send_goal_async(goal_msg)
        # Handle navigation completion and then call manipulation service
        pass

def main(args=None):
    rclpy.init(args=args)
    node = NavigationManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

### Assessment Criteria
- Proper use of all four communication patterns (nodes, topics, services, actions)
- Appropriate QoS configuration for different data types
- Safety considerations integrated throughout the system
- Modular design allowing for independent testing of components
- Proper error handling and graceful degradation

## Summary

ROS2's architecture provides a robust foundation for humanoid robotics applications through its node-based design, flexible communication patterns (topics, services, actions), and modern middleware infrastructure. The distributed nature of the architecture, built on DDS, enables complex humanoid robot systems to coordinate multiple subsystems while maintaining real-time performance and safety requirements.

Understanding these architectural fundamentals is essential for building effective humanoid robot systems that can safely interact with humans and adapt to dynamic environments. The modular approach allows for independent development and testing of subsystems while ensuring seamless integration.

In the next sections, we'll explore how to implement these concepts in practice, including creating packages, managing launch files, and modeling humanoid robots with URDF.

---

*This section covered the fundamental architecture of ROS2 with specific examples relevant to humanoid robotics. The concepts of nodes, topics, services, and actions form the backbone of all ROS2-based humanoid robot systems. The next section will dive deeper into practical package development for humanoid applications.*

*Cross-reference: The architecture concepts learned here will be essential when we explore Digital Twin implementations in Module 2 and Isaac Sim in Module 3, where similar communication patterns are used in simulation environments. The QoS and safety patterns established here directly apply to simulation systems where realistic timing and safety behaviors must be maintained.*