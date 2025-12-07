---
title: Hands-on Exercises and Integration
sidebar_position: 6
---

# Hands-on Exercises and Integration

## Introduction to Integrated Exercises

This section provides hands-on exercises that combine all the ROS2 concepts learned throughout this module, specifically applied to humanoid robotics scenarios. These exercises are designed to reinforce understanding through practical application and prepare students for simulation modules that follow.

The exercises progress from basic single-concept tasks to complex multi-concept challenges that mirror real-world humanoid robot development. Each exercise builds upon the foundational knowledge of ROS2 architecture, Python package development, launch files, and URDF modeling covered in previous sections.

Students will work with simulated humanoid robot systems, implementing control strategies, sensor processing, and coordinated movement patterns. The exercises emphasize the integration aspects that are crucial for humanoid robot operation, where multiple subsystems must work together seamlessly.

**Key Terms:**
- **Integration Exercise**: A task that combines multiple ROS2 concepts
- **Multi-concept Task**: An exercise requiring knowledge from several learning journeys
- **Progressive Difficulty**: Exercises that build in complexity from basic to advanced

## Exercise: Nodes, Topics, Services, and Actions Integration

### Exercise Objective
Implement a complete humanoid robot head control system that integrates nodes, topics, services, and actions in a coordinated manner.

### Scenario Description
You are tasked with creating a humanoid robot head control system that can:
- Track objects using camera data (topics)
- Calibrate sensors when requested (services)
- Execute complex head movement sequences (actions)
- Coordinate with other robot systems (integration)

### Implementation Requirements

1. **Node Structure**:
   - Create a `head_tracker` node that subscribes to camera image topics
   - Create a `head_controller` node that publishes joint commands
   - Create a `calibration_service` node that handles calibration requests

2. **Topic Communication**:
   - Subscribe to `/camera/image_raw` for object detection
   - Subscribe to `/joint_states` for current head position feedback
   - Publish to `/head_controller/position_commands` for head movement

3. **Service Integration**:
   - Implement a `/head_calibration` service that performs head sensor calibration
   - Return success/failure status with diagnostic information

4. **Action Implementation**:
   - Create a `/head_scan_pattern` action that executes a systematic scanning pattern
   - Provide feedback on scan progress
   - Allow cancellation of the scan if needed

### Sample Solution Approach

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from control_msgs.action import FollowJointTrajectory

class HeadTrackerNode(Node):
    def __init__(self):
        super().__init__('head_tracker')

        # Subscriptions
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )

        # Publishers
        self.cmd_pub = self.create_publisher(
            Float64MultiArray, '/head_controller/position_commands', 10
        )

        # Service server
        self.cal_srv = self.create_service(
            Trigger, '/head_calibration', self.calibrate_callback
        )

        # Action server
        self.scan_action_server = ActionServer(
            self, FollowJointTrajectory, '/head_scan_pattern', self.scan_execute
        )

        self.get_logger().info('Head tracker system initialized')

def main(args=None):
    rclpy.init(args=args)
    node = HeadTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercise: Python Packages and Launch Files Integration

### Exercise Objective
Create a complete ROS2 package for humanoid walking control and develop launch files to start the entire system with proper parameter configuration.

### Implementation Steps

1. **Package Structure**:
   - Create a package named `humanoid_walking`
   - Include proper `package.xml` and `setup.py` files
   - Organize source code in a Python module structure

2. **Node Implementation**:
   - Implement a `walking_controller` node
   - Include parameter handling for walking gait parameters
   - Add proper logging and error handling

3. **Launch File Creation**:
   - Create a main launch file `walking_system.launch.py`
   - Include parameter file loading
   - Add conditional node launching based on arguments

4. **Parameter Configuration**:
   - Create a YAML parameter file for walking parameters
   - Include step height, duration, and balance parameters
   - Set appropriate safety limits

### Package Structure Example

```
humanoid_walking/
├── CMakeLists.txt
├── package.xml
├── setup.py
├── setup.cfg
├── config/
│   └── walking_params.yaml
├── launch/
│   └── walking_system.launch.py
└── humanoid_walking/
    ├── __init__.py
    └── walking_controller.py
```

### Parameter File Example (`config/walking_params.yaml`)

```yaml
walking_controller:
  ros__parameters:
    step_height: 0.05  # 5cm step height
    step_length: 0.3   # 30cm step length
    step_duration: 1.0 # 1 second per step
    balance_threshold: 0.02  # 2cm balance tolerance
    max_step_count: 100      # Maximum steps before stopping
    enable_logging: true
```

## Exercise: Incorporating URDF into Control Systems

### Exercise Objective
Integrate a humanoid robot URDF model with a control system to create a complete simulation-ready robot.

### Implementation Requirements

1. **URDF Integration**:
   - Create a simplified humanoid URDF with at least 10 links
   - Include proper joint limits and safety constraints
   - Add visual and collision properties

2. **Robot State Publishing**:
   - Implement a robot state publisher node
   - Connect joint state messages to the URDF model
   - Visualize the robot in RViz

3. **Control Integration**:
   - Create joint trajectory controllers
   - Implement forward and inverse kinematics
   - Add safety checks based on URDF limits

### URDF Integration Example

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from urdf_parser_py.urdf import URDF

class RobotStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_state_publisher')

        # Load URDF model
        self.robot_model = URDF.from_xml_string(self.load_urdf())

        # Joint state subscription
        self.joint_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10
        )

        # Transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info('Robot state publisher initialized')

    def load_urdf(self):
        # Load URDF from file or parameter
        with open('/path/to/humanoid.urdf', 'r') as f:
            return f.read()
```

## Creating Progressive Exercises

### Progressive Difficulty Structure

The exercises in this module follow a progressive difficulty approach:

1. **Basic Level**: Single concept application
2. **Intermediate Level**: Two concept integration
3. **Advanced Level**: Three concept integration
4. **Expert Level**: Complete system integration

### Example Progressive Exercise Set

**Level 1 - Basic**: Create a simple publisher node that sends joint commands

**Level 2 - Intermediate**: Combine publisher with parameter configuration

**Level 3 - Advanced**: Add service call for calibration before movement

**Level 4 - Expert**: Implement complete walking pattern with error handling

## Full System Integration Exercise

### Exercise Objective
Implement a complete humanoid robot behavior that integrates all concepts from this module:

- ROS2 architecture (nodes, topics, services, actions)
- Python package development
- Launch file coordination
- URDF model integration

### Scenario: Humanoid Greeting Behavior

Create a system where a humanoid robot:
1. Detects a person approaching (topic communication)
2. Moves its head to look at the person (action execution)
3. Waits for a greeting command (service call)
4. Executes a waving motion (trajectory following)
5. Returns to neutral position

### System Architecture

The complete system should include:

1. **Perception Node**: Processes camera data to detect people
2. **Behavior Manager**: Coordinates the greeting sequence
3. **Head Controller**: Handles head movement for looking
4. **Arm Controller**: Executes waving motion
5. **Safety Monitor**: Ensures safe operation throughout

### Implementation Checklist

- [ ] ROS2 nodes properly architected
- [ ] Topics correctly configured for data flow
- [ ] Services implemented for command interface
- [ ] Actions created for complex behaviors
- [ ] Python package structure follows ROS2 conventions
- [ ] Launch files coordinate all nodes
- [ ] Parameters properly configured
- [ ] URDF model integrated for visualization
- [ ] Safety checks implemented
- [ ] Error handling in place

## Summary

This module has provided comprehensive hands-on experience with ROS2 concepts specifically applied to humanoid robotics. Students have learned to:

- Integrate multiple ROS2 communication patterns (topics, services, actions)
- Develop properly structured Python packages for humanoid systems
- Create coordinated launch files for complex robot systems
- Incorporate URDF models into control systems
- Design progressive exercises that build complexity gradually

These skills form the foundation for advanced robotics applications covered in subsequent modules, including simulation, digital twins, and advanced control strategies.

---

*This section combined all ROS2 concepts with specific focus on humanoid robotics applications. The concepts learned here will be essential when we explore Digital Twin implementations in Module 2 and Isaac Sim in Module 3, where integrated systems are used extensively.*

*Cross-reference: The integration principles covered in this section form the foundation for creating complex simulation environments in upcoming modules.*