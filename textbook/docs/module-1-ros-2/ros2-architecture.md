---
title: ROS2 Architecture Fundamentals
sidebar_position: 2
---

# ROS2 Architecture Fundamentals

## Introduction

**ROS2** (Robot Operating System 2) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms. In the context of **humanoid robotics**, ROS2 provides the communication infrastructure that allows different subsystems of a humanoid robot—such as perception, planning, control, and actuation—to work together seamlessly.

The architecture of ROS2 is fundamentally different from ROS1, addressing many of its predecessor's limitations. The most significant change is the adoption of **DDS (Data Distribution Service)** as the underlying middleware, which provides better support for real-time systems, security, and distributed computing—essential features for humanoid robots that operate in dynamic environments.

**Key Terms:**
- **Node**: A process that performs computation
- **Topic**: Named bus for message passing
- **Service**: Synchronous request/response pattern
- **Action**: Communication for long-running tasks

## Nodes in ROS2

A **node** is the fundamental unit of computation in ROS2. It's an executable process that performs specific tasks within the robot system. In humanoid robotics, nodes might handle functions such as:

- Joint control and motor management
- Sensor data processing (IMU, cameras, force/torque sensors)
- High-level planning and decision making
- Walking pattern generation
- State estimation and localization

Nodes in ROS2 are organized into a graph structure where they communicate with other nodes using topics, services, and actions. Each node is responsible for a specific function and can be developed, tested, and maintained independently.

In the context of humanoid robots, nodes are particularly valuable because they allow for modularity. For example, a humanoid robot might have separate nodes for:

- Head control (pan/tilt of cameras and sensors)
- Upper body control (arm and hand movements)
- Lower body control (leg movements and balance)
- Sensor fusion (combining data from multiple sensors)
- High-level behavior control

### Node Implementation in Python

In Python, nodes are created using the `rclpy` client library:

```python
import rclpy
from rclpy.node import Node

class JointControllerNode(Node):
    def __init__(self):
        super().__init__('joint_controller')
        # Initialize joint control logic

def main(args=None):
    rclpy.init(args=args)
    node = JointControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## Topics and Message Passing

**Topics** in ROS2 implement a publish/subscribe communication pattern. Publishers send messages to a topic, and subscribers receive messages from a topic. This decouples the publisher from the subscriber, allowing for flexible system design.

In humanoid robotics, topics are commonly used for:

- Sensor data streams (camera images, IMU data, joint positions)
- Control commands (desired joint positions, walking commands)
- State information (current robot pose, battery level)
- Perception results (detected objects, navigation goals)

The publish/subscribe pattern is ideal for humanoid robots because multiple systems often need the same data. For example, camera images might be needed simultaneously by perception systems, control systems, and user interfaces.

### Topic Implementation Example

Here's a simplified example of a topic publisher for joint positions in a humanoid robot:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states)  # 10Hz

    def publish_joint_states(self):
        msg = JointState()
        # Populate joint state message with current positions
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## Services for Synchronous Communication

**Services** provide a synchronous request/response communication pattern. A client sends a request to a service, and the service processes the request and sends back a response. This is useful for operations that need to complete before the client can continue.

In humanoid robotics, services are commonly used for:

- Calibration procedures
- Configuration changes
- Diagnostic requests
- Emergency stop commands
- Complex planning tasks that return a result

### Service Implementation Example

Here's a simplified example of a service that could be used for humanoid robot calibration:

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class CalibrationService(Node):
    def __init__(self):
        super().__init__('calibration_service')
        self.srv = self.create_service(Trigger, 'calibrate_robot', self.calibrate_callback)

    def calibrate_callback(self, request, response):
        # Perform calibration steps
        response.success = True
        response.message = 'Calibration completed successfully'
        return response
```

## Actions for Long-Running Tasks

**Actions** are designed for long-running tasks that provide feedback during execution and can be canceled. They're particularly useful for humanoid robotics tasks that take significant time to complete, such as:

- Walking to a location
- Performing complex manipulation tasks
- Executing multi-step behaviors
- Running extended diagnostic procedures

Actions provide three key features:
- **Goal**: The desired outcome of the action
- **Feedback**: Information about the progress toward the goal
- **Result**: The final outcome of the action

## ROS2 Architecture Diagram

The following conceptual diagram illustrates the ROS2 architecture with its core components and communication patterns:

```
                    ROS2 Architecture Overview
                    ==========================

+----------------+     +----------------+     +----------------+
|   Node A       |     |   Node B       |     |   Node C       |
|                |     |                |     |                |
| - Publisher    |<----| - Subscriber   |     | - Service      |
| - Service Client|    | - Publisher    |---->|   Server       |
| - Action Client|     | - Service      |     | - Subscriber   |
|                |     |   Server       |     |                |
+----------------+     +----------------+     +----------------+
       |                       |                       |
       | Publish to            | Subscribe to          | Provide
       | /sensor_data          | /sensor_data          | /calibrate
       |                       |                     service
       v                       v                       v
+------------------------------------------------------------+
|                    DDS Middleware Layer                    |
|  (Data Distribution Service - handles discovery,          |
|   serialization, and transportation of messages)          |
+------------------------------------------------------------+
                              |
                              | Communication via
                              | Topics, Services, Actions
                              v
+----------------+     +----------------+     +----------------+
|   Node D       |     |   Node E       |     |   Action       |
|                |     |                |     |   Server       |
| - Action       |<--->| - Parameter    |<----|                |
|   Client       |     |   Server       |    | - Execute      |
|                |     |                |    |   Actions      |
+----------------+     +----------------+     +----------------+

Key Communication Patterns:
- Topics: Publish/Subscribe (e.g., sensor data streams)
- Services: Request/Response (e.g., calibration requests)
- Actions: Goal/Feedback/Result (e.g., walking to location)
```

This diagram shows how nodes communicate through the DDS middleware layer, which handles message routing, discovery, and serialization in a distributed system.

## Humanoid-Specific Architecture Considerations

When designing ROS2 architectures for humanoid robots, several unique considerations apply:

### Safety and Redundancy
Humanoid robots operate in human environments, requiring robust safety systems. The ROS2 architecture supports this through:
- Separate safety monitoring nodes
- Multiple communication paths for critical systems
- Graceful degradation when subsystems fail

### Real-time Requirements
Humanoid robots often have strict timing requirements for control loops:
- Joint control typically requires 100Hz+ update rates
- Balance control may require even higher frequencies
- ROS2's DDS middleware provides better real-time performance than ROS1

### Distributed Processing
Humanoid robots often have distributed computing resources:
- On-board computers for high-level planning
- Microcontrollers for low-level control
- Specialized hardware for perception tasks
- ROS2's peer-to-peer architecture supports this distribution naturally

## Hands-on Exercise: Designing a Humanoid Robot Architecture

Now that you understand the core concepts of ROS2 architecture, let's apply this knowledge to design a simple humanoid robot system.

### Exercise Objective
Design a ROS2 architecture for a basic humanoid robot that can:
1. Move its head to look at objects
2. Detect objects using a camera
3. Respond to a "look_at" command to orient its head toward a detected object

### Architecture Design Task
Design the node structure and communication patterns for this system. Consider:

1. **Node Design**: Identify at least 3 nodes needed for this system and their responsibilities
2. **Topic Communication**: Identify what topics would be needed and what messages they would carry
3. **Service/Action Communication**: Determine if services or actions would be appropriate for the "look_at" command
4. **Message Types**: Specify what message types would be appropriate for each communication

### Implementation Steps

1. **Sketch your architecture**: Draw a simple diagram showing the nodes and their connections
2. **Define topics**: List the topics your nodes would publish and subscribe to
3. **Define services/actions**: Determine if the "look_at" command should be implemented as a service or action
4. **Consider safety**: How would you incorporate safety checks into your design?

### Sample Solution Approach
Think about how each of the ROS2 concepts we've discussed (nodes, topics, services, actions) would apply to this humanoid robot scenario. Consider the timing requirements, safety implications, and real-time performance needs.

## Summary

ROS2's architecture provides a robust foundation for humanoid robotics applications through its node-based design, flexible communication patterns (topics, services, actions), and modern middleware. Understanding these architectural fundamentals is essential for building effective humanoid robot systems that can safely interact with humans and adapt to dynamic environments.

In the next sections, we'll explore how to implement these concepts in practice, including creating packages, managing launch files, and modeling humanoid robots with URDF.

---

*This section covered the fundamental architecture of ROS2 with specific examples relevant to humanoid robotics. The next section will dive deeper into the communication patterns of nodes, topics, services, and actions.*

*Cross-reference: The architecture concepts learned here will be essential when we explore Digital Twin implementations in Module 2 and Isaac Sim in Module 3, where similar communication patterns are used in simulation environments.*