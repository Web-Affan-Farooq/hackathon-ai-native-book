# Terminology Reference for ROS2 Foundations Module

## Core ROS2 Concepts

- **Node**: A process that performs computation. In ROS2, nodes are organized into a graph and communicate with other nodes using topics, services, and actions.
- **Topic**: A named bus over which nodes exchange messages. Topics implement a "publish/subscribe" communication pattern where publishers send messages to a topic and subscribers receive messages from a topic.
- **Service**: A synchronous request/response communication pattern between two nodes. One node provides a service, while another node calls the service and waits for a response.
- **Action**: A communication pattern for long-running tasks that provides feedback during execution and can be canceled. Actions are useful for goals that take time to complete.
- **Package**: A filesystem organization of ROS2 code. Packages contain source code, data, and configuration files organized in a standard way to promote reusability.
- **Launch File**: An XML or Python file that starts multiple nodes simultaneously with pre-configured parameters, allowing for complex system startup.

## Humanoid Robotics Terms

- **Humanoid Robot**: A robot with human-like form and capabilities, typically featuring a head, torso, two arms, and two legs.
- **URDF (Unified Robot Description Format)**: An XML format for representing a robot model, including links, joints, and other properties that define the robot's physical structure.
- **Kinematic Chain**: A series of rigid bodies connected by joints that define the motion of a robot mechanism.
- **Bipedal**: Having two legs, referring to walking or standing on two legs as humans do.

## ROS2 Architecture Terms

- **DDS (Data Distribution Service)**: The middleware layer that ROS2 uses for message passing between nodes, providing discovery, serialization, and transportation services.
- **RMW (ROS Middleware)**: The layer that abstracts the underlying middleware implementation (like DDS) to provide a consistent interface to ROS2.
- **Composition**: The practice of combining multiple nodes into a single process to improve performance and reduce overhead.
- **Parameters**: Configuration values that can be set at runtime and changed dynamically during node execution.

## Python-Specific ROS2 Terms

- **rclpy**: The Python client library for ROS2, providing the standard interface for Python programs to interact with ROS2.
- **Message**: Data structures used when publishing to or subscribing from a topic.
- **Service Message**: Data structures used when making service requests and receiving responses.
- **Action Message**: Data structures used for action communication, including goal, result, and feedback messages.

## Module-Specific Conventions

- Always refer to "ROS2" (not "ROS 2" or "ROS-2")
- Use "humanoid robot" as a compound noun when referring to the type of robot
- When referring to the robot's parts, use "arm", "leg", "head", "torso" consistently
- For ROS2 concepts, use the official terminology from the ROS2 documentation
- Code examples should use the `rclpy` Python library