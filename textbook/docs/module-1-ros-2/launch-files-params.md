---
title: Launch Files and Parameters for Humanoid Systems
sidebar_position: 4
---

# Launch Files and Parameters for Humanoid Systems

## Introduction

In ROS2, launch files provide a powerful mechanism for starting multiple nodes simultaneously with predefined configurations. For humanoid robotics applications, this capability is essential as these complex systems typically require numerous coordinated nodes to function properly. A humanoid robot might need to start perception nodes, control nodes, sensor processing nodes, and communication nodes all at once, each with specific parameters tailored to the robot's configuration.

Launch files in ROS2 can be written in Python or XML, with Python being the preferred approach for more complex scenarios. They allow you to:

- Start multiple nodes with a single command
- Configure parameters for each node
- Set up remappings between topics
- Define conditional logic for different robot configurations
- Organize complex system startup procedures

For humanoid robotics, launch files become particularly important because these systems often have different operational modes (standing, walking, manipulation) that require different sets of active nodes and parameters.

**Key Terms:**
- **Launch File**: A configuration file that defines how to start multiple ROS2 nodes
- **Launch Action**: A specific operation within a launch file (e.g., start a node, set a parameter)
- **Launch Description**: The complete specification of what to launch
- **Launch Service**: A node that can start/stop other nodes dynamically

## Launch File Structure and Syntax

ROS2 launch files are typically written in Python using the `launch` package. The basic structure includes:

- **LaunchDescription**: The container for all launch actions
- **Node actions**: Define which nodes to launch with their parameters
- **Parameter files**: YAML files containing parameter configurations
- **Conditional logic**: Different configurations based on arguments

### Basic Launch File Example

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the joint state publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'rate': 50}]
        ),

        # Launch the robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher'
        ),

        # Launch the humanoid controller
        Node(
            package='humanoid_controller',
            executable='controller_node',
            name='humanoid_controller',
            parameters=[{'control_frequency': 100}]
        )
    ])
```

## Parameters and Configuration for Humanoid Systems

Parameters in ROS2 provide a flexible way to configure nodes without recompiling code. For humanoid robots, parameters are crucial for:

- Joint position and velocity limits
- Control gains and tuning parameters
- Sensor calibration values
- Safety limits and thresholds
- Walking pattern parameters
- Kinematic model parameters

### Parameter File Structure

Parameters are typically stored in YAML files with a hierarchical structure:

```yaml
/**:  # Applies to all nodes
  ros__parameters:
    use_sim_time: false
    log_level: "INFO"

humanoid_controller:
  ros__parameters:
    control_frequency: 100
    max_joint_velocity: 2.0
    joint_tolerance: 0.01
    safety_limits:
      head_pan_max: 1.57
      head_tilt_max: 0.785
      walking_step_height: 0.05
      walking_step_length: 0.3

walking_controller:
  ros__parameters:
    step_duration: 1.0
    balance_threshold: 0.02
    zmp_margin: 0.05
    gait_type: "dual_support"
```

### Parameter Management in Launch Files

Launch files can load parameters from multiple sources:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get launch configuration
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('humanoid_bringup'),
                'config',
                'default_params.yaml'
            ]),
            description='Configuration file for the humanoid robot'
        ),

        # Launch controller with parameter file
        Node(
            package='humanoid_controller',
            executable='controller_node',
            name='humanoid_controller',
            parameters=[
                config_file,  # Load from launch argument
                {'control_frequency': 200},  # Override specific parameter
            ],
            output='screen'
        )
    ])
```

## Python Launch Examples for Humanoid Robot Subsystems

Humanoid robots typically have multiple subsystems that need to be launched together. Here's a simplified example for a walking control subsystem:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Walking pattern generator
        Node(
            package='humanoid_walking',
            executable='pattern_generator',
            name='walking_pattern_generator',
            parameters=[
                {'step_height': 0.05},
                {'step_length': 0.3},
                {'step_duration': 1.0}
            ]
        ),

        # Balance controller
        Node(
            package='humanoid_walking',
            executable='balance_controller',
            name='balance_controller',
            parameters=[
                {'zmp_tolerance': 0.02},
                {'control_frequency': 200}
            ]
        )
    ])
```

## Parameter Management Techniques

Effective parameter management is crucial for humanoid robots that may operate in different environments or configurations. Here are key techniques:

### Environment-Specific Parameters

```python
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Set environment variable for robot configuration
        SetEnvironmentVariable(
            name='HUMANOID_CONFIG',
            value=EnvironmentVariable('HUMANOID_CONFIG', default_value='default')
        ),

        # Launch node with environment-specific parameters
        Node(
            package='humanoid_controller',
            executable='controller_node',
            name='controller',
            parameters=[
                {'config_type': EnvironmentVariable('HUMANOID_CONFIG')}
            ]
        )
    ])
```

### Parameter Validation

For safety-critical humanoid applications, validate parameters at launch:

```python
from launch import LaunchDescription, LaunchContext
from launch.actions import LogInfo
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Validate safety parameters before launching
        LogInfo(
            msg=['Validating safety parameters...']
        ),

        # Launch controller with validated parameters
        Node(
            package='humanoid_controller',
            executable='controller_node',
            name='safety_validated_controller',
            parameters=[
                {'max_joint_velocity': 2.0},  # Safety-limited value
                {'max_torque': 100.0},        # Safety-limited value
                {'emergency_stop_timeout': 0.1}  # 100ms timeout
            ]
        )
    ])
```

## Launch System Architecture Diagram

The following diagram illustrates the relationship between launch files, parameters, and nodes in a humanoid robot system:

```
                ROS2 Launch System Architecture
                ===============================

    Launch File (Python/XML)          Parameter Files (YAML)
    ┌─────────────────────────┐       ┌─────────────────────────┐
    │                         │       │                         │
    │  LaunchDescription      │──────▶│  robot_params.yaml      │
    │  ┌─────────────────────┐│       │  ┌─────────────────────┐│
    │  │Node Action 1        ││       │  │All Nodes:           ││
    │  │- Package: pkg1      ││       │  │  use_sim_time: false││
    │  │- Executable: node1  ││       │  │  log_level: "INFO"  ││
    │  │- Parameters: {...}  ││       │  └─────────────────────┘│
    │  └─────────────────────┘│       │                         │
    │  ┌─────────────────────┐│       │  ┌─────────────────────┐│
    │  │Node Action 2        ││       │  │Specific Node Params ││
    │  │- Package: pkg2      ││       │  │humanoid_controller: ││
    │  │- Executable: node2  ││──────▶│  │  control_frequency: ││
    │  │- Parameters: {...}  ││       │  │  100                ││
    │  └─────────────────────┘│       │  │  max_joint_velocity:││
    │  ┌─────────────────────┐│       │  │  2.0                ││
    │  │Launch Arguments     ││       │  └─────────────────────┘│
    │  │- robot_name         ││       │                         │
    │  │- use_sim_time       ││       │  ┌─────────────────────┐│
    │  └─────────────────────┘│       │  │Subsystem Params     ││
    │                         │       │  │walking_controller:  ││
    │  ┌─────────────────────┐│       │  │  step_height: 0.05  ││
    │  │Conditional Actions  ││       │  │  step_duration: 1.0 ││
    │  │IfCondition(...)     ││       │  └─────────────────────┘│
    │  └─────────────────────┘│       │                         │
    │                         │       └─────────────────────────┘
    └─────────────────────────┘
              │
              │ Launch Process
              ▼
    ┌─────────────────────────────────────────────────────────┐
    │                    ROS2 Runtime                         │
    │                                                         │
    │  ┌─────────────────┐    ┌─────────────────┐            │
    │  │  Node 1         │    │  Node 2         │            │
    │  │  (Controller)     │    │  (Sensor Proc)   │            │
    │  │  Status: Active   │    │  Status: Active │            │
    │  │  Params: ✓        │    │  Params: ✓      │            │
    │  └─────────────────┘    └─────────────────┘            │
    │                                                         │
    │  ┌─────────────────┐    ┌─────────────────┐            │
    │  │  Parameter      │    │  Launch Service │            │
    │  │  Server         │    │  (Optional)     │            │
    │  │  (Centralized    │    │                 │            │
    │  │  param storage)  │    │                 │            │
    │  └─────────────────┘    └─────────────────┘            │
    └─────────────────────────────────────────────────────────┘

Launch Process Flow:
1. Launch file defines what to start and with what parameters
2. Parameter files provide configuration values
3. Launch system starts nodes with specified parameters
4. Nodes connect and communicate via ROS2 topics/services
```

This architecture enables complex humanoid robot systems to be started with consistent configurations.

## Hands-on Exercise: Creating Launch Files for a Humanoid Robot

### Exercise Objective
Create launch files for a humanoid robot that can operate in different modes: basic operation, walking mode, and manipulation mode.

### Implementation Steps

1. **Create a main launch file** that can accept mode arguments
2. **Create parameter files** for each operational mode
3. **Implement conditional launching** based on the selected mode
4. **Test your launch system** with different configurations

### Requirements for Your Launch System
- Support at least 3 operational modes (basic, walking, manipulation)
- Include appropriate safety parameters for each mode
- Use parameter files to separate configuration from launch logic
- Implement proper error handling for missing parameters

### Sample Solution Approach
Consider how you would structure your launch system to handle:
- Different node sets for each operational mode
- Mode-specific parameters and safety limits
- Parameter validation before node startup
- Graceful fallback when parameters are missing

## Summary

Launch files and parameters are fundamental to managing the complexity of humanoid robot systems. They provide a structured way to start multiple coordinated nodes with appropriate configurations, enabling different operational modes and robot configurations from a single interface.

Proper use of launch files and parameters enables humanoid robots to adapt to different environments and tasks while maintaining safety and operational integrity. The modular approach allows for testing individual subsystems while supporting full system integration.

---

*This section covered launch files and parameters with specific focus on humanoid robotics applications. The concepts learned here will be essential when we explore Digital Twin implementations in Module 2 and Isaac Sim in Module 3, where similar launch and parameter systems are used for simulation environments.*

*Cross-reference: The launch and parameter management principles covered in this section form the foundation for creating simulation launch systems in upcoming modules.*