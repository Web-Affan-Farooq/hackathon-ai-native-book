---
title: URDF Modeling for Humanoid Robots
sidebar_position: 5
---

# URDF Modeling for Humanoid Robots

## Introduction to Humanoid URDF

URDF (Unified Robot Description Format) is an XML-based format used in ROS to describe robot models. For humanoid robots, URDF becomes particularly important as it defines the physical structure, kinematic properties, and visual appearance of the robot. A humanoid robot typically consists of multiple links (body parts) connected by joints (motors), and URDF provides the framework to model these relationships.

In humanoid robotics applications, URDF serves several critical functions:

- **Kinematic modeling**: Defines how different body parts connect and move relative to each other
- **Dynamic simulation**: Provides mass, inertia, and friction properties for physics simulation
- **Visualization**: Describes how the robot appears in RViz and simulation environments
- **Collision detection**: Defines collision boundaries for safe robot operation

When creating URDF for humanoid robots, special attention must be paid to the unique characteristics of bipedal systems, including balance considerations, complex kinematic chains, and the need for coordinated multi-joint movements.

**Key Terms:**
- **Link**: A rigid body part of the robot (e.g., torso, arm, leg)
- **Joint**: A connection between two links that allows relative motion
- **Kinematic Chain**: A sequence of links connected by joints
- **End Effector**: The terminal part of a kinematic chain (e.g., hand, foot)

## URDF Structure and XML Syntax

A humanoid robot URDF file follows a hierarchical structure with specific XML elements. The basic structure includes:

- **Robot element**: The root element containing the entire robot description
- **Link elements**: Define rigid body parts with visual, collision, and inertial properties
- **Joint elements**: Define connections between links with specific motion types
- **Material elements**: Define visual appearance properties

### Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- Define materials -->
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>

  <!-- Define links -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Define joints -->
  <joint name="torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
  </joint>
</robot>
```

### Link Properties

Each link in a humanoid robot URDF contains three main property sections:

- **Visual**: Defines how the link appears in visualization tools
- **Collision**: Defines the collision boundaries for physics simulation
- **Inertial**: Defines mass and inertia properties for dynamics calculations

### Joint Types for Humanoid Robots

Humanoid robots require various joint types to achieve human-like motion:

- **Revolute joints**: Rotate around a single axis (like human joints)
- **Fixed joints**: Connect links with no relative motion
- **Continuous joints**: Rotate continuously around an axis
- **Prismatic joints**: Linear sliding motion (less common in humanoid robots)

## Humanoid Joint Configurations

Humanoid robots have specific joint configurations that mimic human anatomy. These configurations must be carefully modeled to ensure realistic movement and proper kinematics.

### Upper Body Configuration

The upper body of a humanoid typically includes:

- **Neck joint**: Usually 1-2 degrees of freedom for head movement
- **Shoulder joints**: 3 degrees of freedom for arm movement
- **Elbow joints**: 1 degree of freedom for forearm rotation
- **Wrist joints**: 2-3 degrees of freedom for hand positioning
- **Hand joints**: Multiple joints for finger movement (optional in basic models)

### Lower Body Configuration

The lower body configuration for bipedal locomotion includes:

- **Hip joints**: 3 degrees of freedom for leg movement
- **Knee joints**: 1 degree of freedom for leg extension
- **Ankle joints**: 2 degrees of freedom for foot positioning
- **Foot joints**: Fixed connection to end effector

### Joint Limitations and Safety

When modeling humanoid joints, it's crucial to set appropriate joint limits that reflect both mechanical constraints and safety requirements:

```xml
<joint name="left_knee_joint" type="revolute">
  <parent link="left_thigh"/>
  <child link="left_shin"/>
  <origin xyz="0 0 -0.4" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="-0.05" upper="2.0" effort="30" velocity="1.0"/>
</joint>
```

## Kinematic Chain Examples

Kinematic chains in humanoid robots define how movement propagates from one body part to another. Understanding these chains is essential for creating functional humanoid models.

### Leg Kinematic Chain

The leg represents a typical kinematic chain in humanoid robots:

```
Base Link → Pelvis → Thigh → Shin → Foot
```

Each joint in this chain contributes to the final position and orientation of the foot (end effector). For walking robots, the kinematic chain must be designed to support:

- **Stance phase**: When the foot is in contact with the ground
- **Swing phase**: When the leg moves forward in preparation for the next step
- **Balance**: Maintaining center of mass within support polygon

### Arm Kinematic Chain

The arm chain typically follows:

```
Torso → Shoulder → Upper Arm → Forearm → Hand
```

This chain allows for reaching and manipulation tasks. The kinematic design must consider:

- **Workspace**: The reachable volume of the hand
- **Dexterity**: Ability to orient the hand in various directions
- **Obstacle avoidance**: Preventing self-collision during movement

## Humanoid-Specific URDF Examples

Here's a simplified example of a humanoid robot lower body in URDF format:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid_lower_body">
  <!-- Torso (base for leg chains) -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.2" ixy="0.0" ixz="0.0" iyy="0.3" iyz="0.0" izz="0.4"/>
    </inertial>
  </link>

  <!-- Left Leg -->
  <link name="left_hip">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <link name="left_thigh">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <link name="left_shin">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.04" ixy="0.0" ixz="0.0" iyy="0.04" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <link name="left_foot">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Joints for left leg -->
  <joint name="left_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_hip"/>
    <origin xyz="0 0.1 -0.2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.0" upper="1.0" effort="50" velocity="2.0"/>
  </joint>

  <joint name="left_thigh_joint" type="revolute">
    <parent link="left_hip"/>
    <child link="left_thigh"/>
    <origin xyz="0 0 -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="0.5" effort="100" velocity="2.0"/>
  </joint>

  <joint name="left_knee_joint" type="revolute">
    <parent link="left_thigh"/>
    <child link="left_shin"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.0" effort="80" velocity="2.0"/>
  </joint>

  <joint name="left_ankle_joint" type="revolute">
    <parent link="left_shin"/>
    <child link="left_foot"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="40" velocity="2.0"/>
  </joint>
</robot>
```

## Hands-on Exercise: Creating a Basic Humanoid URDF

### Exercise Objective
Create a simplified humanoid robot URDF with at least a torso, two arms, and two legs using proper kinematic chains.

### Implementation Steps

1. **Define the base link** (torso) with appropriate visual and inertial properties
2. **Create the right arm chain** with shoulder, elbow, and wrist joints
3. **Create the left arm chain** mirroring the right arm
4. **Create the leg chains** with hip, knee, and ankle joints
5. **Add proper joint limits** that reflect human-like movement ranges
6. **Validate your URDF** using a URDF checker tool

### Requirements for Your URDF Model
- Include at least 10 links (torso, head, 4 arms/legs segments, 2 feet, 2 hands)
- Use appropriate joint types for each connection
- Set realistic mass and inertia values
- Define collision properties for each link
- Include proper joint limits to prevent damage

### Sample Solution Approach
Consider how you would structure your model to handle:
- Balance during single and double support phases
- Reachable workspace for manipulation tasks
- Proper kinematic chains for coordinated movement
- Collision avoidance between body parts

## Summary

URDF modeling for humanoid robots requires careful attention to kinematic structure, dynamic properties, and safety considerations. The hierarchical nature of URDF allows for complex multi-chain systems like humanoid robots, where each limb operates as an independent kinematic chain connected to the central torso.

Proper URDF modeling is fundamental to successful humanoid robot simulation, control, and visualization. The structure must accurately represent the physical robot while enabling the complex coordinated movements required for humanoid behaviors.

---

*This section covered URDF modeling with specific focus on humanoid robotics applications. The concepts learned here will be essential when we explore Digital Twin implementations in Module 2 and Isaac Sim in Module 3, where URDF models are used extensively for simulation environments.*

*Cross-reference: The URDF modeling principles covered in this section form the foundation for creating simulation models in upcoming modules.*