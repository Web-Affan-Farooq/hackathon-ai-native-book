---
title: URDF Modeling for Humanoid Robots
sidebar_position: 6
---


# URDF Modeling for Humanoid Robots

## Introduction to Humanoid URDF

**URDF (Unified Robot Description Format)** is an XML-based format used in ROS to describe robot models with precision and detail. For humanoid robots, URDF becomes particularly critical as it defines the complete physical structure, kinematic properties, dynamic characteristics, and visual appearance of the robot. A humanoid robot typically consists of multiple interconnected links (body parts) connected by joints (motors and actuators), and URDF provides the comprehensive framework to model these complex relationships accurately.

In humanoid robotics applications, URDF serves several essential functions that directly impact robot performance and safety:

- **Kinematic modeling**: Defines how different body parts connect and move relative to each other in precise mathematical relationships
- **Dynamic simulation**: Provides mass, inertia, and friction properties for accurate physics simulation and control
- **Visualization**: Describes how the robot appears in RViz, Gazebo, and other visualization environments
- **Collision detection**: Defines collision boundaries and properties for safe robot operation and path planning
- **Control system integration**: Enables forward and inverse kinematics calculations for coordinated movement

When creating URDF for humanoid robots, special attention must be paid to the unique characteristics of bipedal systems, including balance considerations, complex multi-chain kinematics, coordinated multi-joint movements, and the specific requirements for stable locomotion and manipulation tasks.

**Key Terms:**
- **URDF**: Unified Robot Description Format, the XML-based robot modeling language
- **Link**: A rigid body part of the robot (e.g., torso, arm, leg) with associated physical properties
- **Joint**: A connection between two links that allows relative motion with specified degrees of freedom
- **Kinematic Chain**: A sequence of links connected by joints forming a functional movement system
- **End Effector**: The terminal part of a kinematic chain (e.g., hand, foot) that interacts with the environment
- **Inertial Properties**: Mass, center of mass, and inertia tensor values for dynamic simulation
- **DH Parameters**: Denavit-Hartenberg parameters for precise kinematic modeling

## Advanced URDF Structure and XML Syntax for Humanoid Applications

A humanoid robot URDF file follows a sophisticated hierarchical structure with specialized XML elements designed for complex multi-limb systems. The comprehensive structure includes:

- **Robot element**: The root element containing the entire robot description with global properties
- **Link elements**: Define rigid body parts with visual, collision, and detailed inertial properties
- **Joint elements**: Define connections between links with specific motion types, limits, and dynamics
- **Material elements**: Define visual appearance properties including colors and textures
- **Transmission elements**: Define how actuators connect to joints for control system integration
- **Gazebo elements**: Define simulation-specific properties for physics engines and sensors

### Comprehensive URDF Structure Example

```xml
<?xml version="1.0"?>
<robot name="advanced_humanoid_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Include common macros and definitions -->
  <xacro:include filename="$(find humanoid_description)/urdf/materials.urdf.xacro"/>
  <xacro:include filename="$(find humanoid_description)/urdf/transmissions.urdf.xacro"/>

  <!-- Define global properties and constants -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="PI_2" value="1.570796327" />

  <!-- Robot base properties -->
  <xacro:property name="base_mass" value="10.0" />
  <xacro:property name="base_length" value="0.6" />
  <xacro:property name="base_radius" value="0.2" />

  <!-- Define materials -->
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  <material name="red">
    <color rgba="0.8 0.0 0.0 1.0"/>
  </material>
  <material name="white">
    <color rgba="0.9 0.9 0.9 1.0"/>
  </material>

  <!-- Robot base/torso link with complete properties -->
  <link name="base_link">
    <!-- Visual properties for rendering -->
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder length="${base_length}" radius="${base_radius}"/>
      </geometry>
      <material name="blue"/>
    </visual>

    <!-- Collision properties for physics simulation -->
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder length="${base_length}" radius="${base_radius}"/>
      </geometry>
    </collision>

    <!-- Inertial properties for dynamics simulation -->
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="${base_mass}"/>
      <inertia
        ixx="${base_mass/12.0 * (3*base_radius*base_radius + base_length*base_length)}"
        ixy="0.0" ixz="0.0"
        iyy="${base_mass/12.0 * (3*base_radius*base_radius + base_length*base_length)}"
        iyz="0.0"
        izz="${base_mass/2.0 * base_radius*base_radius}"/>
    </inertial>
  </link>

  <!-- Define joints with complete specifications -->
  <joint name="torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 ${base_length/2}" rpy="0 0 0"/>
    <axis xyz="0 0 0"/>
  </joint>
</robot>
```

### Link Properties and Safety Considerations

Each link in a humanoid robot URDF contains three critical property sections that must be carefully specified for safety and performance:

**Visual Properties**: Defines how the link appears in visualization tools, including geometry, materials, and textures. For humanoid robots, visual properties should accurately represent the physical robot to ensure proper simulation-to-reality transfer.

**Collision Properties**: Defines the collision boundaries and shapes for physics simulation. For humanoid robots, collision properties are crucial for safety, as they determine how the robot interacts with its environment and prevents collisions with obstacles and humans.

**Inertial Properties**: Defines mass, center of mass, and inertia tensor values for dynamics calculations. For humanoid robots, accurate inertial properties are essential for balance control, walking stability, and safe motion planning.

### Joint Types and Specifications for Humanoid Robots

Humanoid robots require sophisticated joint types to achieve human-like motion with safety and precision:

- **Revolute joints**: Rotate around a single axis with position limits, commonly used for human-like joints
- **Continuous joints**: Rotate continuously around an axis, useful for wheels or unconstrained rotation
- **Prismatic joints**: Linear sliding motion, used for telescoping mechanisms
- **Fixed joints**: Connect links with no relative motion, used for rigid attachments
- **Floating joints**: 6 degrees of freedom, rarely used but available for special applications

## Advanced Humanoid Joint Configurations and Kinematic Modeling

Humanoid robots have sophisticated joint configurations that must closely mimic human anatomy while providing the necessary range of motion and safety for robotic applications. These configurations require careful modeling to ensure realistic movement and proper kinematics.

### Upper Body Configuration with Safety Considerations

The upper body of a humanoid robot requires precise joint configuration for manipulation and interaction tasks:

**Neck and Head System**:
- **Neck joint**: Typically 2-3 degrees of freedom for head movement (pan and tilt, sometimes with roll)
- **Head link**: Contains sensors (cameras, IMUs) and requires careful mass distribution
- **Joint limits**: Constrained to prevent damage to cables and ensure safe operation

**Shoulder Complex**:
- **Shoulder abduction/adduction**: Side-to-side movement of the arm
- **Shoulder flexion/extension**: Forward/backward movement of the arm
- **Shoulder rotation**: Internal/external rotation of the arm
- **Safety limits**: Constrained to prevent self-collision and maintain balance

**Arm Configuration**:
- **Elbow joint**: Single degree of freedom for forearm flexion/extension
- **Wrist joints**: 2-3 degrees of freedom for hand positioning and orientation
- **Hand/End effector**: Multiple joints for grasping and manipulation

### Lower Body Configuration for Bipedal Locomotion

The lower body configuration is critical for stable bipedal locomotion and balance:

**Hip Complex**:
- **Hip abduction/adduction**: Side-to-side leg movement
- **Hip flexion/extension**: Forward/backward leg movement
- **Hip rotation**: Internal/external leg rotation
- **Safety considerations**: Limits prevent excessive strain on the robot's structure

**Leg Configuration**:
- **Knee joint**: Single degree of freedom for leg extension/flexion
- **Ankle joints**: 2 degrees of freedom for foot positioning and balance
- **Foot links**: Provide stable contact with the ground surface

### Joint Limitations, Dynamics, and Safety Integration

When modeling humanoid joints, it's crucial to integrate comprehensive joint limits, dynamic properties, and safety considerations:

```xml
<!-- Advanced joint definition with complete safety and dynamics parameters -->
<joint name="left_knee_joint" type="revolute">
  <parent link="left_thigh"/>
  <child link="left_shin"/>
  <origin xyz="0 0 -0.4" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit
    lower="0.0"           <!-- Minimum knee extension (fully extended) -->
    upper="2.2"           <!-- Maximum knee flexion (close to body) -->
    effort="100.0"        <!-- Maximum torque (Nm) -->
    velocity="5.0"        <!-- Maximum angular velocity (rad/s) -->
    stiffness="1000.0"    <!-- Joint stiffness for control -->
    damping="50.0"/>      <!-- Joint damping for smooth motion -->

  <!-- Safety and control parameters -->
  <safety_controller
    k_position="100"
    k_velocity="10"
    soft_lower_limit="${-0.1}"
    soft_upper_limit="${2.3}"/>

  <!-- Joint calibration if needed -->
  <calibration rising="0.0"/>
</joint>
```

## Advanced Kinematic Chain Design and Analysis for Humanoid Systems

Kinematic chains in humanoid robots define how movement propagates from one body part to another, and understanding these chains is essential for creating functional and safe humanoid models.

### Leg Kinematic Chain for Bipedal Locomotion

The leg represents a complex kinematic chain in humanoid robots that must support both static balance and dynamic walking:

```
                    Humanoid Leg Kinematic Chain
                    ============================

Base Link (Torso) → Hip Joint → Thigh Link → Knee Joint → Shin Link → Ankle Joint → Foot Link

Degrees of Freedom:
- Hip: 3 DOF (abduction/adduction, flexion/extension, rotation)
- Knee: 1 DOF (flexion/extension)
- Ankle: 2 DOF (pitch, roll)

Functional Requirements:
- Support phase: Maintain stable ground contact during stance
- Swing phase: Clear ground during leg swing for walking
- Balance: Adjust foot position to maintain center of mass
- Safety: Prevent excessive joint angles and torques
```

For walking robots, the kinematic chain must be designed to support multiple phases of locomotion:

- **Stance phase**: When the foot is in contact with the ground and supports the robot's weight
- **Swing phase**: When the leg moves forward in preparation for the next step
- **Double support**: When both feet contact the ground during gait transitions
- **Balance adjustment**: Continuous adjustment of foot placement for stability

### Arm Kinematic Chain for Manipulation

The arm chain enables reaching and manipulation tasks with dexterous control:

```
                    Humanoid Arm Kinematic Chain
                    ============================

Torso Link → Shoulder Joint → Upper Arm → Elbow Joint → Forearm → Wrist Joint → Hand

Degrees of Freedom:
- Shoulder: 3 DOF (complex multi-axis joint)
- Elbow: 1 DOF (flexion/extension)
- Wrist: 2-3 DOF (rotation and orientation)

Workspace Analysis:
- Reachable workspace: Volume where the hand can be positioned
- Dexterous workspace: Volume where the hand can achieve any orientation
- Singularity analysis: Identification of joint configurations with reduced mobility
```

The kinematic design must consider multiple factors for effective manipulation:

- **Workspace**: The reachable volume of the hand with full dexterity
- **Dexterity**: Ability to orient the hand in various directions at any position
- **Obstacle avoidance**: Prevention of self-collision and environmental collisions
- **Singularity avoidance**: Maintaining full mobility throughout the workspace
- **Load capacity**: Ensuring sufficient strength for manipulation tasks

### Advanced Inverse Kinematics for Humanoid Robots

Humanoid robots require sophisticated inverse kinematics solutions to coordinate multiple kinematic chains:

```xml
<!-- Example of a complex humanoid with multiple kinematic chains -->
<robot name="full_humanoid">
  <!-- Torso as central reference -->
  <link name="torso">
    <inertial>
      <mass value="8.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.5" ixy="0.0" ixz="0.0" iyy="0.6" iyz="0.0" izz="0.4"/>
    </inertial>
  </link>

  <!-- Left arm chain -->
  <joint name="left_shoulder" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.1 0.2 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-PI_2}" upper="${PI_2}" effort="50" velocity="3"/>
  </joint>

  <!-- Additional joints and links for complete humanoid model -->
  <!-- ... -->
</robot>
```

## Complete Humanoid Robot URDF Example with Safety and Control Integration

Here's a comprehensive example of a humanoid robot URDF with proper safety considerations, control integration, and realistic physical properties:

```xml
<?xml version="1.0"?>
<robot name="safety_oriented_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Include common definitions -->
  <xacro:include filename="$(find humanoid_description)/urdf/materials.urdf.xacro"/>
  <xacro:include filename="$(find humanoid_description)/urdf/transmission_utils.urdf.xacro"/>

  <!-- Define robot constants -->
  <xacro:property name="M_PI" value="3.1415926535897931"/>
  <xacro:property name="HUMANOID_HEIGHT" value="1.6"/>
  <xacro:property name="HUMANOID_MASS" value="60.0"/>

  <!-- Materials -->
  <material name="humanoid_body">
    <color rgba="0.7 0.7 0.9 1.0"/>
  </material>

  <!-- Torso - main body with sensors -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.25 0.6"/>
      </geometry>
      <material name="humanoid_body"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.25 0.6"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.2" ixy="0.0" ixz="0.0" iyy="0.3" iyz="0.0" izz="0.15"/>
    </inertial>
  </link>

  <!-- Head with sensors -->
  <link name="head">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="humanoid_body"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.004" ixy="0.0" ixz="0.0" iyy="0.004" iyz="0.0" izz="0.004"/>
    </inertial>
  </link>

  <!-- Neck joint -->
  <joint name="neck_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="${-M_PI/3}" upper="${M_PI/3}" effort="10" velocity="2"/>
  </joint>

  <!-- Left leg chain -->
  <link name="left_hip">
    <visual>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.06" length="0.2"/>
      </geometry>
      <material name="humanoid_body"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.06" length="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <inertia ixx="0.002" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Hip joint with safety limits -->
  <joint name="left_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_hip"/>
    <origin xyz="-0.05 0.1 -0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="150" velocity="2"/>
    <safety_controller k_position="100" k_velocity="10"/>
  </joint>

  <!-- Additional leg segments... -->
  <!-- Right leg, arms, and other components would continue similarly -->

  <!-- Transmission definitions for ROS control -->
  <transmission name="left_hip_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="left_hip_joint">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="left_hip_motor">
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>

  <!-- Gazebo-specific properties -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
  </gazebo>
</robot>
```

## Hands-on Exercise: Complete Humanoid Robot URDF Development

### Exercise Objective
Design and implement a complete humanoid robot URDF model that includes proper kinematic chains, realistic physical properties, safety considerations, and control system integration for a functional humanoid robot capable of basic walking and manipulation.

### Scenario Description
Create a humanoid robot URDF model that:

1. **Implements complete kinematic chains** for both arms and legs with proper joint configurations
2. **Includes realistic physical properties** with accurate mass, inertia, and collision models
3. **Integrates safety considerations** with appropriate joint limits and safety controllers
4. **Provides control system interfaces** with proper transmission definitions
5. **Ensures simulation compatibility** with Gazebo physics engine integration

### Implementation Requirements

**1. Complete Robot Structure:**
- Torso with central processing unit and battery placement
- Head with sensor mounting (cameras, IMUs)
- Two arms with 7+ degrees of freedom each
- Two legs with 6 degrees of freedom each
- Proper mass distribution and center of mass considerations

**2. Safety and Control Integration:**
- Joint limits that prevent damage and ensure stability
- Safety controllers with soft limits and velocity constraints
- Transmission definitions for ROS control integration
- Collision properties that prevent self-collision

**3. Physical Accuracy:**
- Realistic mass properties for each link
- Accurate inertia tensors calculated from geometry
- Proper center of mass placement for balance
- Appropriate friction and contact properties

**4. Simulation Compatibility:**
- Gazebo plugin definitions for sensors
- Physics engine parameters for stable simulation
- Visual properties for proper rendering
- Collision shapes optimized for performance

### Sample Implementation Approach

```xml
<!-- Complete humanoid skeleton with all necessary components -->
<robot name="complete_humanoid_robot">
  <!-- Implementation would include all the components described above -->
  <!-- Multiple kinematic chains, safety systems, and control integration -->
</robot>
```

### Assessment Criteria
- Proper kinematic chain implementation with realistic joint configurations
- Accurate physical properties with realistic mass distribution
- Comprehensive safety integration with joint limits and controllers
- Proper control system interfaces with transmission definitions
- Simulation compatibility with Gazebo physics engine
- Clean, maintainable URDF structure using Xacro for reusability

## Summary

URDF modeling for humanoid robots requires meticulous attention to kinematic structure, dynamic properties, safety considerations, and control system integration. The hierarchical nature of URDF allows for complex multi-chain systems like humanoid robots, where each limb operates as an independent kinematic chain connected to the central torso while maintaining coordinated movement capabilities.

Proper URDF modeling is fundamental to successful humanoid robot simulation, control, and visualization. The structure must accurately represent the physical robot while enabling the complex coordinated movements required for humanoid behaviors, including stable bipedal locomotion, dexterous manipulation, and safe human interaction.

The advanced modeling techniques described in this section enable humanoid robots to achieve realistic movement patterns, maintain stability during complex tasks, and operate safely in human environments. Proper URDF models form the foundation for all subsequent control, planning, and simulation work in humanoid robotics applications.

---

*This section covered comprehensive URDF modeling with specific focus on safety, kinematic accuracy, and control system integration for humanoid robotics applications. The concepts of structured URDF design, safety integration, and multi-chain kinematics form the foundation for creating realistic humanoid robot models. The next section will explore practical exercises combining all ROS2 concepts learned in this module.*

*Cross-reference: The URDF modeling principles covered in this section directly apply to simulation environments in Module 2, where these models are used extensively in Gazebo and other physics engines. The safety considerations and control integration patterns established here are essential for creating stable and reliable simulation environments that accurately represent real-world humanoid robot behavior.*