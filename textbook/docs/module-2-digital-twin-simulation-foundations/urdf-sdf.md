---
title: URDF to SDF Conversion for Humanoid Robots
sidebar_position: 3
---

# URDF to SDF Conversion for Humanoid Robots

## Introduction to URDF and SDF Formats

The Unified Robot Description Format (URDF) and Simulation Description Format (SDF) are both XML-based formats used to describe robot models, but they serve different purposes in the robotics pipeline. URDF is the native format for ROS and is primarily designed for describing robot kinematics and visual properties. SDF, on the other hand, is the native format for Gazebo/Ignition simulation environments and includes additional information needed for accurate physics simulation.

For humanoid robotics applications, the conversion from URDF to SDF is a critical step in preparing robot models for simulation. This conversion process involves not just translating the format, but also enhancing the model with simulation-specific properties that ensure realistic behavior in the virtual environment. The Digital Twin concept relies heavily on this conversion process, as it creates the virtual representation that mirrors the physical robot's properties and behaviors.

**Key Terms:**
- **URDF**: Unified Robot Description Format - ROS-native format for robot description
- **SDF**: Simulation Description Format - Gazebo-native format for simulation
- **Links**: Rigid body components of a robot model
- **Joints**: Connections between links that allow relative motion
- **Inertial Properties**: Mass, center of mass, and moment of inertia values
- **Collision Models**: Geometric shapes used for collision detection
- **Visual Models**: Geometric shapes used for rendering and visualization

## Structural Differences Between URDF and SDF

While both URDF and SDF describe robot models, they have different structures and capabilities:

### URDF Structure
```xml
<robot name="humanoid_robot">
  <link name="base_link">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="1.0" ixy="0" ixz="0" iyy="1.0" iyz="0" izz="1.0"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
  </link>
</robot>
```

### SDF Structure
```xml
<sdf version="1.7">
  <model name="humanoid_robot">
    <pose>0 0 0 0 0 0</pose>
    <link name="base_link">
      <inertial>
        <mass>1.0</mass>
        <pose>0 0 0 0 0 0</pose>
        <inertia>
          <ixx>1.0</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>1.0</iyy>
          <iyz>0</iyz>
          <izz>1.0</izz>
        </inertia>
      </inertial>
      <visual name="visual">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <box>
            <size>0.1 0.1 0.1</size>
          </box>
        </geometry>
      </visual>
      <collision name="collision">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <box>
            <size>0.1 0.1 0.1</size>
          </box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
```

The primary differences include:
- SDF wraps the model in a `<sdf>` tag with version information
- SDF uses `<model>` instead of `<robot>`
- SDF uses `<pose>` instead of `<origin>` for transformations
- SDF has different syntax for specifying mass and inertia values
- SDF supports multiple worlds/models in a single file
- SDF has enhanced plugin and sensor support for simulation

## Conversion Process for Humanoid Models

Converting URDF to SDF for humanoid robots requires special attention to the unique characteristics of bipedal systems. Humanoid robots have complex kinematic chains, multiple contact points with the environment, and specific balance requirements that must be preserved in the simulation.

### Basic Conversion Steps

1. **Transform the root element**: Change `<robot>` to `<model>` and wrap in `<sdf>` tag
2. **Convert coordinate systems**: Change `<origin>` elements to `<pose>` elements
3. **Adjust mass/inertia syntax**: Modify mass and inertia specifications to SDF format
4. **Add simulation-specific elements**: Include physics properties and simulation parameters

### Example Conversion Process

Here's a more complex example showing the conversion of a humanoid leg segment:

**Original URDF:**
```xml
<link name="left_leg">
  <inertial>
    <mass value="2.0"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.01"/>
  </inertial>
  <visual>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <geometry>
      <cylinder radius="0.05" length="0.4"/>
    </geometry>
  </visual>
  <collision>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <geometry>
      <cylinder radius="0.05" length="0.4"/>
    </geometry>
  </collision>
</link>

<joint name="left_knee" type="revolute">
  <parent link="left_thigh"/>
  <child link="left_leg"/>
  <origin xyz="0 0 -0.4" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  <dynamics damping="1.0" friction="0.1"/>
</joint>
```

**Converted SDF:**
```xml
<link name="left_leg">
  <inertial>
    <mass>2.0</mass>
    <pose>0 0 -0.2 0 0 0</pose>
    <inertia>
      <ixx>0.05</ixx>
      <ixy>0</ixy>
      <ixz>0</ixz>
      <iyy>0.05</iyy>
      <iyz>0</iyz>
      <izz>0.01</izz>
    </inertia>
  </inertial>
  <visual name="visual_left_leg">
    <pose>0 0 -0.2 0 0 0</pose>
    <geometry>
      <cylinder>
        <radius>0.05</radius>
        <length>0.4</length>
      </cylinder>
    </geometry>
  </visual>
  <collision name="collision_left_leg">
    <pose>0 0 -0.2 0 0 0</pose>
    <geometry>
      <cylinder>
        <radius>0.05</radius>
        <length>0.4</length>
      </cylinder>
    </geometry>
  </collision>
  <self_collide>false</self_collide>
  <enable_wind>false</enable_wind>
  <kinematic>false</kinematic>
</link>

<joint name="left_knee" type="revolute">
  <parent>left_thigh</parent>
  <child>left_leg</child>
  <pose>0 0 -0.4 0 0 0</pose>
  <axis>
    <xyz>1 0 0</xyz>
    <limit>
      <lower>-1.57</lower>
      <upper>1.57</upper>
      <effort>100</effort>
      <velocity>1.0</velocity>
    </limit>
    <dynamics>
      <damping>1.0</damping>
      <friction>0.1</friction>
    </dynamics>
  </axis>
</joint>
```

## Humanoid-Specific Conversion Considerations

Humanoid robots have unique requirements that must be carefully addressed during the URDF to SDF conversion process:

### Joint Configuration for Humanoid Systems

Humanoid robots typically have multiple degrees of freedom in their limbs, requiring careful attention to:

- **Joint limits**: Properly configured limits that reflect the physical robot's capabilities
- **Effort limits**: Appropriate torque limits for humanoid joints
- **Velocity limits**: Realistic speed constraints for humanoid movements
- **Damping and friction**: Properly tuned parameters for realistic motion

### Inertial Property Mapping

Accurate inertial properties are especially critical for humanoid robots due to their complex balance requirements:

- **Mass distribution**: Proper mass allocation across links to match physical robot
- **Center of mass**: Accurate CoM positioning for realistic balance simulation
- **Moment of inertia**: Proper tensor values for realistic rotational dynamics

For humanoid robots, special attention must be paid to the inertial properties of the torso and legs, as these significantly impact the robot's ability to maintain balance during locomotion.

### Collision and Contact Modeling

Humanoid robots interact with their environment through multiple contact points, requiring careful collision modeling:

- **Foot contact**: Accurate collision geometry for feet to enable stable walking
- **Hand contact**: Proper collision models for manipulation tasks
- **Self-collision**: Prevention of unwanted link collisions during movement
- **Contact parameters**: Friction and restitution coefficients appropriate for humanoid locomotion

## Enhanced SDF Elements for Simulation

Beyond the basic conversion, SDF files for humanoid robots often include additional elements that enhance simulation realism:

### Physics Properties
```xml
<physics type="dart">
  <gravity>0 0 -9.8</gravity>
  <solver>
    <type>PGS</type>
    <iterations>1000</iterations>
    <precon_iters>0</precon_iters>
    <contact_surface_layer>0.001</contact_surface_layer>
    <contact_max_correcting_vel>100</contact_max_correcting_vel>
    <cfm>0</cfm>
    <erp>0.2</erp>
    <max_contacts>20</max_contacts>
  </solver>
  <constraints>
    <contact_surface_layer>0.001</contact_surface_layer>
    <contact_max_correcting_vel>100</contact_max_correcting_vel>
  </constraints>
</physics>
```

### Sensor Integration
```xml
<sensor name="imu_sensor" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <pose>0 0 0 0 0 0</pose>
  <topic>imu/data</topic>
  <ignition_frame_id>base_link</ignition_frame_id>
</sensor>
```

### Plugin Configuration
```xml
<plugin name="humanoid_controller" filename="libgazebo_ros_control.so">
  <robotNamespace>/humanoid_robot</robotNamespace>
  <robotParam>/humanoid_robot/robot_description</robotParam>
</plugin>
```

## Tools for URDF to SDF Conversion

Several tools can assist with the conversion process:

1. **Direct conversion**: Using the `gz sdf` command-line tool
2. **Programmatic conversion**: Writing custom scripts to transform URDF elements
3. **Manual conversion**: For complex models requiring fine-tuning

The `gz sdf` tool can perform basic conversions:
```bash
gz sdf -p robot.urdf > robot.sdf
```

However, for humanoid robots, manual adjustment is often necessary to ensure proper physics properties and simulation behavior.

## Validation of Converted Models

After conversion, it's essential to validate that the SDF model behaves correctly in simulation:

1. **Visual inspection**: Ensure the model appears correctly in Gazebo
2. **Physics validation**: Verify that the model responds appropriately to gravity and forces
3. **Kinematic validation**: Check that joint limits and ranges are respected
4. **Contact validation**: Verify that collision detection works properly

For humanoid robots, particular attention should be paid to:
- Balance and stability in the simulation
- Proper contact behavior during walking or manipulation
- Realistic joint motion that matches the physical robot

## Summary

Converting URDF models to SDF format is a critical step in preparing humanoid robots for simulation-based development. The conversion process involves more than just format translation—it requires careful attention to physics properties, collision models, and simulation-specific parameters that ensure realistic behavior in the virtual environment. This conversion process is fundamental to Digital Twin workflows, where the virtual model must accurately reflect the physical robot's properties and behaviors.

The process requires understanding both the structural differences between the formats and the humanoid-specific requirements for realistic simulation. Proper conversion enables effective testing and validation of humanoid robot behaviors in a safe, repeatable virtual environment before deployment to physical hardware.

---

*This section covered the process of converting URDF models to SDF format with specific focus on humanoid robotics applications. The concepts learned here are essential for creating effective simulation models that accurately represent physical humanoid robots. In the next section, we'll explore physics simulation essentials that build upon these model conversion concepts to create realistic humanoid behaviors in simulation.*

*Cross-reference: The URDF/SDF conversion concepts covered in this section are fundamental to creating simulation models that will be used in Module 3 with Isaac Sim, where similar conversion and simulation principles apply.*