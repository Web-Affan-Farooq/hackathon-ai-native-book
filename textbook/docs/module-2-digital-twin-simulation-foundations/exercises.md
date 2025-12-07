---
title: Hands-on Exercises — Integrating ROS2 and Simulation Concepts
sidebar_position: 5
---

# Hands-on Exercises — Integrating ROS2 and Simulation Concepts

## Introduction to Integrated Exercises

This section provides hands-on exercises that integrate all the concepts learned in Module 2, connecting ROS2 systems from Module 1 with Gazebo simulation environments. These exercises are designed to reinforce understanding through practical application and prepare students for simulation-to-real transfer in upcoming modules.

The exercises follow a progressive difficulty approach, building on concepts from earlier learning journeys while remaining accessible as independent activities. Each exercise connects to the Digital Twin concept by demonstrating how virtual models can be used to test and validate behaviors before considering real-world deployment.

**Exercise Types:**
- **Conceptual Exercises**: Focus on understanding fundamental concepts
- **Practical Exercises**: Involve hands-on implementation and experimentation
- **Integration Exercises**: Combine multiple concepts in complex scenarios
- **Simulation-to-Real Exercises**: Bridge simulation concepts to real-world applications

## Exercise 1: Digital Twin Foundation Exercise

### Objective
Create a basic Digital Twin scenario where a simple robot model is represented in both simulation and connected to a ROS2 control system.

### Prerequisites
- Understanding of ROS2 nodes, topics, and services
- Basic knowledge of Gazebo simulation environment
- Access to ROS2 and Gazebo installation

### Exercise Description
In this exercise, you'll create a simple Digital Twin scenario where a virtual robot in Gazebo is controlled by a ROS2 node. This represents the fundamental concept of a Digital Twin where the virtual model can be controlled and monitored remotely.

### Steps

1. **Create a simple robot model in URDF**:
   Create a basic robot with a single rotating joint that can be controlled via ROS2.

   ```xml
   <?xml version="1.0"?>
   <robot name="simple_twins_robot">
     <link name="base_link">
       <visual>
         <geometry>
           <cylinder length="0.2" radius="0.1"/>
         </geometry>
         <material name="blue">
           <color rgba="0 0 1 1"/>
         </material>
       </visual>
       <collision>
         <geometry>
           <cylinder length="0.2" radius="0.1"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="1.0"/>
         <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
       </inertial>
     </link>

     <link name="rotating_arm">
       <visual>
         <geometry>
           <box size="0.05 0.05 0.3"/>
         </geometry>
         <material name="red">
           <color rgba="1 0 0 1"/>
         </material>
       </visual>
       <collision>
         <geometry>
           <box size="0.05 0.05 0.3"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="0.2"/>
         <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
       </inertial>
     </link>

     <joint name="arm_joint" type="continuous">
       <parent link="base_link"/>
       <child link="rotating_arm"/>
       <origin xyz="0 0 0.15" rpy="0 0 0"/>
       <axis xyz="0 0 1"/>
     </joint>
   </robot>
   ```

2. **Convert the URDF to SDF format**:
   Use the `gz sdf` command to convert your URDF to SDF format:
   ```bash
   gz sdf -p simple_robot.urdf > simple_robot.sdf
   ```

3. **Create a ROS2 controller node**:
   Write a simple ROS2 node that publishes joint commands to control the rotating arm:

   ```python
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import Float64MultiArray
   import math
   import time

   class TwinController(Node):
       def __init__(self):
           super().__init__('twin_controller')
           self.publisher = self.create_publisher(Float64MultiArray, '/joint_commands', 10)
           self.timer = self.create_timer(0.1, self.publish_command)  # 10Hz
           self.time = 0.0
           self.get_logger().info('Digital Twin Controller Initialized')

       def publish_command(self):
           msg = Float64MultiArray()
           # Create a sinusoidal motion for the joint
           joint_position = math.sin(self.time)  # Position between -1 and 1
           msg.data = [joint_position]
           self.publisher.publish(msg)
           self.time += 0.1
           self.get_logger().info(f'Published joint command: {joint_position:.2f}')

   def main(args=None):
       rclpy.init(args=args)
       node = TwinController()

       try:
           rclpy.spin(node)
       except KeyboardInterrupt:
           pass
       finally:
           node.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

4. **Set up the simulation environment**:
   Create a simple world file for Gazebo:

   ```xml
   <?xml version="1.0" ?>
   <sdf version="1.7">
     <world name="digital_twin_world">
       <include>
         <uri>model://ground_plane</uri>
       </include>

       <include>
         <uri>model://sun</uri>
       </include>

       <physics type="ode">
         <gravity>0 0 -9.8</gravity>
         <max_step_size>0.001</max_step_size>
         <real_time_factor>1.0</real_time_factor>
       </physics>
     </world>
   </sdf>
   ```

5. **Connect the ROS2 controller to the simulation**:
   Use the ROS-Gazebo bridge to connect your controller to the simulated robot.

### Expected Outcome
The virtual robot's arm should rotate in a smooth, sinusoidal pattern, demonstrating the connection between the ROS2 control system and the simulation environment. This represents a basic Digital Twin where the virtual model responds to commands from the control system.

### Connection to Future Modules
This exercise establishes the foundation for more complex Digital Twin scenarios that will be explored in Module 3 with Isaac Sim, where similar connection principles apply but with more sophisticated simulation environments.

## Exercise 2: Gazebo Simulation Exercise

### Objective
Create and run a complete Gazebo simulation environment with a humanoid-inspired robot model, demonstrating core Gazebo concepts with humanoid applications.

### Prerequisites
- Completion of Exercise 1
- Understanding of Gazebo world and model structure
- Knowledge of URDF/SDF conversion process

### Exercise Description
In this exercise, you'll create a more complex simulation environment that incorporates humanoid-inspired elements and demonstrates Gazebo's capabilities for humanoid robotics applications.

### Steps

1. **Create a humanoid-inspired model**:
   Design a simple humanoid model with basic joints that could represent a simplified humanoid robot:

   ```xml
   <?xml version="1.0"?>
   <robot name="simple_humanoid">
     <!-- Torso -->
     <link name="torso">
       <visual>
         <geometry>
           <box size="0.3 0.2 0.5"/>
         </geometry>
         <material name="gray">
           <color rgba="0.5 0.5 0.5 1.0"/>
         </material>
       </visual>
       <collision>
         <geometry>
           <box size="0.3 0.2 0.5"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="5.0"/>
         <inertia ixx="0.2" ixy="0" ixz="0" iyy="0.3" iyz="0" izz="0.4"/>
       </inertial>
     </link>

     <!-- Head -->
     <link name="head">
       <visual>
         <geometry>
           <sphere radius="0.1"/>
         </geometry>
         <material name="white">
           <color rgba="1 1 1 1"/>
         </material>
       </visual>
       <collision>
         <geometry>
           <sphere radius="0.1"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="1.0"/>
         <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
       </inertial>
     </link>

     <joint name="neck_joint" type="fixed">
       <parent link="torso"/>
       <child link="head"/>
       <origin xyz="0 0 0.35" rpy="0 0 0"/>
     </joint>

     <!-- Left Arm -->
     <link name="left_upper_arm">
       <visual>
         <geometry>
           <cylinder length="0.3" radius="0.05"/>
         </geometry>
         <material name="blue">
           <color rgba="0 0 1 1"/>
         </material>
       </visual>
       <collision>
         <geometry>
           <cylinder length="0.3" radius="0.05"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="0.5"/>
         <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
       </inertial>
     </link>

     <joint name="left_shoulder_joint" type="revolute">
       <parent link="torso"/>
       <child link="left_upper_arm"/>
       <origin xyz="0.15 0.1 0.1" rpy="0 0 0"/>
       <axis xyz="0 1 0"/>
       <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
     </joint>
   </robot>
   ```

2. **Set up the simulation environment**:
   Create a world file with humanoid-appropriate physics parameters:

   ```xml
   <?xml version="1.0" ?>
   <sdf version="1.7">
     <world name="humanoid_world">
       <include>
         <uri>model://ground_plane</uri>
       </include>

       <include>
         <uri>model://sun</uri>
       </include>

       <physics type="dart">
         <gravity>0 0 -9.8</gravity>
         <max_step_size>0.001</max_step_size>
         <real_time_factor>1.0</real_time_factor>
       </physics>

       <!-- Add a simple humanoid model -->
       <model name="simple_humanoid" placement_frame="simple_humanoid::torso">
         <pose>0 0 0.5 0 0 0</pose>
         <static>false</static>
       </model>
     </world>
   </sdf>
   ```

3. **Launch the simulation**:
   Start Gazebo with your humanoid world and observe the robot behavior.

4. **Interact with the simulation**:
   Use Gazebo's GUI tools to apply forces, change poses, and observe physics behavior.

### Expected Outcome
You should have a working Gazebo simulation with a simplified humanoid model that demonstrates basic physics simulation concepts. The model should respond appropriately to gravity and external forces.

### Connection to Future Modules
This exercise prepares you for more sophisticated humanoid simulation scenarios in Module 3, where Isaac Sim provides enhanced physics and rendering capabilities for humanoid robotics applications.

## Exercise 3: URDF to SDF Conversion Exercise

### Objective
Practice converting a URDF robot model to SDF format with proper physics properties for humanoid simulation, demonstrating the conversion process with humanoid-specific considerations.

### Prerequisites
- Understanding of URDF and SDF formats
- Knowledge of physics properties for simulation
- Access to conversion tools

### Exercise Description
In this exercise, you'll convert a more complex robot model from URDF to SDF format, paying special attention to humanoid-specific physics properties that ensure realistic simulation behavior.

### Steps

1. **Create a complex URDF model**:
   Design a more detailed robot model with multiple joints that could represent a humanoid limb:

   ```xml
   <?xml version="1.0"?>
   <robot name="humanoid_limb">
     <material name="black">
       <color rgba="0 0 0 1"/>
     </material>

     <material name="orange">
       <color rgba="1 0.4 0 1"/>
     </material>

     <material name="brown">
       <color rgba="0.8 0.4 0.2 1"/>
     </material>

     <!-- Hip joint and thigh -->
     <link name="hip">
       <visual>
         <origin xyz="0 0 0" rpy="0 0 0"/>
         <geometry>
           <cylinder length="0.1" radius="0.08"/>
         </geometry>
         <material name="orange"/>
       </visual>
       <collision>
         <origin xyz="0 0 0" rpy="0 0 0"/>
         <geometry>
           <cylinder length="0.1" radius="0.08"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="1.0"/>
         <origin xyz="0 0 0" rpy="0 0 0"/>
         <inertia ixx="0.001333" ixy="0" ixz="0" iyy="0.001333" iyz="0" izz="0.002"/>
       </inertial>
     </link>

     <link name="thigh">
       <visual>
         <origin xyz="0 0 -0.2" rpy="0 0 0"/>
         <geometry>
           <cylinder length="0.4" radius="0.06"/>
         </geometry>
         <material name="orange"/>
       </visual>
       <collision>
         <origin xyz="0 0 -0.2" rpy="0 0 0"/>
         <geometry>
           <cylinder length="0.4" radius="0.06"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="2.0"/>
         <origin xyz="0 0 -0.2" rpy="0 0 0"/>
         <inertia ixx="0.0267" ixy="0" ixz="0" iyy="0.0267" iyz="0" izz="0.0036"/>
       </inertial>
     </link>

     <joint name="hip_joint" type="revolute">
       <parent link="hip"/>
       <child link="thigh"/>
       <origin xyz="0 0 0" rpy="0 0 0"/>
       <axis xyz="0 1 0"/>
       <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
       <dynamics damping="1.0" friction="0.1"/>
     </joint>

     <!-- Knee joint and shin -->
     <link name="shin">
       <visual>
         <origin xyz="0 0 -0.2" rpy="0 0 0"/>
         <geometry>
           <cylinder length="0.4" radius="0.05"/>
         </geometry>
         <material name="orange"/>
       </visual>
       <collision>
         <origin xyz="0 0 -0.2" rpy="0 0 0"/>
         <geometry>
           <cylinder length="0.4" radius="0.05"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="1.5"/>
         <origin xyz="0 0 -0.2" rpy="0 0 0"/>
         <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.003125"/>
       </inertial>
     </link>

     <joint name="knee_joint" type="revolute">
       <parent link="thigh"/>
       <child link="shin"/>
       <origin xyz="0 0 -0.4" rpy="0 0 0"/>
       <axis xyz="0 1 0"/>
       <limit lower="0" upper="2.36" effort="100" velocity="1.0"/>  <!-- 0 to 135 degrees -->
       <dynamics damping="1.0" friction="0.1"/>
     </joint>

     <!-- Ankle joint and foot -->
     <link name="foot">
       <visual>
         <origin xyz="0 0 -0.05" rpy="0 0 0"/>
         <geometry>
           <box size="0.2 0.1 0.1"/>
         </geometry>
         <material name="black"/>
       </visual>
       <collision>
         <origin xyz="0 0 -0.05" rpy="0 0 0"/>
         <geometry>
           <box size="0.2 0.1 0.1"/>
         </geometry>
       </collision>
       <inertial>
         <mass value="0.5"/>
         <origin xyz="0 0 -0.05" rpy="0 0 0"/>
         <inertia ixx="0.000833" ixy="0" ixz="0" iyy="0.002083" iyz="0" izz="0.002083"/>
       </inertial>
     </link>

     <joint name="ankle_joint" type="revolute">
       <parent link="shin"/>
       <child link="foot"/>
       <origin xyz="0 0 -0.4" rpy="0 0 0"/>
       <axis xyz="0 1 0"/>
       <limit lower="-0.785" upper="0.785" effort="50" velocity="0.5"/>  <!-- -45 to 45 degrees -->
       <dynamics damping="2.0" friction="0.5"/>
     </joint>
   </robot>
   ```

2. **Convert the URDF to SDF using the command line tool**:
   ```bash
   gz sdf -p humanoid_limb.urdf > humanoid_limb.sdf
   ```

3. **Manually inspect and enhance the SDF**:
   Examine the converted SDF and add humanoid-specific physics enhancements:

   ```xml
   <?xml version="1.0" ?>
   <sdf version="1.7">
     <model name="humanoid_limb">
       <pose>0 0 0 0 0 0</pose>
       <static>false</static>
       <self_collide>false</self_collide>

       <!-- Hip joint and thigh -->
       <link name="hip">
         <pose>0 0 0 0 0 0</pose>
         <inertial>
           <mass>1.0</mass>
           <pose>0 0 0 0 0 0</pose>
           <inertia>
             <ixx>0.001333</ixx>
             <ixy>0</ixy>
             <ixz>0</ixz>
             <iyy>0.001333</iyy>
             <iyz>0</iyz>
             <izz>0.002</izz>
           </inertia>
         </inertial>

         <visual name="visual">
           <pose>0 0 0 0 0 0</pose>
           <geometry>
             <cylinder>
               <radius>0.08</radius>
               <length>0.1</length>
             </cylinder>
           </geometry>
           <material>
             <ambient>1 0.4 0 1</ambient>
             <diffuse>1 0.4 0 1</diffuse>
             <specular>1 0.4 0 1</specular>
           </material>
         </visual>

         <collision name="collision">
           <pose>0 0 0 0 0 0</pose>
           <geometry>
             <cylinder>
               <radius>0.08</radius>
               <length>0.1</length>
             </cylinder>
           </geometry>
         </collision>
       </link>

       <!-- Add other links and joints following the same pattern -->
       <!-- Thigh, shin, and foot links with appropriate joints -->

       <!-- Enhance with humanoid-specific physics properties -->
       <joint name="hip_joint" type="revolute">
         <parent>hip</parent>
         <child>thigh</child>
         <pose>0 0 0 0 0 0</pose>
         <axis>
           <xyz>0 1 0</xyz>
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

       <!-- Add other joints following the same pattern -->
     </model>
   </sdf>
   ```

4. **Test the converted model in simulation**:
   Load the SDF model in Gazebo and verify that it behaves correctly with proper physics properties.

### Expected Outcome
You should have successfully converted a URDF model to SDF format with enhanced physics properties appropriate for humanoid simulation. The converted model should exhibit realistic physics behavior in simulation.

### Connection to Future Modules
This exercise provides the foundation for creating more complex humanoid robot models in Module 3, where Isaac Sim offers advanced model conversion and physics capabilities for digital twin applications.

## Exercise 4: Physics Simulation Exercise

### Objective
Configure physics parameters for a humanoid robot model to achieve stable behaviors in simulation, demonstrating understanding of physics simulation essentials for humanoid behaviors.

### Prerequisites
- Understanding of physics simulation concepts
- Experience with Gazebo physics configuration
- Knowledge of humanoid balance and locomotion requirements

### Exercise Description
In this exercise, you'll configure physics parameters for a humanoid robot model to achieve stable standing and simple movement behaviors in simulation, demonstrating the importance of proper physics configuration for humanoid robotics.

### Steps

1. **Create a physics-optimized humanoid model**:
   Design a humanoid model with appropriate physics properties for stable simulation:

   ```xml
   <?xml version="1.0" ?>
   <sdf version="1.7">
     <model name="stable_humanoid">
       <pose>0 0 0 0 0 0</pose>
       <static>false</static>
       <self_collide>false</self_collide>

       <!-- Torso with appropriate mass distribution -->
       <link name="torso">
         <pose>0 0 0.8 0 0 0</pose>  <!-- Start above ground to allow for legs -->
         <inertial>
           <mass>10.0</mass>
           <pose>0 0 0 0 0 0</pose>
           <inertia>
             <ixx>0.5</ixx>
             <ixy>0</ixy>
             <ixz>0</ixz>
             <iyy>0.5</iyy>
             <iyz>0</iyz>
             <izz>0.5</izz>
           </inertia>
         </inertial>

         <visual name="torso_visual">
           <pose>0 0 0 0 0 0</pose>
           <geometry>
             <box>
               <size>0.3 0.3 0.6</size>
             </box>
           </geometry>
           <material>
             <ambient>0.5 0.5 0.5 1</ambient>
             <diffuse>0.5 0.5 0.5 1</diffuse>
             <specular>0.5 0.5 0.5 1</specular>
           </material>
         </visual>

         <collision name="torso_collision">
           <pose>0 0 0 0 0 0</pose>
           <geometry>
             <box>
               <size>0.3 0.3 0.6</size>
             </box>
           </geometry>
         </collision>
       </link>

       <!-- Head -->
       <link name="head">
         <pose>0 0 0.3 0 0 0</pose>
         <inertial>
           <mass>2.0</mass>
           <pose>0 0 0 0 0 0</pose>
           <inertia>
             <ixx>0.02</ixx>
             <ixy>0</ixy>
             <ixz>0</ixz>
             <iyy>0.02</iyy>
             <iyz>0</iyz>
             <izz>0.02</izz>
           </inertia>
         </inertial>

         <visual name="head_visual">
           <pose>0 0 0 0 0 0</pose>
           <geometry>
             <sphere>
               <radius>0.12</radius>
             </sphere>
           </geometry>
           <material>
             <ambient>1 1 1 1</ambient>
             <diffuse>1 1 1 1</diffuse>
             <specular>1 1 1 1</specular>
           </material>
         </visual>

         <collision name="head_collision">
           <pose>0 0 0 0 0 0</pose>
           <geometry>
             <sphere>
               <radius>0.12</radius>
             </sphere>
           </geometry>
         </collision>
       </link>

       <!-- Neck joint -->
       <joint name="neck_joint" type="fixed">
         <parent>torso</parent>
         <child>head</child>
         <pose>0 0 0.3 0 0 0</pose>
       </joint>

       <!-- Left leg for stability test -->
       <link name="left_thigh">
         <pose>0.1 0 -0.3 0 0 0</pose>
         <inertial>
           <mass>3.0</mass>
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

         <visual name="left_thigh_visual">
           <pose>0 0 -0.2 0 0 0</pose>
           <geometry>
             <cylinder>
               <radius>0.06</radius>
               <length>0.4</length>
             </cylinder>
           </geometry>
           <material>
             <ambient>0 0 1 1</ambient>
             <diffuse>0 0 1 1</diffuse>
             <specular>0 0 1 1</specular>
           </material>
         </visual>

         <collision name="left_thigh_collision">
           <pose>0 0 -0.2 0 0 0</pose>
           <geometry>
             <cylinder>
               <radius>0.06</radius>
               <length>0.4</length>
             </cylinder>
           </geometry>
         </collision>
       </link>

       <joint name="left_hip_joint" type="revolute">
         <parent>torso</parent>
         <child>left_thigh</child>
         <pose>0.1 0 -0.3 0 0 0</pose>
         <axis>
           <xyz>0 1 0</xyz>
           <limit>
             <lower>-1.57</lower>
             <upper>1.57</upper>
             <effort>200</effort>
             <velocity>2.0</velocity>
           </limit>
           <dynamics>
             <damping>2.0</damping>
             <friction>0.5</friction>
           </dynamics>
         </axis>
       </joint>

       <!-- Similar right leg -->
       <link name="right_thigh">
         <pose>-0.1 0 -0.3 0 0 0</pose>
         <inertial>
           <mass>3.0</mass>
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

         <visual name="right_thigh_visual">
           <pose>0 0 -0.2 0 0 0</pose>
           <geometry>
             <cylinder>
               <radius>0.06</radius>
               <length>0.4</length>
             </cylinder>
           </geometry>
           <material>
             <ambient>0 0 1 1</ambient>
             <diffuse>0 0 1 1</diffuse>
             <specular>0 0 1 1</specular>
           </material>
         </visual>

         <collision name="right_thigh_collision">
           <pose>0 0 -0.2 0 0 0</pose>
           <geometry>
             <cylinder>
               <radius>0.06</radius>
               <length>0.4</length>
             </cylinder>
           </geometry>
         </collision>
       </link>

       <joint name="right_hip_joint" type="revolute">
         <parent>torso</parent>
         <child>right_thigh</child>
         <pose>-0.1 0 -0.3 0 0 0</pose>
         <axis>
           <xyz>0 1 0</xyz>
           <limit>
             <lower>-1.57</lower>
             <upper>1.57</upper>
             <effort>200</effort>
             <velocity>2.0</velocity>
           </limit>
           <dynamics>
             <damping>2.0</damping>
             <friction>0.5</friction>
           </dynamics>
         </axis>
       </joint>
     </model>

     <!-- Physics configuration -->
     <physics type="dart">
       <gravity>0 0 -9.8</gravity>
       <max_step_size>0.001</max_step_size>
       <real_time_factor>1.0</real_time_factor>
       <max_contacts>10</max_contacts>

       <dart>
         <solver>
           <solver_type>NASOLVER</solver_type>
           <collision_detector>bullet</collision_detector>
         </solver>
       </dart>
     </physics>
   </sdf>
   ```

2. **Create a test world with appropriate physics parameters**:

   ```xml
   <?xml version="1.0" ?>
   <sdf version="1.7">
     <world name="humanoid_stability_test">
       <!-- Ground plane with appropriate friction -->
       <include>
         <uri>model://ground_plane</uri>
       </include>

       <!-- Sun for lighting -->
       <include>
         <uri>model://sun</uri>
       </include>

       <!-- Load our stable humanoid model -->
       <model name="stable_humanoid" placement_frame="stable_humanoid::torso">
         <pose>0 0 0 0 0 0</pose>
       </model>

       <!-- Physics configuration optimized for humanoid stability -->
       <physics type="dart">
         <gravity>0 0 -9.8</gravity>
         <max_step_size>0.001</max_step_size>
         <real_time_factor>1.0</real_time_factor>
         <max_contacts>20</max_contacts>

         <dart>
           <solver>
             <solver_type>NASOLVER</solver_type>
             <collision_detector>bullet</collision_detector>
           </solver>
         </dart>
       </physics>
     </world>
   </sdf>
   ```

3. **Test the stability**:
   - Launch the simulation and observe if the humanoid model maintains stable standing position
   - Apply small external forces to test balance recovery
   - Adjust physics parameters to optimize stability

4. **Experiment with different configurations**:
   - Try different friction coefficients for the feet
   - Adjust damping values for joints
   - Modify the center of mass by changing mass distribution

### Expected Outcome
The humanoid model should maintain a stable standing position without falling over, and should recover balance when subjected to small disturbances. This demonstrates proper physics configuration for humanoid behaviors.

### Connection to Future Modules
This exercise establishes the foundation for more advanced physics simulation scenarios in Module 3, where Isaac Sim provides enhanced physics capabilities for complex humanoid behaviors and simulation-to-real transfer.

## Exercise 5: Integration Exercise - Complete Digital Twin Scenario

### Objective
Integrate all concepts learned in this module by creating a complete Digital Twin scenario that connects ROS2 nodes with Gazebo simulation through ROS-Gazebo interfaces, demonstrating the full workflow from concept to implementation.

### Prerequisites
- All previous exercises completed
- Understanding of ROS2-Gazebo integration
- Knowledge of Digital Twin principles
- Experience with physics simulation configuration

### Exercise Description
In this capstone exercise, you'll create a complete Digital Twin scenario that integrates all concepts from this module: Digital Twin principles, Gazebo simulation, URDF/SDF conversion, and physics simulation. This exercise demonstrates the complete workflow for creating and using a Digital Twin for humanoid robotics.

### Steps

1. **Create a launch file** that starts both the simulation and control nodes:

   ```python
   # launch/humanoid_digital_twin.launch.py
   from launch import LaunchDescription
   from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
   from launch.launch_description_sources import PythonLaunchDescriptionSource
   from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
   from launch_ros.actions import Node
   from launch_ros.substitutions import FindPackageShare

   def generate_launch_description():
       # Launch arguments
       world_file_arg = DeclareLaunchArgument(
           'world',
           default_value='humanoid_world.sdf',
           description='Choose one of the world files from `/worlds`'
       )

       # Path to the world file
       world_file = PathJoinSubstitution([
           FindPackageShare('humanoid_simulation'),
           'worlds',
           LaunchConfiguration('world')
       ])

       # Start Gazebo
       gazebo = IncludeLaunchDescription(
           PythonLaunchDescriptionSource([
               PathJoinSubstitution([
                   FindPackageShare('gazebo_ros'),
                   'launch',
                   'gazebo.launch.py'
               ])
           ]),
           launch_arguments={
               'world': world_file,
               'verbose': 'false',
           }.items()
       )

       # Spawn the humanoid robot
       spawn_entity = Node(
           package='gazebo_ros',
           executable='spawn_entity.py',
           arguments=[
               '-entity', 'humanoid_robot',
               '-file', PathJoinSubstitution([
                   FindPackageShare('humanoid_simulation'),
                   'models',
                   'humanoid_robot.sdf'
               ]),
               '-x', '0',
               '-y', '0',
               '-z', '1.0',
           ],
           output='screen'
       )

       # Start the Digital Twin controller
       twin_controller = Node(
           package='humanoid_simulation',
           executable='twin_controller.py',
           name='twin_controller',
           output='screen'
       )

       # Start the sensor processor
       sensor_processor = Node(
           package='humanoid_simulation',
           executable='sensor_processor.py',
           name='sensor_processor',
           output='screen'
       )

       return LaunchDescription([
           world_file_arg,
           gazebo,
           spawn_entity,
           twin_controller,
           sensor_processor
       ])
   ```

2. **Create a controller node** that implements basic humanoid behaviors:

   ```python
   # scripts/twin_controller.py
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import Float64MultiArray
   from sensor_msgs.msg import JointState
   import math

   class HumanoidTwinController(Node):
       def __init__(self):
           super().__init__('humanoid_twin_controller')

           # Publishers for joint commands
           self.joint_cmd_publisher = self.create_publisher(
               Float64MultiArray,
               '/joint_commands',
               10
           )

           # Subscriber for joint states
           self.joint_state_subscriber = self.create_subscription(
               JointState,
               '/joint_states',
               self.joint_state_callback,
               10
           )

           # Timer for control loop
           self.control_timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

           self.time = 0.0
           self.get_logger().info('Humanoid Digital Twin Controller Started')

       def joint_state_callback(self, msg):
           # Process incoming joint states from simulation
           self.get_logger().debug(f'Received joint states: {msg.name}')

       def control_loop(self):
           # Create a simple oscillating motion for demonstration
           cmd_msg = Float64MultiArray()

           # Example: Create coordinated motion for humanoid joints
           # This would be replaced with actual control algorithms in real applications
           left_hip_pos = math.sin(self.time) * 0.2  # Small oscillation
           right_hip_pos = math.sin(self.time + math.pi) * 0.2  # Opposite phase

           cmd_msg.data = [left_hip_pos, right_hip_pos]

           self.joint_cmd_publisher.publish(cmd_msg)
           self.time += 0.05

           self.get_logger().info(f'Published joint commands: {[f"{x:.2f}" for x in cmd_msg.data]}')

   def main(args=None):
       rclpy.init(args=args)
       controller = HumanoidTwinController()

       try:
           rclpy.spin(controller)
       except KeyboardInterrupt:
           pass
       finally:
           controller.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

3. **Create a sensor processing node** that handles simulation data:

   ```python
   # scripts/sensor_processor.py
   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import JointState, Imu
   from std_msgs.msg import Float64

   class SensorProcessor(Node):
       def __init__(self):
           super().__init__('sensor_processor')

           # Subscribers for sensor data
           self.joint_state_sub = self.create_subscription(
               JointState,
               '/joint_states',
               self.joint_state_callback,
               10
           )

           self.imu_sub = self.create_subscription(
               Imu,
               '/imu_data',
               self.imu_callback,
               10
           )

           # Publisher for processed data
           self.balance_pub = self.create_publisher(
               Float64,
               '/balance_status',
               10
           )

           self.get_logger().info('Sensor Processor Started')

       def joint_state_callback(self, msg):
           # Process joint state data
           self.get_logger().debug(f'Processing {len(msg.name)} joints')

           # Example: Calculate simple balance metric
           balance_metric = self.calculate_balance_from_joints(msg)

           # Publish balance status
           balance_msg = Float64()
           balance_msg.data = balance_metric
           self.balance_pub.publish(balance_msg)

       def imu_callback(self, msg):
           # Process IMU data for balance control
           self.get_logger().debug(f'IMU orientation: {msg.orientation}')

       def calculate_balance_from_joints(self, joint_state):
           # Simple balance calculation based on joint positions
           # In a real system, this would be much more sophisticated
           return 1.0  # Placeholder for balance metric

   def main(args=None):
       rclpy.init(args=args)
       processor = SensorProcessor()

       try:
           rclpy.spin(processor)
       except KeyboardInterrupt:
           pass
       finally:
           processor.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

4. **Test the complete Digital Twin system**:
   - Launch the complete system using your launch file
   - Observe the interaction between the control nodes and the simulation
   - Monitor the communication between the virtual and control systems
   - Validate that the system behaves as expected

5. **Document the integration**:
   - Create a workflow diagram showing the complete Digital Twin system
   - Document the communication patterns between components
   - Identify potential improvements for real-world deployment

### Expected Outcome
You should have a complete working Digital Twin system where ROS2 control nodes interact with a Gazebo simulation of a humanoid robot. The system should demonstrate the bidirectional flow of information characteristic of Digital Twin systems.

### Connection to Future Modules
This exercise provides the foundation for more sophisticated Digital Twin implementations in Module 3, where Isaac Sim offers advanced simulation capabilities and more complex Digital Twin scenarios with enhanced physics and rendering.

## Summary

These exercises have provided hands-on experience with all the core concepts covered in Module 2. Through progressive complexity, you've worked with:

1. Basic Digital Twin concepts and their implementation
2. Gazebo simulation fundamentals for humanoid applications
3. URDF to SDF conversion with humanoid-specific considerations
4. Physics simulation essentials for realistic humanoid behaviors
5. Complete integration of all concepts in a Digital Twin scenario

Each exercise built upon the previous ones, demonstrating the interconnected nature of Digital Twin and simulation concepts in humanoid robotics. The skills developed in these exercises will be essential as you progress to more advanced simulation environments in Module 3 with Isaac Sim.

---

*This section provided integrated exercises that connect all concepts learned in Module 2. The exercises reinforced understanding through practical application and prepared students for simulation-to-real transfer in upcoming modules. In Module 3, we'll explore Isaac Sim and advanced simulation-to-real transfer techniques that build upon the Digital Twin foundations established in this module.*

*Cross-reference: The integration concepts covered in this section form the basis for advanced Digital Twin implementations in Isaac Sim (Module 3), where similar integration principles apply with enhanced capabilities and more sophisticated simulation environments.*