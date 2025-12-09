---
title: Robotics Terms
sidebar_position: 2
---

# Robotics Terms

This section provides concise definitions for key robotics concepts, organized alphabetically.

## A

**Actuators** - Components that convert energy into physical motion. In robotics, actuators are the "muscles" that move robot joints and enable physical interaction with the environment. Common types include servo motors, stepper motors, and hydraulic/pneumatic actuators.

**Actions** - A communication pattern in ROS2 for long-running tasks with feedback. Actions allow clients to send goals to servers, receive continuous feedback during execution, and get results when the task completes. Used for operations like navigation or complex manipulations.

## C

**Controllers** - Software or hardware components that generate commands to drive robot actuators based on desired behavior and sensor feedback. Controllers ensure robot joints follow desired trajectories and maintain stability during operation.

## G

**Gazebo** - A physics-based 3D simulation environment for robotics. Gazebo provides realistic simulation of robots, sensors, and environments with accurate physics modeling. It's commonly used for testing robot algorithms before deployment on real hardware.

## K

**Kinematics** - The study of motion without considering forces. In robotics, forward kinematics calculates end-effector position from joint angles, while inverse kinematics determines joint angles needed to achieve a desired end-effector position.

## N

**Nodes** - Basic computational elements in ROS2 that perform specific tasks. Nodes communicate with each other through topics, services, and actions. Each node typically handles a specific function like sensor processing, control, or visualization.

## P

**Perception** - The ability of a robot to understand its environment through sensor data. Perception systems process inputs from cameras, lidars, and other sensors to detect objects, recognize scenes, and build environmental models.

**Planning** - The process of determining a sequence of actions to achieve a goal. Motion planning generates paths for robots to navigate around obstacles, while task planning determines high-level sequences of operations.

## S

**Sensors** - Devices that detect and measure physical properties from the environment. Common robot sensors include cameras, lidars, IMUs, force/torque sensors, and encoders that provide feedback for robot control and perception.

**Services** - A communication pattern in ROS2 for request-response interactions. Services enable synchronous communication where a client sends a request and waits for a response from a service server. Used for operations that have a clear request-result relationship.

**SLAM (Simultaneous Localization and Mapping)** - A technique that allows robots to build a map of unknown environments while simultaneously tracking their location within that map. Essential for autonomous navigation in unstructured environments.

## T

**TF2 (Transform Library)** - The ROS2 system for managing coordinate frame relationships over time. TF2 enables conversion of data between different coordinate systems, essential for integrating sensor data and controlling robot movements.

**Topics** - Communication channels in ROS2 that enable asynchronous message passing between nodes. Publishers send messages to topics, and subscribers receive messages from topics in a many-to-many relationship.

## U

**URDF (Unified Robot Description Format)** - An XML-based format for describing robot models in ROS. URDF defines robot structure, joints, links, inertial properties, and visual appearance for simulation and control.

## M

**Manipulation** - The ability of a robot to interact with objects in its environment using end-effectors (grippers, tools). Manipulation involves grasping, moving, and placing objects with precision and control.

## S (continued)

**SDF (Simulation Description Format)** - An XML-based format used by Gazebo to describe simulation environments, including robots, objects, and world properties. SDF defines how elements behave in simulation.