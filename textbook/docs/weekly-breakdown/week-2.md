---
title: Week 2 Simulation Basics and URDF Modeling
sidebar_position: 3
---

# Week 2: Simulation Basics and URDF Modeling

## Learning Objectives

By the end of this week, you will be able to:
- Create URDF models for humanoid robots with joints, links, and sensors
- Understand the relationship between URDF and SDF formats
- Build robot models with proper kinematic chains
- Configure physical properties for realistic simulation
- Validate robot models using URDF inspection tools

## Reading Assignments

- Review Module 2: Gazebo basics and simulation setup
- Study URDF and SDF format differences and conversions
- Examine physics simulation principles and dynamics
- Explore advanced modeling techniques for humanoid robots

## Hands-On Tasks

### Task 1: URDF Model Creation
1. Design a simplified humanoid robot model with 10-15 joints
2. Define links with appropriate inertial, visual, and collision properties
3. Create joints with proper limits, types, and axis definitions
4. Add sensor mounts (IMU, cameras, etc.) to the robot model

### Task 2: Model Validation
1. Load your URDF model in RViz for visualization
2. Check for kinematic chain errors and joint configurations
3. Validate mass properties and center of gravity
4. Test URDF parsing with ROS 2 tools

### Task 3: Simulation Integration
1. Convert your URDF to SDF format for Gazebo simulation
2. Create a Gazebo world file with appropriate environment
3. Spawn your robot model in the simulation environment
4. Test basic joint movements and kinematics in simulation

## Checkpoint

- Successfully visualize your URDF model in RViz
- Demonstrate the robot model spawning correctly in Gazebo
- Show proper joint configurations and kinematic behavior
- Validate all physical properties are correctly defined

## Resources

- [URDF Documentation](http://wiki.ros.org/urdf)
- [Gazebo Model Tutorial](http://gazebosim.org/tutorials?cat=build_robot)
- Module 2 exercises for additional practice
- Sample URDF models for reference