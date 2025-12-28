---
title: "Introduction to URDF - Unified Robot Description Format"
description: "Understanding the fundamentals of URDF for robot modeling in ROS 2"
tags: ["urdf", "robotics", "modeling", "ros2"]
sidebar_position: 1
difficulty: "intermediate"
learning_objectives:
  - "Understand what URDF is and its purpose"
  - "Recognize the basic structure of URDF files"
  - "Identify the key elements of robot description"
time_estimated: "25 mins"
---

# Introduction to URDF - Unified Robot Description Format

## Prerequisites

Before diving into URDF modeling, make sure you understand:
- [ROS 2 basics](../ros2-basics/introduction), particularly [nodes](../ros2-basics/nodes) and [topics](../ros2-basics/topics)
- [AI-to-robot integration concepts](../ai-robot-control/perception), especially how [perception systems](../ai-robot-control/perception) use coordinate frames

## What is URDF?

URDF (Unified Robot Description Format) is an XML-based format used in ROS to describe robots. It contains information about the robot's physical structure, including links (rigid parts), joints (connections between links), inertial properties, visual and collision models, and other properties.

## Purpose of URDF

URDF serves several important purposes in robotics:
- **Visualization**: Allows tools like RViz to display the robot in 3D
- **Simulation**: Provides physical properties for physics engines
- **Kinematics**: Enables forward and inverse kinematics calculations
- **Planning**: Supports motion planning algorithms
- **Control**: Helps with robot control by providing structural information

## Basic URDF Structure

A basic URDF file follows this structure:

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Links define rigid bodies -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Joints define connections between links -->
  <joint name="base_to_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0 0.25 0" rpy="0 0 0"/>
  </joint>

  <link name="wheel_link">
    <!-- Similar visual, collision, and inertial definitions -->
  </link>
</robot>
```

## Key Components of URDF

### Links
- Represent rigid parts of the robot
- Each link can have:
  - Visual properties (how it looks)
  - Collision properties (for physics simulation)
  - Inertial properties (for dynamics)

### Joints
- Define how links are connected
- Types include:
  - **fixed**: No movement between links
  - **continuous**: Rotational joint with unlimited range
  - **revolute**: Rotational joint with limited range
  - **prismatic**: Linear sliding joint
  - **floating**: 6-DOF joint (rarely used)
  - **planar**: Movement in a plane

## URDF in the ROS 2 Ecosystem

URDF integrates with ROS 2 through:
- **Robot State Publisher**: Publishes joint states and transforms
- **TF2**: Provides coordinate transforms between links
- **RViz**: Visualizes the robot model
- **Gazebo/other simulators**: Uses URDF for physics simulation

## Limitations of URDF

While URDF is widely used, it has some limitations:
- Cannot represent flexible or soft robots
- Limited support for multi-body systems
- No built-in support for transmission elements
- Static descriptions (no dynamic reconfiguration)

## Xacro: URDF's More Powerful Cousin

Xacro is an XML macro language that extends URDF with:
- Macros and reusable components
- Mathematical expressions
- Conditional statements
- Include statements for modularity

## Example: Simple Two-Link Robot

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <link name="link1">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.02"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>
</robot>
```

This simple example shows the basic structure of a robot with two links connected by a revolute joint, demonstrating the fundamental concepts of URDF.