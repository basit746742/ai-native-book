---
title: "Kinematic Chains - Robot Structure and Movement"
description: "Understanding how links and joints form kinematic chains in robot models"
tags: ["urdf", "kinematics", "robotics", "modeling", "chains"]
sidebar_position: 4
difficulty: "intermediate"
learning_objectives:
  - "Understand kinematic chains and their importance"
  - "Identify different types of kinematic structures"
  - "Recognize how kinematic chains enable robot control"
time_estimated: "30 mins"
---

# Kinematic Chains - Robot Structure and Movement

## Overview

Kinematic chains are the fundamental structural elements that define how robot components are connected and how they move relative to each other. In robotics, a kinematic chain consists of rigid links connected by joints that transmit motion from one end of the chain to the other.

## Types of Kinematic Structures

### 1. Open Kinematic Chains
- **Structure**: Single path from base to end-effector
- **Characteristics**:
  - Simple forward and inverse kinematics
  - Common in robot arms
  - No loops or closed paths
- **Example**: Robot arm with 6 DOF from base to end-effector

### 2. Closed Kinematic Chains
- **Structure**: Contains one or more loops
- **Characteristics**:
  - More complex kinematics
  - Higher stiffness and load capacity
  - Common in parallel robots
- **Example**: Stewart platform, parallel manipulators

### 3. Tree Structures
- **Structure**: One base with multiple branches
- **Characteristics**:
  - Multiple end-effectors possible
  - More complex but versatile
  - Common in humanoid robots
- **Example**: Humanoid robot with arms, legs, and head

## Kinematic Chain Representation in URDF

In URDF, kinematic chains are represented through the parent-child relationships defined by joints:

```xml
<!-- Base of the chain -->
<link name="base_link"/>

<joint name="joint_1" type="revolute">
  <parent link="base_link"/>
  <child link="link_1"/>
</joint>

<link name="link_1"/>

<joint name="joint_2" type="revolute">
  <parent link="link_1"/>
  <child link="link_2"/>
</joint>

<link name="link_2"/>

<!-- End of the chain -->
```

## Forward Kinematics

Forward kinematics calculates the position and orientation of the end-effector given the joint angles:

- **Input**: Joint angles (θ₁, θ₂, ..., θₙ)
- **Output**: End-effector pose (position and orientation)
- **Use**: Predicting where the robot will be given its joint configuration

## Inverse Kinematics

Inverse kinematics calculates the required joint angles to achieve a desired end-effector pose:

- **Input**: Desired end-effector pose
- **Output**: Joint angles (θ₁, θ₂, ..., θₙ)
- **Use**: Planning robot motion to reach specific positions

## Common Kinematic Chain Configurations

### Robot Arm Chain
```
Base → Shoulder → Elbow → Wrist → End-Effector
```

### Humanoid Leg Chain
```
Torso → Hip → Thigh → Knee → Shank → Ankle → Foot
```

### Humanoid Arm Chain
```
Torso → Shoulder → Upper Arm → Elbow → Forearm → Wrist → Hand
```

## Kinematic Chain Analysis

### Degrees of Freedom (DOF)
- Each joint contributes DOF to the chain
- A 6-DOF chain can position and orient its end-effector in 3D space
- Redundant chains have more than 6 DOF for the same task

### Workspace
- **Reachable Workspace**: All points the end-effector can reach
- **Dexterous Workspace**: All points with full orientation capability
- Depends on joint limits and chain geometry

### Singularity
- Points where the robot loses one or more DOF
- Jacobian matrix becomes singular (non-invertible)
- Should be avoided in robot control

## Humanoid Robot Kinematic Structures

### Humanoid Torso (Tree Structure)
```
                    Head
                   /
Torso —— Waist —— Spine
                   \
                Pelvis
```

### Humanoid Limbs (Open Chains)
- **Arms**: Torso → Shoulder → Upper Arm → Elbow → Forearm → Wrist → Hand
- **Legs**: Pelvis → Hip → Thigh → Knee → Shank → Ankle → Foot

### Redundancy in Humanoid Robots
- Humanoid robots often have redundant DOF (more than 6)
- Allows for multiple configurations to reach the same position
- Enables posture optimization and obstacle avoidance

## URDF Implementation Considerations

### Root Link
- Every kinematic chain must have a root link (no parent)
- Typically named "base_link" or specific to robot type
- Serves as the reference frame for the entire robot

### Chain Consistency
- Joint parent-child relationships must form valid chains
- No orphaned links (except those connected to the environment)
- Proper naming conventions for easy identification

### Example: Humanoid Arm Chain in URDF
```xml
<!-- Root of the arm chain -->
<link name="torso"/>

<joint name="left_shoulder_yaw" type="revolute">
  <parent link="torso"/>
  <child link="left_upper_arm"/>
  <origin xyz="0.15 0.1 0.2" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="20" velocity="1"/>
</joint>

<link name="left_upper_arm">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.3"/>
    </geometry>
  </visual>
  <inertial>
    <mass value="2.0"/>
    <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.001"/>
  </inertial>
</link>

<joint name="left_elbow_pitch" type="revolute">
  <parent link="left_upper_arm"/>
  <child link="left_forearm"/>
  <origin xyz="0 0 -0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="0" upper="3.14" effort="15" velocity="1"/>
</joint>

<link name="left_forearm">
  <visual>
    <geometry>
      <cylinder radius="0.04" length="0.25"/>
    </geometry>
  </visual>
  <inertial>
    <mass value="1.5"/>
    <inertia ixx="0.03" ixy="0" ixz="0" iyy="0.03" iyz="0" izz="0.0008"/>
  </inertial>
</link>

<joint name="left_wrist_pitch" type="revolute">
  <parent link="left_forearm"/>
  <child link="left_hand"/>
  <origin xyz="0 0 -0.25" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
</joint>

<link name="left_hand">
  <visual>
    <geometry>
      <box size="0.1 0.08 0.05"/>
    </geometry>
  </visual>
  <inertial>
    <mass value="0.5"/>
    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
  </inertial>
</link>
```

## Kinematic Chain Validation

### Tools for Analysis
- **RViz**: Visualize robot model and check joint movements
- **URDF Checkers**: Validate syntax and structure
- **Forward Kinematics**: Test joint-to-pose calculations
- **Inverse Kinematics Solvers**: Verify reachability

### Common Issues
- **Invalid Joint Chains**: Broken parent-child relationships
- **Incorrect Joint Axes**: Wrong orientation of motion
- **Inconsistent Units**: Mixed measurement systems
- **Missing Inertial Properties**: Required for dynamics simulation

## Kinematic Chains in ROS 2

### TF2 (Transforms)
- Uses kinematic chain information to compute transforms
- Provides coordinate frames for all links
- Essential for navigation, manipulation, and perception

### Robot State Publisher
- Publishes joint states and link transforms
- Uses URDF kinematic information
- Enables visualization and coordinate transformation

### MoveIt!
- Planning framework that uses kinematic chains
- Performs inverse kinematics and motion planning
- Requires proper URDF chain definitions

Understanding kinematic chains is crucial for creating robot models that can be properly controlled, simulated, and used in ROS 2 applications. The chain structure determines how motion propagates through the robot and enables sophisticated control algorithms.