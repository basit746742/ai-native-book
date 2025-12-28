---
title: "URDF Joints - Connections Between Components"
description: "Understanding joints as the connections between links in robot models"
tags: ["urdf", "joints", "robotics", "modeling"]
sidebar_position: 3
difficulty: "intermediate"
learning_objectives:
  - "Understand the concept of joints in URDF"
  - "Identify different joint types and their characteristics"
  - "Recognize how joints define robot kinematics"
time_estimated: "25 mins"
---

# URDF Joints - Connections Between Components

## Overview

Joints in URDF define how links are connected to each other, specifying the degrees of freedom and constraints between rigid bodies. They are crucial for defining the robot's kinematic structure and enabling motion planning and control.

## Joint Structure

A basic joint definition in URDF includes:

```xml
<joint name="joint_name" type="joint_type">
  <!-- Specifies the parent link -->
  <parent link="parent_link_name"/>

  <!-- Specifies the child link -->
  <child link="child_link_name"/>

  <!-- Position and orientation of the joint -->
  <origin xyz="x y z" rpy="roll pitch yaw"/>
</joint>
```

## Joint Types

### 1. Fixed Joint
- **Type**: `fixed`
- **Degrees of Freedom**: 0
- **Description**: Rigid connection with no movement
- **Use Case**: Mounting sensors, attaching non-moving components

```xml
<joint name="sensor_mount" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.1 0 0.2" rpy="0 0 0"/>
</joint>
```

### 2. Continuous Joint
- **Type**: `continuous`
- **Degrees of Freedom**: 1 (rotation)
- **Description**: Unlimited rotation around joint axis
- **Use Case**: Continuous rotating wheels, rotating sensors

```xml
<joint name="continuous_joint" type="continuous">
  <parent link="base_link"/>
  <child link="rotating_part"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
</joint>
```

### 3. Revolute Joint
- **Type**: `revolute`
- **Degrees of Freedom**: 1 (rotation)
- **Description**: Limited rotation around joint axis
- **Use Case**: Elbow, knee, shoulder joints

```xml
<joint name="elbow_joint" type="revolute">
  <parent link="upper_arm"/>
  <child link="lower_arm"/>
  <origin xyz="0 0 -0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
</joint>
```

### 4. Prismatic Joint
- **Type**: `prismatic`
- **Degrees of Freedom**: 1 (linear motion)
- **Description**: Linear sliding motion along joint axis
- **Use Case**: Linear actuators, telescoping mechanisms

```xml
<joint name="linear_joint" type="prismatic">
  <parent link="base_link"/>
  <child link="slider_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="0" upper="0.5" effort="20" velocity="0.1"/>
</joint>
```

### 5. Floating Joint
- **Type**: `floating`
- **Degrees of Freedom**: 6
- **Description**: Free movement in all directions
- **Use Case**: Rarely used, mainly for special simulation cases

### 6. Planar Joint
- **Type**: `planar`
- **Degrees of Freedom**: 3
- **Description**: Movement in a plane with rotation about normal
- **Use Case**: Planar mechanisms

## Joint Properties

### Origin
- Specifies the position and orientation of the joint relative to the parent link
- Uses xyz (position) and rpy (roll-pitch-yaw) values
- Defines the transformation from parent to child when joint value is zero

### Axis
- Defines the axis of motion for 1-DOF joints
- Specified as a unit vector (x, y, z)
- Direction is in the child link's frame

### Limits (for revolute and prismatic joints)
```xml
<limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
```
- **lower/upper**: Joint limits in radians (revolute) or meters (prismatic)
- **effort**: Maximum effort/torque the joint can exert (in N or Nm)
- **velocity**: Maximum velocity (in rad/s or m/s)

### Dynamics
```xml
<dynamics damping="0.1" friction="0.0"/>
```
- **damping**: Damping coefficient
- **friction**: Joint friction coefficient

### Safety Controller
```xml
<safety_controller soft_lower_limit="-1.5" soft_upper_limit="1.5"
                  k_position="10" k_velocity="10"/>
```
- Defines safety limits and control parameters

## Common Joint Applications in Humanoid Robots

### Upper Body
- **Shoulder**: Usually 3 DOF (revolute joints)
- **Elbow**: 1-2 DOF (revolute joints)
- **Wrist**: Multiple DOF (revolute joints)
- **Hand**: Many DOF for grasping

### Lower Body
- **Hip**: 3 DOF (revolute joints)
- **Knee**: 1 DOF (revolute joint)
- **Ankle**: 2-3 DOF (revolute joints)
- **Foot**: Usually fixed connection

## Joint Axes Convention

For humanoid robots, it's important to maintain consistent joint axis conventions:
- **Shoulder abduction/adduction**: Typically around Y-axis
- **Shoulder flexion/extension**: Typically around Z-axis
- **Elbow flexion**: Typically around Y-axis
- **Hip abduction/adduction**: Typically around Y-axis
- **Hip flexion/extension**: Typically around X-axis
- **Knee flexion**: Typically around Y-axis

## Example: Humanoid Arm Joint Structure

```xml
<!-- Shoulder joint group -->
<joint name="shoulder_yaw" type="revolute">
  <parent link="torso"/>
  <child link="upper_arm"/>
  <origin xyz="0.15 0 0.2" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="20" velocity="1"/>
</joint>

<joint name="shoulder_pitch" type="revolute">
  <parent link="upper_arm"/>
  <child link="forearm"/>
  <origin xyz="0 0 -0.3" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-2.35" upper="2.35" effort="20" velocity="1"/>
</joint>

<joint name="elbow_pitch" type="revolute">
  <parent link="forearm"/>
  <child link="hand"/>
  <origin xyz="0 0 -0.25" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="0" upper="3.14" effort="15" velocity="1"/>
</joint>
```

## Best Practices for Joints

1. **Consistent Naming**: Use clear, descriptive names that indicate function (e.g., "left_elbow_pitch")

2. **Proper Joint Limits**: Set realistic limits based on physical constraints

3. **Correct Joint Axes**: Ensure axes are oriented correctly for intended motion

4. **Realistic Effort Limits**: Set effort limits that reflect actual actuator capabilities

5. **Kinematic Chain**: Ensure joints form a proper kinematic chain without loops (unless using specialized packages)

6. **Joint Zero Position**: Consider what the "zero" position means for each joint in your control system

Understanding joints is crucial for creating robot models that can be properly controlled and for enabling accurate kinematic calculations in ROS 2 systems.