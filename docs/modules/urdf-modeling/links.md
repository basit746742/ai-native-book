---
title: "URDF Links - Physical Components"
description: "Understanding links as the rigid components of robot models in URDF"
tags: ["urdf", "links", "robotics", "modeling"]
sidebar_position: 2
difficulty: "intermediate"
learning_objectives:
  - "Understand the concept of links in URDF"
  - "Identify the different properties of links"
  - "Recognize how links represent physical components"
time_estimated: "20 mins"
---

# URDF Links - Physical Components

## Overview

Links in URDF represent the rigid parts of a robot. They are the building blocks that define the physical structure of the robot. Each link is a rigid body with specific properties that describe its appearance, collision characteristics, and physical properties.

## Structure of a Link

A basic link definition in URDF includes several optional but important elements:

```xml
<link name="link_name">
  <!-- Visual properties for display -->
  <visual>
    <!-- How the link appears visually -->
  </visual>

  <!-- Collision properties for physics simulation -->
  <collision>
    <!-- How the link interacts in collision detection -->
  </collision>

  <!-- Inertial properties for dynamics -->
  <inertial>
    <!-- Mass and inertia for physics simulation -->
  </inertial>
</link>
```

## Visual Properties

The `<visual>` element defines how a link appears when visualized:

```xml
<visual>
  <!-- Position and orientation relative to link origin -->
  <origin xyz="0 0 0" rpy="0 0 0"/>

  <!-- Geometry of the visual representation -->
  <geometry>
    <box size="1 1 1"/>
    <!-- OR -->
    <cylinder radius="0.1" length="0.5"/>
    <!-- OR -->
    <sphere radius="0.1"/>
    <!-- OR -->
    <mesh filename="package://my_robot/meshes/link.stl"/>
  </geometry>

  <!-- Material properties -->
  <material name="blue">
    <color rgba="0 0 1 1"/>
  </material>
</visual>
```

### Visual Origin
- Specifies the position and orientation of the visual geometry relative to the link's origin
- Uses xyz (position) and rpy (roll-pitch-yaw) values

### Visual Geometry
- **Box**: Defined by size="x y z"
- **Cylinder**: Defined by radius and length
- **Sphere**: Defined by radius
- **Mesh**: Defined by a file reference to 3D model

## Collision Properties

The `<collision>` element defines how the link interacts in collision detection:

```xml
<collision>
  <!-- Position and orientation relative to link origin -->
  <origin xyz="0 0 0" rpy="0 0 0"/>

  <!-- Geometry of the collision representation -->
  <geometry>
    <box size="1 1 1"/>
    <!-- OR other geometry types -->
  </geometry>
</collision>
```

### Key Points about Collision:
- Can be different from visual geometry (simpler shapes for performance)
- Used by physics engines for collision detection
- May have multiple collision elements for complex shapes

## Inertial Properties

The `<inertial>` element defines the physical properties needed for dynamics simulation:

```xml
<inertial>
  <!-- Mass of the link -->
  <mass value="1.0"/>

  <!-- Origin of the inertial reference frame -->
  <origin xyz="0 0 0" rpy="0 0 0"/>

  <!-- Inertia matrix -->
  <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
</inertial>
```

### Inertial Properties Explained:
- **Mass**: The mass of the link in kilograms
- **Origin**: Position of the center of mass relative to the link origin
- **Inertia**: The 3x3 inertia matrix (simplified to 6 values in URDF)

## Practical Examples

### Example 1: Simple Wheel Link
```xml
<link name="wheel_link">
  <visual>
    <geometry>
      <cylinder radius="0.1" length="0.05"/>
    </geometry>
    <material name="black">
      <color rgba="0 0 0 1"/>
    </material>
  </visual>

  <collision>
    <geometry>
      <cylinder radius="0.1" length="0.05"/>
    </geometry>
  </collision>

  <inertial>
    <mass value="0.5"/>
    <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.002"/>
  </inertial>
</link>
```

### Example 2: Complex Link with Mesh
```xml
<link name="sensor_link">
  <visual>
    <geometry>
      <mesh filename="package://my_robot/meshes/sensor.dae"/>
    </geometry>
  </visual>

  <collision>
    <!-- Simplified collision shape -->
    <geometry>
      <cylinder radius="0.03" length="0.04"/>
    </geometry>
  </collision>

  <inertial>
    <mass value="0.1"/>
    <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
  </inertial>
</link>
```

## Best Practices for Links

1. **Meaningful Names**: Use descriptive names for links (e.g., "left_wheel", "camera_mount")

2. **Appropriate Collision Models**: Use simpler shapes for collision to improve performance

3. **Accurate Inertial Properties**: Ensure mass and inertia values match the real physical properties

4. **Consistent Units**: Use consistent units (meters for distance, kilograms for mass)

5. **Origin Consistency**: Keep link origins consistent (often at geometric center or connection point)

## Common Link Types in Humanoid Robots

### Limb Links
- Upper arm, lower arm, hand
- Torso segments
- Thigh, shank, foot

### Specialized Links
- Camera mounts
- Sensor housings
- End-effector tools

Understanding links is fundamental to creating accurate robot models that can be properly visualized, simulated, and controlled in ROS 2 systems.