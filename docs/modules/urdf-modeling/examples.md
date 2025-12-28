---
title: "Practical Examples - URDF Humanoid Model"
description: "Practical examples demonstrating URDF modeling for humanoid robots"
tags: ["urdf", "examples", "humanoid", "robotics", "modeling"]
sidebar_position: 6
difficulty: "intermediate"
learning_objectives:
  - "Implement a complete humanoid robot model in URDF"
  - "Understand the integration of all URDF components"
  - "Apply best practices for humanoid modeling"
time_estimated: "40 mins"
---

# Practical Examples - URDF Humanoid Model

## Overview

This section provides practical examples of creating a humanoid robot model in URDF. We'll build a simplified humanoid model with a torso, arms, and legs, demonstrating how to integrate all URDF components into a complete robot model.

## Complete Humanoid Robot Example

Here's a complete example of a simple humanoid robot model:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Material definitions -->
  <material name="blue">
    <color rgba="0 0 1 1"/>
  </material>
  <material name="red">
    <color rgba="1 0 0 1"/>
  </material>
  <material name="white">
    <color rgba="1 1 1 1"/>
  </material>
  <material name="black">
    <color rgba="0 0 0 1"/>
  </material>

  <!-- Base link - torso -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.6"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.6"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.5" ixy="0.0" ixz="0.0" iyy="0.6" iyz="0.0" izz="0.3"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.004" ixy="0.0" ixz="0.0" iyy="0.004" iyz="0.0" izz="0.004"/>
    </inertial>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="5" velocity="1"/>
  </joint>

  <!-- Left Arm -->
  <!-- Left Shoulder -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_shoulder_pitch" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <!-- Left Elbow -->
  <link name="left_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <origin xyz="0 0 -0.125"/>
      <inertia ixx="0.007" ixy="0.0" ixz="0.0" iyy="0.007" iyz="0.0" izz="0.0008"/>
    </inertial>
  </link>

  <joint name="left_elbow_pitch" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="3.14" effort="8" velocity="1"/>
  </joint>

  <!-- Left Hand -->
  <link name="left_hand">
    <visual>
      <geometry>
        <box size="0.1 0.08 0.05"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.08 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.0003" ixy="0.0" ixz="0.0" iyy="0.0004" iyz="0.0" izz="0.0005"/>
    </inertial>
  </link>

  <joint name="left_wrist_pitch" type="revolute">
    <parent link="left_lower_arm"/>
    <child link="left_hand"/>
    <origin xyz="0 0 -0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.78" upper="0.78" effort="5" velocity="1"/>
  </joint>

  <!-- Right Arm (similar to left, mirrored) -->
  <link name="right_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="right_shoulder_pitch" type="revolute">
    <parent link="base_link"/>
    <child link="right_upper_arm"/>
    <origin xyz="-0.15 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <link name="right_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <origin xyz="0 0 -0.125"/>
      <inertia ixx="0.007" ixy="0.0" ixz="0.0" iyy="0.007" iyz="0.0" izz="0.0008"/>
    </inertial>
  </link>

  <joint name="right_elbow_pitch" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="3.14" effort="8" velocity="1"/>
  </joint>

  <link name="right_hand">
    <visual>
      <geometry>
        <box size="0.1 0.08 0.05"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.08 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.0003" ixy="0.0" ixz="0.0" iyy="0.0004" iyz="0.0" izz="0.0005"/>
    </inertial>
  </link>

  <joint name="right_wrist_pitch" type="revolute">
    <parent link="right_lower_arm"/>
    <child link="right_hand"/>
    <origin xyz="0 0 -0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.78" upper="0.78" effort="5" velocity="1"/>
  </joint>

  <!-- Left Leg -->
  <link name="left_thigh">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <origin xyz="0 0 -0.2"/>
      <inertia ixx="0.03" ixy="0.0" ixz="0.0" iyy="0.03" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <joint name="left_hip_pitch" type="revolute">
    <parent link="base_link"/>
    <child link="left_thigh"/>
    <origin xyz="0.08 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="15" velocity="1"/>
  </joint>

  <link name="left_shank">
    <visual>
      <geometry>
        <cylinder length="0.35" radius="0.05"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.35" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.175"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.0015"/>
    </inertial>
  </link>

  <joint name="left_knee_pitch" type="revolute">
    <parent link="left_thigh"/>
    <child link="left_shank"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.35" effort="12" velocity="1"/>
  </joint>

  <link name="left_foot">
    <visual>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <joint name="left_ankle_pitch" type="revolute">
    <parent link="left_shank"/>
    <child link="left_foot"/>
    <origin xyz="0 0 -0.35" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.78" upper="0.78" effort="8" velocity="1"/>
  </joint>

  <!-- Right Leg (similar to left, mirrored) -->
  <link name="right_thigh">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <origin xyz="0 0 -0.2"/>
      <inertia ixx="0.03" ixy="0.0" ixz="0.0" iyy="0.03" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <joint name="right_hip_pitch" type="revolute">
    <parent link="base_link"/>
    <child link="right_thigh"/>
    <origin xyz="-0.08 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="15" velocity="1"/>
  </joint>

  <link name="right_shank">
    <visual>
      <geometry>
        <cylinder length="0.35" radius="0.05"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.35" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.175"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.0015"/>
    </inertial>
  </link>

  <joint name="right_knee_pitch" type="revolute">
    <parent link="right_thigh"/>
    <child link="right_shank"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.35" effort="12" velocity="1"/>
  </joint>

  <link name="right_foot">
    <visual>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <joint name="right_ankle_pitch" type="revolute">
    <parent link="right_shank"/>
    <child link="right_foot"/>
    <origin xyz="0 0 -0.35" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.78" upper="0.78" effort="8" velocity="1"/>
  </joint>
</robot>
```

## Python Integration Example

Here's how to load and use this URDF model in a ROS 2 Python node:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class HumanoidRobotNode(Node):
    def __init__(self):
        super().__init__('humanoid_robot_node')

        # Initialize joint state publisher
        self.joint_state_publisher = self.create_publisher(
            JointState,
            'joint_states',
            10
        )

        # Initialize TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer for publishing states
        self.timer = self.create_timer(0.1, self.publish_joint_states)

        # Initialize joint positions
        self.joint_positions = {
            'neck_joint': 0.0,
            'left_shoulder_pitch': 0.0,
            'left_elbow_pitch': 0.0,
            'left_wrist_pitch': 0.0,
            'right_shoulder_pitch': 0.0,
            'right_elbow_pitch': 0.0,
            'right_wrist_pitch': 0.0,
            'left_hip_pitch': 0.0,
            'left_knee_pitch': 0.0,
            'left_ankle_pitch': 0.0,
            'right_hip_pitch': 0.0,
            'right_knee_pitch': 0.0,
            'right_ankle_pitch': 0.0
        }

        self.get_logger().info('Humanoid Robot Node Started')

    def publish_joint_states(self):
        # Create joint state message
        msg = JointState()
        msg.name = list(self.joint_positions.keys())
        msg.position = list(self.joint_positions.values())
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Publish joint states
        self.joint_state_publisher.publish(msg)

        # Update joint positions for demonstration (simple oscillating motion)
        time = self.get_clock().now().nanoseconds / 1e9
        self.joint_positions['left_shoulder_pitch'] = 0.5 * math.sin(time)
        self.joint_positions['right_shoulder_pitch'] = 0.5 * math.sin(time + math.pi)
        self.joint_positions['left_elbow_pitch'] = 0.3 * math.sin(time * 1.5)
        self.joint_positions['right_knee_pitch'] = 0.4 * math.sin(time * 0.8)

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidRobotNode()

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

## Launch File Example

Here's a launch file to run the robot model:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    urdf_model_path = LaunchConfiguration('urdf_model_path')

    declare_urdf_model_path = DeclareLaunchArgument(
        name='urdf_model_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('my_robot_description'),
            'urdf',
            'simple_humanoid.urdf'
        ]),
        description='Absolute path to robot urdf file'
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': open(urdf_model_path.perform(context={}), 'r').read()
        }]
    )

    # Joint State Publisher (GUI)
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[{
            'use_gui': True
        }]
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('my_robot_description'),
            'rviz',
            'humanoid.rviz'
        ])]
    )

    return LaunchDescription([
        declare_urdf_model_path,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])
```

## Key Components Explained

### 1. Root Link
- The `base_link` serves as the root of the kinematic tree
- All other links are connected to it through joints
- Often represents the main body/torso of the humanoid

### 2. Symmetric Limbs
- Arms and legs are modeled with similar structures
- Left and right sides are mirrored in position
- Joint limits may differ based on physical constraints

### 3. Inertial Properties
- Proper mass and inertia values are essential for simulation
- Values should reflect the actual physical properties of the robot
- Can be calculated using CAD software or estimated

### 4. Visual vs Collision
- Visual geometries define how the robot looks
- Collision geometries define how it interacts in simulation
- Collision geometries can be simplified for performance

## Best Practices Demonstrated

1. **Consistent Naming**: Joint names clearly indicate function and side (left/right)

2. **Proper Inertial Values**: Mass and inertia values are realistic for each link

3. **Realistic Joint Limits**: Limits reflect the physical capabilities of humanoid joints

4. **Appropriate Effort Values**: Effort limits correspond to realistic actuator capabilities

5. **Symmetric Design**: Left and right limbs are modeled consistently

## Testing the Model

To test this humanoid model:

1. **Validate the URDF**:
   ```bash
   check_urdf /path/to/your/humanoid.urdf
   ```

2. **Visualize in RViz**:
   ```bash
   ros2 launch your_package humanoid.launch.py
   ```

3. **Check transforms**:
   ```bash
   ros2 run tf2_tools view_frames
   ```

This complete example demonstrates how to create a functional humanoid robot model that can be used in ROS 2 for visualization, simulation, and control applications.