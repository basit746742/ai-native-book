---
title: "URDF Integration with ROS 2"
description: "Understanding how URDF models integrate with ROS 2 systems and tools"
tags: ["urdf", "ros2", "integration", "robotics", "modeling"]
sidebar_position: 5
difficulty: "intermediate"
learning_objectives:
  - "Understand how URDF models integrate with ROS 2"
  - "Identify key ROS 2 packages for URDF support"
  - "Recognize the role of URDF in ROS 2 robot systems"
time_estimated: "25 mins"
---

# URDF Integration with ROS 2

## Overview

URDF (Unified Robot Description Format) integrates seamlessly with ROS 2 to provide robot description, visualization, simulation, and control capabilities. This integration enables the development of sophisticated robotic systems with accurate models for perception, planning, and control.

## Key ROS 2 Packages for URDF

### robot_state_publisher
- **Purpose**: Publishes joint states and transforms for robot links
- **Function**: Reads URDF and joint positions to publish TF transforms
- **Topic**: `/joint_states` (sensor_msgs/JointState)
- **TF**: Publishes transforms for all links in the URDF

```python
# Example usage in launch file
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=['path/to/robot_description.yaml']
        )
    ])
```

### joint_state_publisher
- **Purpose**: Publishes joint state messages
- **Function**: Either publishes static joint states or GUI for manual control
- **Topic**: `/joint_states` (sensor_msgs/JointState)

### tf2 (Transform Library)
- **Purpose**: Provides coordinate transforms between robot links
- **Function**: Uses URDF kinematic information to compute transforms
- **Key Feature**: Essential for navigation, perception, and manipulation

## URDF in ROS 2 Workflows

### Robot Description Parameter
URDF is typically passed to ROS 2 nodes as a parameter named `robot_description`:

```xml
<launch>
  <node pkg="robot_state_publisher" exec="robot_state_publisher">
    <param name="robot_description" value="$(find-pkg-share my_robot_description)/urdf/my_robot.urdf"/>
  </node>
</launch>
```

### Loading URDF from Files
```python
import rclpy
from rclpy.node import Node
import xacro

class RobotStatePublisherNode(Node):
    def __init__(self):
        super().__init__('robot_state_publisher')

        # Load URDF from file
        urdf_file = self.declare_parameter(
            'robot_description_file',
            'default.urdf'
        ).get_parameter_value().string_value

        # Parse URDF/Xacro
        robot_description = self.load_urdf(urdf_file)

        # Set as parameter for other nodes
        self.set_parameters([Parameter('robot_description', value=robot_description)])

    def load_urdf(self, file_path):
        # Load and process URDF file
        if file_path.endswith('.xacro'):
            doc = xacro.parse(open(file_path))
            xacro.process_doc(doc)
            return doc.toprettyxml(indent='  ')
        else:
            with open(file_path, 'r') as file:
                return file.read()
```

## Visualization Integration

### RViz
- **Purpose**: Visualizes robot models in 3D
- **Integration**: Uses robot_description parameter to load URDF
- **Displays**: RobotModel, TF, sensors, and other data

### RobotModel Display
- Shows the robot in 3D based on current joint states
- Colors can be set via materials in URDF
- Updates in real-time as joints move

## Simulation Integration

### Gazebo (Ignition)
- **Purpose**: Physics simulation environment
- **Integration**: Uses URDF for collision and visual properties
- **Features**: Dynamics simulation, sensor simulation, contact physics

### Example: Launching with Gazebo
```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Load URDF
    urdf_path = os.path.join(
        get_package_share_directory('my_robot_description'),
        'urdf',
        'my_robot.urdf'
    )

    return LaunchDescription([
        # Publish robot state
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_path).read()}]
        ),

        # Launch Gazebo with robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ])
        )
    ])
```

## Control Integration

### Joint State Interface
- **Purpose**: Provides access to current joint positions
- **Topic**: `/joint_states`
- **Message Type**: sensor_msgs/JointState
- **Integration**: robot_state_publisher subscribes to joint positions and publishes transforms

### Controller Integration
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Subscribe to joint states
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        # Initialize TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

    def joint_state_callback(self, msg):
        # Process joint states and publish transforms
        # This enables real-time visualization and navigation
        pass
```

## Perception Integration

### Coordinate Frames
- Each link in URDF creates a coordinate frame
- TF2 provides transforms between all frames
- Essential for sensor fusion and localization

### Example: Camera Frame Integration
```xml
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.05 0.05 0.05"/>
    </geometry>
  </visual>
</link>

<joint name="camera_mount" type="fixed">
  <parent link="head_link"/>
  <child link="camera_link"/>
  <origin xyz="0.1 0 0.05" rpy="0 0 0"/>
</joint>
```

## MoveIt! Integration

### Motion Planning
- **Purpose**: Motion planning and trajectory execution
- **Integration**: Uses URDF for collision checking and kinematics
- **Features**: Inverse kinematics, path planning, collision avoidance

### Planning Scene
- Incorporates URDF model into planning environment
- Uses visual and collision geometries for collision detection
- Enables complex manipulation planning

## Best Practices for URDF Integration

### 1. Parameter Management
- Store URDF in robot_description parameter
- Use xacro for complex models with macros and calculations
- Organize URDF files in package structure

### 2. Coordinate Frame Conventions
- Use consistent naming conventions
- Follow REP-103 for coordinate frame conventions
- Document frame relationships clearly

### 3. Performance Optimization
- Simplify collision geometries for simulation
- Use appropriate mesh resolutions
- Limit number of links for performance

### 4. Validation
- Test URDF with check_urdf tool
- Verify transforms in RViz
- Test with robot_state_publisher

## Example: Complete Integration Launch File

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    urdf_model_path = LaunchConfiguration('urdf_model_path')

    # Declare launch arguments
    declare_urdf_model_path = DeclareLaunchArgument(
        name='urdf_model_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('my_robot_description'),
            'urdf',
            'my_robot.urdf'
        ]),
        description='Absolute path to robot urdf file'
    )

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': open(urdf_model_path.perform(context={}), 'r').read()
        }]
    )

    # Joint State Publisher node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{
            'use_gui': True
        }]
    )

    # Return the launch description
    return LaunchDescription([
        declare_urdf_model_path,
        robot_state_publisher_node,
        joint_state_publisher_node
    ])
```

## Common Integration Issues

### 1. Missing Transforms
- **Symptom**: Robot not displaying in RViz
- **Solution**: Check robot_state_publisher is running with correct URDF

### 2. Invalid Joint Names
- **Symptom**: Joint states not matching URDF
- **Solution**: Verify joint names match between controllers and URDF

### 3. Frame Mismatch
- **Symptom**: Robot parts in wrong positions
- **Solution**: Check joint origins and parent-child relationships

The integration of URDF with ROS 2 enables the creation of sophisticated robotic systems where accurate models support perception, planning, control, and simulation. Proper integration is essential for reliable robot operation.