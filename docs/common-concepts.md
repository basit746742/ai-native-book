---
title: "Common Concepts Across Modules"
description: "Key concepts that span multiple modules in the ROS 2 Robotics Nervous System"
sidebar_position: 6
---

# Common Concepts Across Modules

This page highlights key concepts that appear across multiple modules in the ROS 2 Robotics Nervous System educational content. Understanding these common themes will help you see the connections between different aspects of ROS 2 development.

## Core Communication Patterns

### Topics (Publish/Subscribe)
**Module 1**: [ROS 2 Basics - Topics](./modules/ros2-basics/topics)
**Module 2**: Applied in [Perception systems](./modules/ai-robot-control/perception)
**Module 3**: Used with [URDF models](./modules/urdf-modeling/ros2-integration) for sensor data

The publish/subscribe pattern is fundamental to ROS 2. Publishers send data to named topics, and subscribers receive data from those topics. This pattern is ideal for streaming data like sensor readings, status updates, and continuous commands.

### Services (Request/Response)
**Module 1**: [ROS 2 Basics - Services](./modules/ros2-basics/services)
**Module 2**: Used in [decision-making systems](./modules/ai-robot-control/decision-making)
**Module 3**: Applied in [model loading services](./modules/urdf-modeling/ros2-integration)

The request/response pattern is used for operations that have a clear input and output. A client sends a request to a service server and waits for a response. This is ideal for configuration changes, specific operations, and tasks with immediate results.

### Actions (Long-Running Tasks)
**Module 2**: [Action Execution](./modules/ai-robot-control/action)
**Module 3**: Used with [manipulation tasks](./modules/urdf-modeling/ros2-integration)

Actions combine features of both topics and services, providing a pattern for long-running tasks that may provide feedback and can be canceled.

## Coordinate Systems and Transforms

### TF2 (Transform Library)
**Module 1**: Introduced in [Nodes concept](./modules/ros2-basics/nodes)
**Module 2**: Used in [perception systems](./modules/ai-robot-control/perception)
**Module 3**: Essential for [URDF integration](./modules/urdf-modeling/ros2-integration)

TF2 provides coordinate transforms between different frames in ROS 2. Each link in a URDF model creates a coordinate frame, and TF2 enables transformations between them. This is crucial for navigation, perception, and manipulation.

## Quality of Service (QoS)

### Reliability and Durability
**Module 1**: Covered in [Topics](./modules/ros2-basics/topics) and [Services](./modules/ros2-basics/services)
**Module 2**: Applied in [AI integration](./modules/ai-robot-control/rclpy-integration)
**Module 3**: Important for [simulation integration](./modules/urdf-modeling/ros2-integration)

QoS settings control how messages are delivered. Reliability determines if messages must be delivered (reliable) or can be dropped (best effort). Durability determines if messages persist for late-joining subscribers.

## AI Integration Patterns

### Perception → Decision → Action Flow
**Module 1**: Introduced in [basic concepts](./modules/ros2-basics/introduction)
**Module 2**: Detailed in [AI-to-Robot Control](./modules/ai-robot-control/perception)
**Module 3**: Applied with [robot models](./modules/urdf-modeling/ros2-integration)

This fundamental pattern describes how intelligent robotic systems operate:
1. **Perception**: Sensors gather information about the environment
2. **Decision**: AI algorithms process information and determine actions
3. **Action**: Commands are executed to affect the environment

## Robot Modeling Components

### Links and Joints
**Module 1**: Basic concepts in [Nodes](./modules/ros2-basics/nodes)
**Module 2**: Applied in [AI systems](./modules/ai-robot-control/perception)
**Module 3**: Detailed in [URDF modeling](./modules/urdf-modeling/links) and [joints](./modules/urdf-modeling/joints)

Robot models consist of links (rigid bodies) connected by joints (constraints). Understanding this structure is essential for both simulation and control.

## Node-Based Architecture

### Modular Design
**Module 1**: Explained in [Nodes concept](./modules/ros2-basics/nodes)
**Module 2**: Applied in [AI integration](./modules/ai-robot-control/rclpy-integration)
**Module 3**: Used in [model integration](./modules/urdf-modeling/ros2-integration)

ROS 2 follows a modular design where each node handles a specific function. This promotes reusability, testability, and maintainability of robotic systems.

## Common Message Types

### Sensor Messages
- **sensor_msgs**: Used across all modules for sensor data
- **geometry_msgs**: For positions, orientations, and velocities
- **std_msgs**: Basic data types used throughout ROS 2

## Best Practices Applied Across Modules

### 1. Descriptive Naming
Use clear, descriptive names for nodes, topics, and services across all modules.

### 2. Consistent Coordinate Frames
Follow REP-103 conventions for coordinate frames (X-forward, Y-left, Z-up).

### 3. Appropriate QoS Settings
Choose QoS settings based on the requirements of your specific application.

### 4. Error Handling
Implement proper error handling in all modules to ensure robust operation.

### 5. Documentation
Document your code and system architecture to enable maintenance and extension.

## Integration Points

### Where Modules Connect
- **Module 1 & 2**: AI nodes subscribe to topics from basic ROS 2 systems
- **Module 1 & 3**: URDF models are used with basic ROS 2 communication
- **Module 2 & 3**: AI systems control robots modeled with URDF
- **All Modules**: Common use of TF2, message types, and node architecture

Understanding these common concepts will help you see the bigger picture of how different aspects of ROS 2 development work together to create sophisticated robotic systems.