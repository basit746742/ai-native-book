---
title: "ROS 2 Glossary"
description: "Comprehensive glossary of ROS 2 terms and concepts"
sidebar_position: 4
---

# ROS 2 Glossary

This glossary provides definitions for key terms and concepts used throughout the ROS 2 Robotics Nervous System educational modules.

## A

### Action
A communication pattern in ROS 2 designed for long-running tasks that may provide feedback during execution and can be canceled. Consists of goal, feedback, and result components.

### AI Agent
A system that perceives its environment and takes actions to achieve goals. In ROS 2, AI agents often subscribe to sensor topics and publish commands.

## B

### Behavior Tree
A hierarchical structure used to organize robot behaviors. Provides a modular and reusable way to design complex robot behaviors.

## C

### Client Library
Software libraries that enable ROS 2 functionality in different programming languages (e.g., rclpy for Python, rclcpp for C++).

### Collision
In URDF, the geometric representation of a link used for physics simulation and collision detection.

### Coordinate Frame
A reference frame used to represent position and orientation in 3D space. ROS 2 uses TF2 for coordinate frame transformations.

## D

### DDS (Data Distribution Service)
Middleware that provides the underlying communication infrastructure for ROS 2. Enables the publish/subscribe and request/response patterns.

### Decision Making
The cognitive component of intelligent robotic systems where AI algorithms process perception data to determine appropriate actions.

## E

### End-Effector
The tool or device at the end of a robot manipulator that interacts with the environment.

## F

### Fixed Joint
A joint type in URDF that creates a rigid connection with no movement between links.

## G

### Goal
In action communication, the request sent to the action server specifying what the client wants the server to do.

## I

### Inertial
In URDF, the physical properties of a link that define its mass and moment of inertia for dynamics simulation.

### Inverse Kinematics (IK)
The process of calculating the required joint angles to achieve a desired end-effector pose.

## J

### Joint
In URDF, defines how links are connected and specifies the degrees of freedom between them.

### Joint State
A message type that contains the position, velocity, and effort of robot joints.

## K

### Kinematic Chain
A series of rigid links connected by joints that transmit motion from one end of the chain to the other.

## L

### Link
In URDF, represents a rigid part of the robot with visual, collision, and inertial properties.

## M

### Message
A data structure used for communication between ROS 2 nodes, defined in .msg files.

### MoveIt!
A motion planning framework for ROS 2 that provides inverse kinematics, path planning, and collision avoidance.

## N

### Node
A process that performs computation in ROS 2. The fundamental unit of execution that can contain publishers, subscribers, services, and actions.

## P

### Perception
The process by which robots understand their environment through sensor data processing.

### Prismatic Joint
A joint type in URDF that allows linear sliding motion along the joint axis.

### Publisher
A communication interface that sends messages to a topic in the publish/subscribe pattern.

## R

### Robot Operating System (ROS 2)
A flexible framework for writing robot software. Provides libraries, tools, and conventions for robot development.

### Robot State Publisher
A ROS 2 node that reads joint positions and publishes the appropriate transforms for robot links.

### ROS 2
The second generation of the Robot Operating System, designed for production environments with improved real-time capabilities and security.

### rclpy
The Python client library for ROS 2, providing the Python API for ROS 2 functionality.

### Revolute Joint
A joint type in URDF that allows rotational motion around the joint axis with limited range.

## S

### Service
A communication pattern in ROS 2 for request/response operations. A client sends a request and waits for a response.

### Subscriber
A communication interface that receives messages from a topic in the publish/subscribe pattern.

## T

### TF2 (Transform Library)
The transform library in ROS 2 that provides coordinate transforms between different frames.

### Topic
A named bus in ROS 2 for publish/subscribe communication. Publishers send data to topics, subscribers receive data from topics.

## U

### URDF (Unified Robot Description Format)
An XML-based format used in ROS to describe robots, including their physical structure, links, joints, and other properties.

## V

### Visual
In URDF, the geometric representation of a link used for visualization purposes.

## X

### Xacro
An XML macro language that extends URDF with features like macros, mathematical expressions, and conditional statements.