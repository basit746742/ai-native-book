---
title: "Action Execution in ROS 2"
description: "Understanding how commands are executed in ROS 2 AI-robot systems"
tags: ["ros2", "actions", "control", "execution"]
sidebar_position: 3
difficulty: "intermediate"
learning_objectives:
  - "Understand the role of action execution in AI-robot systems"
  - "Distinguish between actions, topics, and services"
  - "Recognize how AI decisions are converted to robot actions"
time_estimated: "20 mins"
---

# Action Execution in ROS 2

## Overview

Action execution is the final component of the perception → decision → action flow in AI-robot systems. In ROS 2, actions provide a communication pattern for long-running tasks that may provide feedback and can be canceled.

## What Are Actions in ROS 2?

Actions are a communication pattern in ROS 2 that are designed for long-running tasks. They combine the best of both topics and services:

- Like services: They have a goal (request) and result (response)
- Like topics: They provide feedback during execution
- Additional feature: They can be canceled after starting

## Action Components

### Goal
- The request sent to the action server
- Specifies what the client wants the server to do
- Includes any parameters needed for the task

### Feedback
- Status updates sent from the server to the client during execution
- Allows the client to monitor progress
- Can be used for visualization or adaptive behavior

### Result
- The final outcome of the action
- Sent when the action completes (successfully or unsuccessfully)
- Contains the final status and any output data

## When to Use Actions vs Other Communication Types

### Use Actions for:
- Long-running tasks (e.g., navigation to a distant location)
- Tasks that benefit from progress monitoring
- Tasks that might need to be canceled
- Complex operations with multiple steps

### Use Topics for:
- Continuous data streams
- Broadcasting information to multiple nodes
- Real-time sensor data

### Use Services for:
- Quick request/response operations
- Configuration changes
- Tasks with immediate results

## Common Action Examples in Robotics

### Navigation Actions
- **Goal**: Navigate to a specific location
- **Feedback**: Current progress, remaining distance
- **Result**: Success/failure, final pose

### Manipulation Actions
- **Goal**: Pick up an object at a specific location
- **Feedback**: Current gripper position, grasp status
- **Result**: Success/failure, object status

### Calibration Actions
- **Goal**: Calibrate a sensor system
- **Feedback**: Current calibration step, progress
- **Result**: Calibration success, parameters

## Integration with AI Systems

In AI-robot systems, the typical flow is:
1. **Perception**: Sensor data processed to understand the environment
2. **Decision**: AI algorithms determine what action to take
3. **Action**: An action goal is sent to an appropriate action server
4. **Execution**: The action server executes the task, providing feedback
5. **Monitoring**: The AI system monitors progress and can adapt as needed

## Quality of Service Considerations

For action-based communication:
- **Reliability**: Usually required for action goals and results
- **Durability**: Transient for goals that need to persist for new clients
- **History**: Keep last for feedback streams

## Example: Navigation Action Flow

1. Perception system detects a goal location
2. AI decision system determines navigation is needed
3. Action client sends a navigation goal to the navigation action server
4. Action server begins navigation, sending feedback about progress
5. AI system monitors progress and can cancel if conditions change
6. Action server completes navigation and sends final result