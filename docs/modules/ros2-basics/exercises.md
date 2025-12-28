---
title: "Conceptual Exercises - Topics vs Services"
description: "Exercises to help understand the difference between topics and services in ROS 2"
tags: ["ros2", "exercises", "topics", "services", "communication"]
sidebar_position: 6
difficulty: "beginner"
learning_objectives:
  - "Distinguish between topics and services"
  - "Apply communication patterns appropriately"
  - "Analyze scenarios for proper communication choice"
time_estimated: "20 mins"
---

# Conceptual Exercises - Topics vs Services

## Exercise 1: Scenario Analysis

For each scenario below, determine whether topics or services would be the appropriate communication method and explain why.

### Scenario A: Temperature Sensor
A temperature sensor node continuously reads the room temperature and needs to share this data with other nodes in the system.

**Your Answer:**
<details>
<summary>Click to see answer</summary>
Topics - This is a continuous stream of sensor data that multiple nodes might want to receive simultaneously. The publish/subscribe pattern is ideal for this use case.
</details>

### Scenario B: Navigation Request
A high-level planning node needs to request a path from a navigation node, specifying a start and goal location.

**Your Answer:**
<details>
<summary>Click to see answer</summary>
Service - This is a request/response operation where the planner sends a request (start and goal) and expects a specific response (the path). This has a clear input/output relationship.
</details>

### Scenario C: Robot Status
A robot state node needs to broadcast the current battery level, position, and operational status to all interested nodes.

**Your Answer:**
<details>
<summary>Click to see answer</summary>
Topics - This is continuous status information that multiple nodes may need simultaneously. The broadcast nature fits the publish/subscribe pattern.
</details>

## Exercise 2: Communication Pattern Matching

Match the communication characteristics with the appropriate ROS 2 primitive (Topic or Service):

1. Synchronous communication
2. Asynchronous communication
3. Broadcast to multiple receivers
4. Point-to-point request/response
5. Continuous data streaming
6. Task-oriented operations

<details>
<summary>Click to see answers</summary>
1. Service
2. Topic
3. Topic
4. Service
5. Topic
6. Service
</details>

## Exercise 3: Design Challenge

You are designing a robot system with the following components:
- Camera node (publishes images)
- Object detection node (processes images and detects objects)
- Path planner node (plans navigation paths)
- Motor controller node (controls robot movement)
- UI node (displays status and accepts commands)

For each communication need, choose the appropriate communication method (topic or service) and explain your reasoning:

1. Camera sending images to object detection node
2. UI requesting current robot position
3. Path planner sending navigation commands to motor controller
4. Robot periodically reporting battery status to UI

<details>
<summary>Click to see answers</summary>
1. Topic - Continuous image stream that might be used by multiple nodes
2. Service - Request/response for specific information
3. Topic - Continuous command stream for robot movement
4. Topic - Continuous status broadcast to interested nodes
</details>

## Exercise 4: Quality Considerations

Consider the same robot system above. For each communication, what Quality of Service (QoS) considerations might be important?

1. Camera images to object detection
2. Navigation commands to motor controller
3. Battery status to UI

<details>
<summary>Click to see answers</summary>
1. Camera images: May want best effort for performance, keep last few images for latest data
2. Navigation commands: Need reliable delivery, may want transient durability
3. Battery status: Reliable delivery important, may want to keep last value for new subscribers
</details>