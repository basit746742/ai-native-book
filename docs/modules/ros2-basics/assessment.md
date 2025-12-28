---
title: "Assessment - ROS 2 Basics"
description: "Assessment questions to evaluate understanding of ROS 2 basics"
tags: ["ros2", "assessment", "topics", "services", "nodes"]
sidebar_position: 7
difficulty: "beginner"
learning_objectives:
  - "Demonstrate understanding of ROS 2 architecture"
  - "Distinguish between topics and services"
  - "Apply communication patterns appropriately"
time_estimated: "15 mins"
---

# Assessment - ROS 2 Basics

## Question 1: Conceptual Understanding
What is the main difference between topics and services in ROS 2?

A) Topics are faster than services
B) Topics use publish/subscribe pattern, services use request/response pattern
C) Topics can only send strings, services can send any data type
D) Topics are synchronous, services are asynchronous

<details>
<summary>Click to see answer</summary>
B) Topics use publish/subscribe pattern, services use request/response pattern
</details>

## Question 2: Application
Which communication method would be most appropriate for a node that continuously publishes sensor data to multiple other nodes?

A) Service
B) Topic
C) Parameter
D) Action

<details>
<summary>Click to see answer</summary>
B) Topic - The publish/subscribe pattern is ideal for broadcasting continuous sensor data to multiple subscribers
</details>

## Question 3: Scenario Analysis
Match the following scenarios to the appropriate communication method (Topic or Service):

A) Broadcasting the robot's current position to multiple nodes
B) Requesting the robot to move to a specific location
C) Requesting the robot to take a photograph
D) Streaming camera images for processing

<details>
<summary>Click to see answer</summary>
A) Topic - Broadcasting to multiple nodes, continuous status updates
B) Service - Request/response for a specific action with potential confirmation
C) Service - Task-oriented operation with clear request/response
D) Topic - Continuous stream of data that might be used by multiple nodes
</details>

## Question 4: Architecture
What is a ROS 2 node?

A) A type of message used for communication
B) A hardware component of the robot
C) A process that performs computation and can contain publishers, subscribers, etc.
D) A configuration file for ROS 2

<details>
<summary>Click to see answer</summary>
C) A process that performs computation and can contain publishers, subscribers, etc.
</details>