---
title: "Assessment - AI-to-Robot Control"
description: "Assessment questions to evaluate understanding of AI-to-robot control concepts"
tags: ["ros2", "assessment", "ai", "integration", "perception", "decision", "action"]
sidebar_position: 7
difficulty: "intermediate"
learning_objectives:
  - "Demonstrate understanding of AI-to-robot integration"
  - "Apply perception → decision → action concepts"
  - "Choose appropriate communication patterns"
time_estimated: "20 mins"
---

# Assessment - AI-to-Robot Control

## Question 1: Architecture Understanding
What is the correct order of the perception → decision → action flow in AI-robot systems?

A) Decision → Perception → Action
B) Perception → Action → Decision
C) Perception → Decision → Action
D) Action → Perception → Decision

<details>
<summary>Click to see answer</summary>
C) Perception → Decision → Action - First the robot perceives the environment through sensors, then the AI system makes decisions based on this information, then the robot acts on those decisions.
</details>

## Question 2: Communication Pattern
Which ROS 2 communication pattern is most appropriate for a long-running task that provides feedback during execution?

A) Topic
B) Service
C) Action
D) Parameter

<details>
<summary>Click to see answer</summary>
C) Action - Actions are specifically designed for long-running tasks that provide feedback and can be canceled, unlike topics (continuous) or services (request/response).
</details>

## Question 3: rclpy Integration
Which of the following is the correct way to create a subscriber in rclpy?

A) `self.subscription = self.create_subscription(MessageType, 'topic_name', callback, qos_profile)`
B) `self.subscription = self.create_subscriber(MessageType, 'topic_name', callback)`
C) `self.subscription = Subscriber(MessageType, 'topic_name', callback)`
D) `subscription = create_subscription(MessageType, 'topic_name', callback)`

<details>
<summary>Click to see answer</summary>
A) `self.subscription = self.create_subscription(MessageType, 'topic_name', callback, qos_profile)` - This is the correct method signature in rclpy.
</details>

## Question 4: System Design
In a robot system that needs to detect objects, decide what to do, and execute actions, which of the following represents the best architectural design?

A) One monolithic node that handles all perception, decision, and action
B) Separate nodes for perception, decision, and action that communicate via topics
C) All functionality in the main control loop without ROS 2
D) A single service that handles everything

<details>
<summary>Click to see answer</summary>
B) Separate nodes for perception, decision, and action that communicate via topics - This follows the ROS 2 design principle of modularity and allows each component to be developed and tested independently.
</details>

## Question 5: Quality of Service
For a safety-critical perception system that detects obstacles, which QoS profile would be most appropriate?

A) Best effort, volatile, keep last
B) Reliable, durable, keep all
C) Reliable, volatile, keep last
D) Best effort, durable, keep all

<details>
<summary>Click to see answer</summary>
C) Reliable, volatile, keep last - For safety, you need reliable delivery (messages must arrive), but only the most recent obstacle information is relevant (volatile), and you only need the latest data (keep last).
</details>

## Question 6: AI Integration
When integrating a machine learning model with ROS 2, what is the best practice for code organization?

A) Put all AI code directly in the ROS 2 callbacks
B) Keep AI logic separate from ROS 2 communication code
C) Use global variables for sharing data between AI and ROS 2 parts
D) Only run AI code in services, never in nodes

<details>
<summary>Click to see answer</summary>
B) Keep AI logic separate from ROS 2 communication code - This makes the AI components more testable, reusable, and maintainable by separating concerns.
</details>

## Question 7: Communication Flow
In the perception → decision → action flow, what typically happens when the decision-making process is slow?

A) The robot stops until a decision is made
B) The perception system stops collecting data
C) The system may queue decisions or use the latest available decision
D) The action system ignores the delay

<details>
<summary>Click to see answer</summary>
C) The system may queue decisions or use the latest available decision - In real systems, there's often a trade-off between using fresh but delayed decisions versus stale but timely decisions.
</details>

## Question 8: Node Responsibilities
Which of the following should typically be handled by a decision-making node in an AI-robot system?

A) Converting sensor raw data to processed information
B) Executing physical movements of robot hardware
C) Processing sensor information to determine appropriate actions
D) Publishing sensor data to topics

<details>
<summary>Click to see answer</summary>
C) Processing sensor information to determine appropriate actions - The decision-making node takes in processed perception data and determines what actions should be taken.
</details>