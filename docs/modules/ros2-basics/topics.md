---
title: "ROS 2 Topics - Publish/Subscribe Pattern"
description: "Understanding topics and the publish/subscribe communication pattern in ROS 2"
tags: ["ros2", "topics", "publish-subscribe", "communication"]
sidebar_position: 3
difficulty: "beginner"
learning_objectives:
  - "Understand the publish/subscribe communication pattern"
  - "Identify when to use topics for communication"
  - "Recognize the components of topic-based communication"
time_estimated: "20 mins"
---

# ROS 2 Topics - Publish/Subscribe Pattern

## Overview

Topics are one of the primary communication mechanisms in ROS 2, using a publish/subscribe pattern. This pattern allows for asynchronous, decoupled communication between nodes where publishers send data and subscribers receive data.

## How Topics Work

In the publish/subscribe pattern:

- **Publishers** send data to a named topic
- **Subscribers** receive data from a named topic
- Multiple publishers can publish to the same topic
- Multiple subscribers can subscribe to the same topic
- Publishers and subscribers are decoupled - they don't need to know about each other

## Characteristics

- **Unidirectional**: Data flows from publisher to subscriber
- **Asynchronous**: Publishers and subscribers don't need to be synchronized
- **Broadcast**: One message can be received by multiple subscribers
- **Continuous**: Data is continuously streamed while the publisher is active

## When to Use Topics

Topics are ideal for:
- Streaming sensor data (camera images, LIDAR scans, IMU data)
- Continuous control commands
- Status updates
- Any data that needs to be broadcast to multiple nodes

## Quality of Service (QoS)

Topics support Quality of Service settings that control how messages are delivered:
- **Reliability**: Best effort or reliable delivery
- **Durability**: Volatile or transient-local (persistence)
- **History**: Keep-all or keep-last (buffer size)

## Example Use Cases

- Camera nodes publishing image streams
- IMU nodes publishing sensor readings
- Control nodes publishing velocity commands
- Status nodes publishing robot state information