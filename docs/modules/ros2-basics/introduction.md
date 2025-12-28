---
title: "Introduction to ROS 2 Architecture"
description: "Understanding the fundamental architecture of ROS 2 and its communication patterns"
tags: ["ros2", "architecture", "communication"]
sidebar_position: 1
difficulty: "beginner"
learning_objectives:
  - "Understand the core concepts of ROS 2"
  - "Identify the main components of ROS 2 architecture"
  - "Recognize the purpose of different communication patterns"
time_estimated: "20 mins"
---

# Introduction to ROS 2 Architecture

## What is ROS 2?

ROS 2 (Robot Operating System 2) is not an actual operating system, but rather a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

## Core Architecture Concepts

ROS 2 uses a distributed architecture where computation is spread across multiple processes (potentially running on multiple machines) that interact through a publish/subscribe messaging model.

### Nodes
Nodes are the fundamental unit of execution in ROS 2. Each node is a process that performs computation. Nodes are designed to be modular, with each node responsible for a specific task. Nodes can be written in different programming languages (C++, Python, etc.) and can run on different machines.

### Communication Primitives
ROS 2 provides several ways for nodes to communicate with each other:

- **Topics**: For streaming data using a publish/subscribe pattern
- **Services**: For remote procedure calls using a request/response pattern
- **Actions**: For long-running tasks with feedback and goal management
- **Parameters**: For configuration data that can be shared between nodes

## The ROS 2 Middleware

ROS 2 uses DDS (Data Distribution Service) as its middleware. DDS provides the underlying communication infrastructure that enables the publish/subscribe and request/response patterns. The choice of DDS implementation can be changed at runtime, allowing for different quality of service characteristics.

## Why ROS 2?

ROS 2 was developed to address limitations in the original ROS, including:
- Real-time support
- Multi-robot systems
- Deterministic behavior
- Security features
- Deployment in production environments