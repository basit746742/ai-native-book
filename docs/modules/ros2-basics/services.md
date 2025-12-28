---
title: "ROS 2 Services - Request/Response Pattern"
description: "Understanding services and the request/response communication pattern in ROS 2"
tags: ["ros2", "services", "request-response", "communication"]
sidebar_position: 4
difficulty: "beginner"
learning_objectives:
  - "Understand the request/response communication pattern"
  - "Identify when to use services for communication"
  - "Recognize the components of service-based communication"
time_estimated: "18 mins"
---

# ROS 2 Services - Request/Response Pattern

## Overview

Services provide a request/response communication pattern in ROS 2, where a client sends a request to a server and waits for a response. This is a synchronous communication pattern that's useful for operations that have a clear beginning and end.

## How Services Work

In the request/response pattern:

- **Service Clients** send a request to a named service and wait for a response
- **Service Servers** receive requests and send back responses
- There is typically one server for each service name
- Multiple clients can call the same service
- The client blocks until it receives a response (or times out)

## Characteristics

- **Bidirectional**: Request goes one way, response goes the other
- **Synchronous**: Client waits for response before continuing
- **Point-to-point**: Usually one server responds to one client at a time
- **Task-oriented**: Good for operations with clear input/output

## When to Use Services

Services are ideal for:
- Operations that require a specific result
- Configuration changes
- Action triggering (e.g., "take picture", "start process")
- Any operation that has a clear start and end

## Service Definition

Services are defined with two message types:
- **Request**: The data sent from client to server
- **Response**: The data sent from server to client

## Example Use Cases

- Navigation: "Plan a path to this location"
- Perception: "Detect objects in this image"
- Configuration: "Set the robot's operational mode"
- Calibration: "Calibrate the sensor system"