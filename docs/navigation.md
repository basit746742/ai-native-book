---
title: "Module Navigation Guide"
description: "Guide to navigating between different modules in the ROS 2 Robotics Nervous System"
sidebar_position: 3
---

# Module Navigation Guide

This guide helps you navigate efficiently between the different modules in the ROS 2 Robotics Nervous System educational content.

## Module Progression

The modules are designed to be taken in sequence for optimal learning:

```
[ROS 2 Basics] → [AI-to-Robot Control] → [Humanoid Modeling with URDF]
```

Each module builds on the concepts from the previous one, but they are designed to be independently testable as well.

## Quick Navigation Links

### ROS 2 Basics Module
- [Introduction to ROS 2 Architecture](./modules/ros2-basics/introduction)
- [Understanding ROS 2 Nodes](./modules/ros2-basics/nodes)
- [Topics - Publish/Subscribe Pattern](./modules/ros2-basics/topics)
- [Services - Request/Response Pattern](./modules/ros2-basics/services)
- [Practical Examples](./modules/ros2-basics/examples)
- [Conceptual Exercises](./modules/ros2-basics/exercises)
- [Module Assessment](./modules/ros2-basics/assessment)

### AI-to-Robot Control Module
- [Perception in ROS 2](./modules/ai-robot-control/perception)
- [Decision Making in AI-Robot Systems](./modules/ai-robot-control/decision-making)
- [Action Execution in ROS 2](./modules/ai-robot-control/action)
- [rclpy Integration](./modules/ai-robot-control/rclpy-integration)
- [Practical Examples](./modules/ai-robot-control/examples)
- [Conceptual Exercises](./modules/ai-robot-control/exercises)
- [Module Assessment](./modules/ai-robot-control/assessment)

### Humanoid Modeling with URDF Module
- [Introduction to URDF](./modules/urdf-modeling/introduction)
- [URDF Links - Physical Components](./modules/urdf-modeling/links)
- [URDF Joints - Connections Between Components](./modules/urdf-modeling/joints)
- [Kinematic Chains - Robot Structure](./modules/urdf-modeling/kinematic-chains)
- [URDF Integration with ROS 2](./modules/urdf-modeling/ros2-integration)
- [Practical Examples](./modules/urdf-modeling/examples)
- [Conceptual Exercises](./modules/urdf-modeling/exercises)
- [Module Assessment](./modules/urdf-modeling/assessment)

## Cross-Module Concepts

Some concepts span multiple modules:

### Communication Patterns
- **Topics**: Introduced in [ROS 2 Basics](./modules/ros2-basics/topics), applied in [AI Control](./modules/ai-robot-control/perception)
- **Services**: Introduced in [ROS 2 Basics](./modules/ros2-basics/services), applied in [AI Control](./modules/ai-robot-control/decision-making)

### Coordinate Systems
- **Frames**: Introduced in [ROS 2 Basics](./modules/ros2-basics/introduction), expanded in [URDF Integration](./modules/urdf-modeling/ros2-integration)

### AI Integration
- **rclpy**: Introduced in [AI Control](./modules/ai-robot-control/rclpy-integration), used with [URDF models](./modules/ai-robot-control/rclpy-integration) for complete systems

## Assessment Progression

Each module has its own assessment, but the concepts build on each other:

1. Complete the [ROS 2 Basics Assessment](./modules/ros2-basics/assessment) to verify communication pattern understanding
2. Take the [AI-to-Robot Control Assessment](./modules/ai-robot-control/assessment) to confirm integration knowledge
3. Finish with the [URDF Modeling Assessment](./modules/urdf-modeling/assessment) to validate modeling skills

## Next Steps

After completing all modules, you'll be prepared for Module 2 on simulation and digital twins.

## Quick Reference

| Concept | Module 1 (ROS 2 Basics) | Module 2 (AI Control) | Module 3 (URDF Modeling) |
|---------|------------------------|----------------------|-------------------------|
| Nodes | ✅ Introduction | ✅ Implementation | ✅ Application |
| Topics/Services | ✅ Core concepts | ✅ Application | ⚠️ Integration |
| AI Integration | ❌ Not covered | ✅ Core focus | ✅ Application |
| Robot Modeling | ❌ Not covered | ⚠️ Basic usage | ✅ Core focus |
| Perception | ⚠️ Basic mention | ✅ Core focus | ⚠️ Application |
| Kinematics | ❌ Not covered | ❌ Not covered | ✅ Core focus |

*Legend: ✅ = Primary focus, ⚠️ = Mentioned/applied, ❌ = Not covered*