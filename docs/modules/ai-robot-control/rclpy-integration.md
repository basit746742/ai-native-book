---
title: "rclpy Integration - Python ROS 2 Client Library"
description: "Understanding how to use rclpy to connect Python AI agents to ROS 2"
tags: ["ros2", "rclpy", "python", "ai", "integration"]
sidebar_position: 4
difficulty: "intermediate"
learning_objectives:
  - "Understand the rclpy client library"
  - "Learn how to create ROS 2 nodes in Python"
  - "Integrate Python AI agents with ROS 2 systems"
time_estimated: "30 mins"
---

# rclpy Integration - Python ROS 2 Client Library

## Overview

rclpy is the Python client library for ROS 2. It provides the Python API that allows Python programs to interact with the ROS 2 middleware, enabling Python-based AI agents to communicate with other ROS 2 nodes.

## Core Components of rclpy

### Node
The fundamental building block of a ROS 2 program in Python. All communication elements (publishers, subscribers, services, etc.) are associated with a node.

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        # Initialize communication interfaces here
```

### Publishers
Used to send messages to topics:

```python
self.publisher = self.create_publisher(String, 'topic_name', 10)
msg = String()
msg.data = 'Hello World'
self.publisher.publish(msg)
```

### Subscribers
Used to receive messages from topics:

```python
self.subscription = self.create_subscription(
    String,
    'topic_name',
    self.listener_callback,
    10
)

def listener_callback(self, msg):
    self.get_logger().info('Received: %s' % msg.data)
```

### Services
For request/response communication:

```python
# Server side
self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

def add_two_ints_callback(self, request, response):
    response.sum = request.a + request.b
    self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
    return response

# Client side
self.cli = self.create_client(AddTwoInts, 'add_two_ints')
```

### Actions
For long-running tasks with feedback:

```python
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

goal_msg = NavigateToPose.Goal()
goal_msg.pose = pose
send_goal_future = self._action_client.send_goal_async(goal_msg)
```

## Integrating AI Agents with rclpy

### Pattern 1: AI Node
Create a dedicated node that runs AI algorithms and communicates via ROS 2:

```python
import rclpy
from rclpy.node import Node
import tensorflow as tf  # Example AI library

class AIAgentNode(Node):
    def __init__(self):
        super().__init__('ai_agent_node')

        # Subscribe to sensor/perception data
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

        # Publish decisions/actions
        self.publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

    def image_callback(self, msg):
        # Process image with AI model
        processed_data = self.ai_model.predict(msg)

        # Convert to robot command
        cmd = self.convert_to_command(processed_data)

        # Publish command
        self.publisher.publish(cmd)
```

### Pattern 2: AI as Service
Provide AI capabilities as a service to other nodes:

```python
class AIPerceptionService(Node):
    def __init__(self):
        super().__init__('ai_perception_service')
        self.srv = self.create_service(
            ObjectDetection,
            'detect_objects',
            self.detect_objects_callback
        )
        # Load AI model
        self.ai_model = self.load_model()

    def detect_objects_callback(self, request, response):
        # Process request with AI
        results = self.ai_model.process(request.image)

        # Format response
        response.objects = results
        return response
```

## Best Practices for AI Integration

### 1. Separate AI Logic from ROS 2 Communication
Keep your AI algorithms separate from ROS 2 communication code for better testability:

```python
class AIModel:
    def __init__(self):
        # Initialize AI model
        pass

    def process(self, data):
        # Pure AI processing
        return results

class AINode(Node):
    def __init__(self):
        super().__init__('ai_node')
        self.ai_model = AIModel()
        # ROS 2 setup
```

### 2. Handle Asynchronous Operations
Use appropriate callbacks and async patterns:

```python
def image_callback(self, msg):
    # Process in background thread if needed
    future = self.executor.create_task(self.process_image_async, msg)
```

### 3. Resource Management
AI models can be resource-intensive, so manage memory and computation carefully:

```python
def __init__(self):
    super().__init__('ai_node')
    # Initialize model once
    self.ai_model = self.initialize_model()
    # Set up timers to control processing rate
    self.timer = self.create_timer(0.1, self.process_if_needed)
```

## Quality of Service Considerations

For AI nodes using rclpy:
- Consider the processing time of AI algorithms when setting QoS policies
- Use appropriate history depth based on AI algorithm needs
- Consider reliability for safety-critical AI decisions

## Example: Complete AI Integration Node

A complete example showing how to integrate a simple AI agent with ROS 2:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class SimpleAIAvoidanceNode(Node):
    def __init__(self):
        super().__init__('simple_ai_avoidance')

        # Subscribe to laser scan data
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )

        # Publish velocity commands
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.get_logger().info('Simple AI Avoidance Node Started')

    def scan_callback(self, msg):
        # Simple AI: if obstacle is close, turn; otherwise go forward
        min_distance = min(msg.ranges)

        cmd = Twist()
        if min_distance < 1.0:  # If obstacle within 1 meter
            cmd.angular.z = 0.5  # Turn
        else:
            cmd.linear.x = 0.5   # Go forward

        self.publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    ai_node = SimpleAIAvoidanceNode()

    try:
        rclpy.spin(ai_node)
    except KeyboardInterrupt:
        pass
    finally:
        ai_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This demonstrates the complete perception → decision → action flow using rclpy to connect a simple AI agent to a ROS 2 robot system.