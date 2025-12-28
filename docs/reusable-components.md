---
title: "Reusable Components and Patterns"
description: "Common components and patterns used throughout the ROS 2 educational content"
sidebar_position: 7
---

# Reusable Components and Patterns

This page documents common components and patterns that can be reused across different modules in the ROS 2 Robotics Nervous System educational content.

## Code Example Templates

### Basic Node Template
```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        # Initialize communication interfaces here
        # Add your initialization code here

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Publisher Template
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Replace with appropriate message type

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(String, 'topic_name', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
```

### Subscriber Template
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Replace with appropriate message type

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String,  # Replace with appropriate message type
            'topic_name',  # Replace with actual topic name
            self.listener_callback,
            10  # Queue size
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
```

### Service Server Template
```python
import rclpy
from rclpy.node import Node
# Import your service type, e.g., from example_interfaces.srv import AddTwoInts

class ServiceServerNode(Node):
    def __init__(self):
        super().__init__('service_server')
        # Replace with your actual service type
        self.srv = self.create_service(
            AddTwoInts,  # Replace with actual service type
            'service_name',  # Replace with actual service name
            self.service_callback
        )

    def service_callback(self, request, response):
        # Process the request and set the response
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response
```

### Service Client Template
```python
import rclpy
from rclpy.node import Node
# Import your service type, e.g., from example_interfaces.srv import AddTwoInts

class ServiceClientNode(Node):
    def __init__(self):
        super().__init__('service_client')
        self.cli = self.create_client(AddTwoInts, 'service_name')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
```

## URDF Templates

### Basic Link Template
```xml
<link name="link_name">
  <visual>
    <geometry>
      <!-- Choose one: box, cylinder, sphere, or mesh -->
      <box size="0.1 0.1 0.1"/>
      <!-- OR <cylinder radius="0.1" length="0.1"/> -->
      <!-- OR <sphere radius="0.1"/> -->
      <!-- OR <mesh filename="package://path/to/mesh.stl"/> -->
    </geometry>
    <material name="material_name">
      <color rgba="1 0 0 1"/>  <!-- Red color example -->
    </material>
  </visual>

  <collision>
    <geometry>
      <!-- Usually similar to visual but can be simplified -->
      <box size="0.1 0.1 0.1"/>
    </geometry>
  </collision>

  <inertial>
    <mass value="1.0"/>
    <origin xyz="0 0 0"/>
    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
  </inertial>
</link>
```

### Basic Joint Template
```xml
<joint name="joint_name" type="joint_type">
  <!-- Joint types: fixed, continuous, revolute, prismatic -->
  <parent link="parent_link_name"/>
  <child link="child_link_name"/>
  <origin xyz="x y z" rpy="roll pitch yaw"/>
  <axis xyz="x y z"/>  <!-- Only for 1-DOF joints -->
  <!-- For revolute and prismatic joints with limits: -->
  <!--
  <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  -->
</joint>
```

## Documentation Templates

### Concept Page Template
```markdown
---
title: "Concept Title"
description: "Brief description of the concept"
tags: ["tag1", "tag2", "tag3"]
sidebar_position: 1
difficulty: "beginner" # or "intermediate", "advanced"
learning_objectives:
  - "Objective 1"
  - "Objective 2"
time_estimated: "15 mins" # Estimated time to complete
---

# Concept Title

## Overview
Brief introduction to the concept.

## Key Points
- Point 1
- Point 2
- Point 3

## Examples
Examples demonstrating the concept.

## Best Practices
Best practices related to the concept.

## Next Steps
How this concept connects to other concepts or modules.
```

### Example Code Block Template
```markdown
## Example: Example Title

```python
# Python code example
import rclpy
from rclpy.node import Node

class ExampleNode(Node):
    def __init__(self):
        super().__init__('example_node')
        # Example code here
```

### Explanation
Explanation of the example code and its purpose.
```

## Common Patterns

### 1. The ROS 2 Node Pattern
- Inherit from `rclpy.node.Node`
- Call `super().__init__('node_name')`
- Create communication interfaces in `__init__`
- Use `rclpy.spin()` to process callbacks

### 2. The Publisher-Subscriber Pattern
- Publishers send data to named topics
- Subscribers receive data from named topics
- Same topic name required for communication
- Message types must match between publisher and subscriber

### 3. The Service Pattern
- Service servers provide functionality
- Service clients request functionality
- Request-response communication model
- Synchronous operation

### 4. The URDF Link-Joint Pattern
- Links define rigid bodies
- Joints define connections between links
- Parent-child relationships form kinematic chains
- Coordinate frames created for each link

## Educational Components

### Exercise Template
```markdown
## Exercise: Exercise Title

### Question
The question or problem to solve.

### Your Answer
<details>
<summary>Click to see answer</summary>
The answer and explanation.
</details>
```

### Assessment Template
```markdown
## Question N: Question Title
The question text.

A) Option A
B) Option B
C) Option C
D) Option D

<details>
<summary>Click to see answer</summary>
Correct answer and explanation.
</details>
```

## Common Error Patterns and Solutions

### 1. Topic Connection Issues
**Problem**: Nodes not communicating
**Solution**: Check topic names match exactly, message types match

### 2. URDF Parsing Errors
**Problem**: Invalid XML or missing elements
**Solution**: Validate XML syntax, ensure required elements present

### 3. Transform Issues
**Problem**: Missing transforms in RViz
**Solution**: Check robot_state_publisher running, joint states published

These reusable components and patterns provide a consistent foundation for developing ROS 2 applications and educational content.