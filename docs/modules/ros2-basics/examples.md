---
title: "Practical Examples - Publisher/Subscriber"
description: "Practical examples demonstrating basic publisher/subscriber implementation in ROS 2"
tags: ["ros2", "examples", "publisher", "subscriber", "rclpy"]
sidebar_position: 5
difficulty: "beginner"
learning_objectives:
  - "Implement a basic publisher node"
  - "Implement a basic subscriber node"
  - "Understand the relationship between publisher and subscriber"
time_estimated: "25 mins"
---

# Practical Examples - Publisher/Subscriber

## Overview

This section provides practical examples of implementing publisher and subscriber nodes in ROS 2 using Python (rclpy). These examples demonstrate the fundamental publish/subscribe communication pattern.

## Publisher Example

Here's a simple publisher that sends a "Hello World" message every 2 seconds:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Subscriber Example

Here's a simple subscriber that receives messages from the publisher:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Key Components Explained

### Publisher Components:
- **Node Creation**: Creates a ROS 2 node with a name
- **Publisher Creation**: Creates a publisher for a specific message type and topic
- **Timer**: Triggers message publishing at regular intervals
- **Message Creation**: Creates and populates message objects
- **Publish Call**: Sends the message to the topic

### Subscriber Components:
- **Node Creation**: Creates a ROS 2 node with a name
- **Subscription Creation**: Creates a subscription to a specific message type and topic
- **Callback Function**: Processes incoming messages
- **Message Processing**: Handles the received message data

## Running the Examples

1. Source your ROS 2 installation: `source /opt/ros/humble/setup.bash`
2. Run the publisher in one terminal: `python3 publisher.py`
3. Run the subscriber in another terminal: `python3 subscriber.py`
4. You should see messages being published and received

## Key Takeaways

- Publishers and subscribers must use the same topic name to communicate
- Message types must match between publisher and subscriber
- Publishers and subscribers can be run independently and will connect automatically
- The communication is asynchronous - publishers don't wait for subscribers