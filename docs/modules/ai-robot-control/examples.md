---
title: "Practical Examples - Perception → Decision → Action Flow"
description: "Practical examples demonstrating the complete perception → decision → action flow in AI-robot systems"
tags: ["ros2", "examples", "ai", "perception", "decision", "action", "rclpy"]
sidebar_position: 5
difficulty: "intermediate"
learning_objectives:
  - "Implement the complete perception → decision → action flow"
  - "Connect AI agents to ROS 2 using rclpy"
  - "Understand the integration of all components"
time_estimated: "35 mins"
---

# Practical Examples - Perception → Decision → Action Flow

## Overview

This section provides practical examples that demonstrate the complete perception → decision → action flow in AI-robot systems using ROS 2. These examples show how to integrate AI algorithms with ROS 2 communication patterns.

## Complete Example: AI Object Recognition and Navigation

This example demonstrates a complete system that:
1. **Perception**: Detects objects in camera images
2. **Decision**: Determines which object to approach based on AI processing
3. **Action**: Navigates to the selected object

### Perception Node: Object Detection

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
import cv2
from cv2 import aruco
import numpy as np

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        # Subscribe to camera images
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

        # Publish detected objects
        self.publisher = self.create_publisher(
            Detection2DArray,
            'detected_objects',
            10
        )

        # Initialize simple detection (using ArUco markers as example)
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters_create()

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV
        image = self.ros_to_cv2(msg)

        # Detect ArUco markers
        corners, ids, _ = aruco.detectMarkers(image, self.aruco_dict, parameters=self.parameters)

        # Create detection message
        detections = Detection2DArray()
        detections.header = msg.header

        if ids is not None:
            for i, corner in enumerate(corners):
                detection = Detection2D()
                detection.header = msg.header
                # Simplified detection representation
                center_x = int(np.mean(corner[0][:, 0]))
                center_y = int(np.mean(corner[0][:, 1]))
                detection.bbox.center.x = center_x
                detection.bbox.center.y = center_y
                # Add to detections
                detections.detections.append(detection)

        # Publish detections
        self.publisher.publish(detections)

    def ros_to_cv2(self, ros_image):
        # Simplified conversion (in practice, use cv_bridge)
        import numpy as np
        dtype = np.uint8
        image_np = np.frombuffer(ros_image.data, dtype=dtype)
        image_np = image_np.reshape((ros_image.height, ros_image.width, 3))
        return image_np

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Decision Node: AI-Based Object Selection

```python
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseStamped
import math

class AIDecisionNode(Node):
    def __init__(self):
        super().__init__('ai_decision_node')

        # Subscribe to detected objects
        self.subscription = self.create_subscription(
            Detection2DArray,
            'detected_objects',
            self.detection_callback,
            10
        )

        # Publish navigation goal
        self.nav_publisher = self.create_publisher(
            PoseStamped,
            'navigation_goal',
            10
        )

        # Keep track of objects for decision making
        self.known_objects = {}

    def detection_callback(self, msg):
        # Process detected objects
        for detection in msg.detections:
            # Simple AI: select the closest object that hasn't been visited recently
            distance = self.calculate_distance_to_robot(detection.bbox.center)

            # Store object info
            obj_id = self.get_object_id(detection)
            self.known_objects[obj_id] = {
                'position': detection.bbox.center,
                'distance': distance,
                'last_visited': self.get_clock().now()
            }

        # Select the best object to approach
        best_object = self.select_best_object()

        if best_object:
            # Create navigation goal
            goal = self.create_navigation_goal(best_object)
            self.nav_publisher.publish(goal)

    def calculate_distance_to_robot(self, position):
        # Simplified: assume robot is at (0,0)
        return math.sqrt(position.x**2 + position.y**2)

    def get_object_id(self, detection):
        # Simplified object identification
        return f"obj_{detection.bbox.center.x}_{detection.bbox.center.y}"

    def select_best_object(self):
        # AI decision logic: select closest unvisited object
        if not self.known_objects:
            return None

        # Find closest object
        closest_obj = min(self.known_objects.values(), key=lambda x: x['distance'])
        return closest_obj

    def create_navigation_goal(self, obj):
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        goal.pose.position.x = obj['position'].x
        goal.pose.position.y = obj['position'].y
        goal.pose.orientation.w = 1.0  # No rotation
        return goal

def main(args=None):
    rclpy.init(args=args)
    node = AIDecisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Action Node: Navigation

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class NavigationActionNode(Node):
    def __init__(self):
        super().__init__('navigation_action_node')

        # Subscribe to navigation goals
        self.subscription = self.create_subscription(
            PoseStamped,
            'navigation_goal',
            self.goal_callback,
            10
        )

        # Create action client for navigation
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

    def goal_callback(self, msg):
        self.get_logger().info(f'Received navigation goal: {msg.pose.position}')

        # Wait for action server
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Navigation action server not available')
            return

        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg

        # Send goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        # Get result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Navigation feedback: {feedback.current_pose}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')
        # Navigation completed - ready for next decision

def main(args=None):
    rclpy.init(args=args)
    node = NavigationActionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## Key Integration Points

### 1. Message Type Consistency
All nodes must use compatible message types. In the example:
- Perception node publishes `Detection2DArray`
- Decision node subscribes to `Detection2DArray` and publishes `PoseStamped`
- Action node subscribes to `PoseStamped` and uses `NavigateToPose` action

### 2. Timing and Synchronization
Consider the timing requirements:
- Perception nodes may run at high frequency (30 Hz for cameras)
- Decision nodes may run at lower frequency (1-5 Hz)
- Action nodes respond to goals as they arrive

### 3. Error Handling
Implement proper error handling in each component:
- Perception: Handle sensor failures
- Decision: Handle cases where no valid objects are detected
- Action: Handle navigation failures and timeouts

## Quality of Service Settings

For the perception → decision → action flow:

```python
# Perception node - high frequency data
self.subscription = self.create_subscription(
    Image,
    'camera/image_raw',
    self.image_callback,
    rclpy.qos.QoSProfile(
        depth=1,  # Only latest image needed
        reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,  # May drop frames
        durability=rclpy.qos.DurabilityPolicy.VOLATILE
    )
)

# Decision node - reliable communication
self.publisher = self.create_publisher(
    PoseStamped,
    'navigation_goal',
    rclpy.qos.QoSProfile(
        depth=10,
        reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
        durability=rclpy.qos.DurabilityPolicy.VOLATILE
    )
)
```

## Running the Complete System

1. Start the navigation stack (if using real navigation)
2. Run the perception node: `ros2 run my_package object_detection_node`
3. Run the decision node: `ros2 run my_package ai_decision_node`
4. Run the action node: `ros2 run my_package navigation_action_node`
5. Provide camera images (from real camera or bag file)

The system will then:
1. Detect objects in the camera images
2. Select the most appropriate object to approach
3. Navigate to that object
4. Repeat the cycle

This complete example demonstrates how AI algorithms can be integrated into the ROS 2 perception → decision → action flow using rclpy for Python integration.