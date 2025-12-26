---
title: "Chapter 1 - ROS 2 Fundamentals: Nodes, Topics, and Services"
sidebar_label: Chapter 1 - Fundamentals
description: Understanding the core concepts of ROS 2 including nodes, topics, and services
---

# ROS 2 Fundamentals: Nodes, Topics, and Services

## Introduction

Welcome to Module 1: The Robotic Nervous System (ROS 2). In this chapter, we'll explore the fundamental concepts that form the backbone of robotic software development using ROS 2 (Robot Operating System 2). Think of ROS 2 as the "nervous system" of a robot - it enables different components to communicate and coordinate effectively.

## What is ROS 2?

ROS 2 is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms. Unlike traditional software frameworks, ROS 2 is designed specifically for the distributed nature of robotic systems.

## Core Concepts

### Nodes

A **node** is an individual process that performs computation. In ROS 2, nodes are the basic building blocks of a system. Each node can perform a specific task and communicate with other nodes to achieve complex behaviors.

**Characteristics of Nodes:**
- Each node runs as a separate process
- Nodes can be written in different programming languages (C++, Python, etc.)
- Nodes communicate with each other through topics, services, and actions
- A single robot system can have dozens or even hundreds of nodes running simultaneously

**Example:**
```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World'
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Topics

A **topic** provides a publish-subscribe communication pattern for continuous data streams. Topics enable asynchronous communication between nodes, where one or more nodes (publishers) send data to one or more other nodes (subscribers).

**Key Points about Topics:**
- Topics support many-to-many communication
- Publishers don't know who subscribes to their data
- Subscribers don't know who publishes the data they receive
- Topics are ideal for sensor data, robot state information, and other continuous streams

**Example:**
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

### Services

A **service** provides request-response communication for specific tasks that require a reply. Services enable synchronous communication where a client sends a request and waits for a response from a server.

**Key Points about Services:**
- Services provide synchronous, request-response communication
- Services are ideal for tasks that require a specific response
- Each service has exactly one server and can have multiple clients
- Services are good for actions like changing robot state, executing specific commands, etc.

**Example:**
```python
# Service definition (in srv/AddTwoInts.srv)
int64 a
int64 b
---
int64 sum

# Service server
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Communication Patterns Comparison

| Aspect | Topics | Services |
|--------|--------|----------|
| Communication Type | Asynchronous | Synchronous |
| Data Flow | Continuous stream | Request/Response |
| Use Case | Sensor data, robot state | Specific actions, commands |
| Latency | Low (fire and forget) | Higher (wait for response) |
| Number of Publishers/Servers | Many publishers, many subscribers | One server, many clients |

## Practical Example: A Simple Robot System

Let's consider a simple mobile robot with the following nodes:

1. **Laser Scanner Node**: Publishes laser scan data on the `/scan` topic
2. **Navigation Node**: Subscribes to `/scan` and publishes velocity commands to `/cmd_vel`
3. **Motor Controller Node**: Subscribes to `/cmd_vel` and controls the physical motors
4. **Emergency Stop Service**: Provides a service at `/emergency_stop` that can be called to immediately stop the robot

This demonstrates how nodes, topics, and services work together to create a complete robotic system.

## Summary

In this chapter, we've covered the fundamental concepts of ROS 2:

- **Nodes** are the basic computational elements
- **Topics** enable asynchronous, continuous communication
- **Services** provide synchronous, request-response communication

These concepts form the foundation of ROS 2 and are essential for building any robotic system. Understanding these patterns is crucial for effective robot software development.

## Next Steps

In the next chapter, we'll explore how to connect Python agents to ROS controllers and develop AI algorithms that can control robot behavior through the ROS 2 middleware.