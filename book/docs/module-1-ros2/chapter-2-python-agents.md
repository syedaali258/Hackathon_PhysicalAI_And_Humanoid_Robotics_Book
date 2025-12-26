---
title: Chapter 2 - Python Agents with rclpy and ROS Controllers
sidebar_label: Chapter 2 - Python Agents
description: Connecting Python agents to ROS controllers and developing AI algorithms for robot behavior
---

# Python Agents with rclpy and ROS Controllers

## Introduction

In this chapter, we'll explore how to connect Python agents to ROS controllers and develop AI algorithms that can control robot behavior through the ROS 2 middleware. Python is one of the most popular languages for AI and robotics due to its simplicity and the rich ecosystem of libraries available.

## Understanding rclpy

`rclpy` is the Python client library for ROS 2. It provides the standard interface for Python programs to interact with ROS 2. With `rclpy`, you can create nodes, publish and subscribe to topics, provide and use services, and more.

### Installing rclpy

If you're working in a Python virtual environment, you'll need to source the ROS 2 installation before using rclpy:

```bash
source /opt/ros/humble/setup.bash  # Replace 'humble' with your ROS 2 distribution
source install/local_setup.bash   # If you built ROS 2 from source
```

## Creating Your First Python Agent Node

Let's start with a simple Python agent that acts as a controller for a mobile robot:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Create publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Create subscriber for laser scan data
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Create a timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        # Robot state
        self.obstacle_detected = False
        self.obstacle_distance = float('inf')

        self.get_logger().info('Robot controller node initialized')

    def scan_callback(self, msg):
        """Process laser scan data to detect obstacles."""
        # Check for obstacles in front of the robot
        min_distance = min(msg.ranges)
        self.obstacle_distance = min_distance
        self.obstacle_detected = min_distance < 1.0  # Obstacle within 1 meter

        if self.obstacle_detected:
            self.get_logger().info(f'Obstacle detected at {min_distance:.2f}m')

    def control_loop(self):
        """Main control loop for the robot."""
        cmd_vel = Twist()

        if self.obstacle_detected:
            # Stop the robot if an obstacle is detected
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.get_logger().info('Obstacle detected - stopping robot')
        else:
            # Move forward if no obstacle
            cmd_vel.linear.x = 0.5  # Move forward at 0.5 m/s
            cmd_vel.angular.z = 0.0  # No rotation
            self.get_logger().info('Moving forward')

        self.cmd_vel_publisher.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()

    try:
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        pass
    finally:
        robot_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced Python Agent: AI-Based Navigation

Now let's create a more sophisticated Python agent that uses AI principles for navigation:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
from collections import deque
import math

class AINavigationAgent(Node):
    def __init__(self):
        super().__init__('ai_navigation_agent')

        # Publishers and subscribers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Control timer
        self.timer = self.create_timer(0.1, self.control_loop)

        # Robot state
        self.position = [0.0, 0.0]  # x, y
        self.orientation = 0.0      # theta
        self.scan_data = None
        self.goal = [5.0, 5.0]      # Target position
        self.path_history = deque(maxlen=100)  # Store last 100 positions

        # AI parameters
        self.kp_linear = 0.5  # Proportional gain for linear velocity
        self.kp_angular = 1.0 # Proportional gain for angular velocity
        self.safe_distance = 0.8  # Minimum distance to obstacles

        self.get_logger().info('AI Navigation Agent initialized')

    def odom_callback(self, msg):
        """Update robot position from odometry."""
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y

        # Convert quaternion to euler
        orientation_q = msg.pose.pose.orientation
        self.orientation = math.atan2(
            2.0 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y),
            1.0 - 2.0 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        )

        # Store position in history
        self.path_history.append((self.position[0], self.position[1]))

    def scan_callback(self, msg):
        """Process laser scan data."""
        self.scan_data = np.array(msg.ranges)
        # Replace invalid ranges with maximum range
        self.scan_data[np.isnan(self.scan_data)] = msg.range_max
        self.scan_data[np.isinf(self.scan_data)] = msg.range_max

    def compute_control(self):
        """Compute control commands using AI-based approach."""
        if self.scan_data is None:
            return Twist()  # Return zero velocity if no scan data

        # Calculate distance to goal
        dx = self.goal[0] - self.position[0]
        dy = self.goal[1] - self.position[1]
        distance_to_goal = math.sqrt(dx*dx + dy*dy)

        # Calculate desired heading to goal
        desired_heading = math.atan2(dy, dx)

        # Calculate heading error
        heading_error = desired_heading - self.orientation
        # Normalize angle to [-pi, pi]
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi

        # Simple proportional controller
        angular_velocity = self.kp_angular * heading_error

        # Check for obstacles in front
        front_scan_start = len(self.scan_data) // 2 - 10
        front_scan_end = len(self.scan_data) // 2 + 10
        front_distances = self.scan_data[front_scan_start:front_scan_end]

        min_front_distance = np.min(front_distances) if len(front_distances) > 0 else float('inf')

        # Adjust linear velocity based on obstacles
        if min_front_distance < self.safe_distance:
            linear_velocity = 0.0  # Stop if too close to obstacle
        else:
            # Reduce speed as we get closer to obstacles
            safety_factor = min(min_front_distance / self.safe_distance, 1.0)
            linear_velocity = self.kp_linear * distance_to_goal * safety_factor
            linear_velocity = min(linear_velocity, 0.5)  # Limit max speed

        cmd_vel = Twist()
        cmd_vel.linear.x = linear_velocity
        cmd_vel.angular.z = angular_velocity

        return cmd_vel

    def control_loop(self):
        """Main control loop."""
        cmd_vel = self.compute_control()
        self.cmd_vel_publisher.publish(cmd_vel)

        # Log current state
        distance_to_goal = math.sqrt(
            (self.goal[0] - self.position[0])**2 +
            (self.goal[1] - self.position[1])**2
        )

        self.get_logger().info(
            f'Position: ({self.position[0]:.2f}, {self.position[1]:.2f}), ' +
            f'Distance to goal: {distance_to_goal:.2f}, ' +
            f'Velocity: ({cmd_vel.linear.x:.2f}, {cmd_vel.angular.z:.2f})'
        )

def main(args=None):
    rclpy.init(args=args)
    ai_agent = AINavigationAgent()

    try:
        rclpy.spin(ai_agent)
    except KeyboardInterrupt:
        pass
    finally:
        ai_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Working with ROS Controllers

ROS controllers are specialized nodes that interface with hardware or simulated hardware. Here's how to work with common controller types:

### Joint State Controller

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.05, self.publish_joint_states)  # 20 Hz

        # Initialize joint names and positions
        self.joint_names = ['wheel_left_joint', 'wheel_right_joint', 'arm_joint']
        self.joint_positions = [0.0, 0.0, 0.0]  # radians or meters
        self.joint_velocities = [0.0, 0.0, 0.0]
        self.joint_efforts = [0.0, 0.0, 0.0]

    def publish_joint_states(self):
        """Publish joint state messages."""
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.joint_positions
        msg.velocity = self.joint_velocities
        msg.effort = self.joint_efforts

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    joint_publisher = JointStatePublisher()

    try:
        rclpy.spin(joint_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        joint_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Connecting to Robot Controllers

To connect your Python agent to actual robot controllers, you'll typically:

1. **Subscribe to sensor topics** (e.g., `/joint_states`, `/scan`, `/odom`)
2. **Publish to actuator topics** (e.g., `/cmd_vel`, `/joint_commands`)
3. **Use services for specific actions** (e.g., `/set_parameters`, `/get_state`)

### Example: Robot Arm Controller

```python
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
import time

class RobotArmController(Node):
    def __init__(self):
        super().__init__('robot_arm_controller')

        # Publisher for joint trajectory commands
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

        # Subscriber for current state
        self.state_subscriber = self.create_subscription(
            JointTrajectoryControllerState,
            '/arm_controller/state',
            self.state_callback,
            10
        )

        self.current_positions = None
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

    def state_callback(self, msg):
        """Update current joint positions."""
        self.current_positions = list(msg.actual.positions)
        self.get_logger().info(f'Current positions: {self.current_positions}')

    def move_to_position(self, target_positions, duration=5.0):
        """Move robot arm to target joint positions."""
        if len(target_positions) != len(self.joint_names):
            self.get_logger().error('Target positions do not match joint names')
            return

        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.velocities = [0.0] * len(target_positions)
        point.accelerations = [0.0] * len(target_positions)
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)

        trajectory_msg.points = [point]

        self.trajectory_publisher.publish(trajectory_msg)
        self.get_logger().info(f'Moving to positions: {target_positions}')

def main(args=None):
    rclpy.init(args=args)
    arm_controller = RobotArmController()

    # Allow some time for connections
    time.sleep(1.0)

    # Move to a sample position
    arm_controller.move_to_position([0.5, 0.3, -0.2, 0.0])

    try:
        rclpy.spin(arm_controller)
    except KeyboardInterrupt:
        pass
    finally:
        arm_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices for Python Agents

1. **Error Handling**: Always include proper error handling for ROS communication
2. **Resource Management**: Clean up resources properly when shutting down
3. **Threading**: Be aware of ROS's threading model and use appropriate synchronization
4. **Parameter Management**: Use ROS parameters for configurable values
5. **Logging**: Use ROS logging facilities for debugging and monitoring

## Summary

In this chapter, we've covered:

- Creating Python agents using rclpy
- Connecting to various ROS controllers
- Implementing AI-based control algorithms
- Working with different message types and controllers
- Best practices for robust Python agents

These concepts form the foundation for creating sophisticated AI agents that can interact with robotic systems through ROS 2.

## Next Steps

In the next chapter, we'll explore how to read and modify URDF files to understand the physical structure of humanoid robots and customize robot models.