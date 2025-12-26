import React, { useState } from 'react';
import styles from './PythonAgentExample.module.css';

const PythonAgentExample = () => {
  const [activeExample, setActiveExample] = useState('basic');
  const [codeOutput, setCodeOutput] = useState('');

  const examples = {
    basic: {
      title: "Basic Robot Controller",
      code: `import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

    def control_loop(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.5  # Move forward
        cmd_vel.angular.z = 0.0  # No rotation
        self.publisher.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()`,
      description: "A simple controller that makes the robot move forward continuously."
    },
    ai: {
      title: "AI Navigation Agent",
      code: `import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class AINavigationAgent(Node):
    def __init__(self):
        super().__init__('ai_navigation_agent')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_subscriber = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.obstacle_detected = False

    def scan_callback(self, msg):
        min_distance = min(msg.ranges) if msg.ranges else float('inf')
        self.obstacle_detected = min_distance < 1.0

    def control_loop(self):
        cmd_vel = Twist()
        if self.obstacle_detected:
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.5  # Turn to avoid obstacle
        else:
            cmd_vel.linear.x = 0.5  # Move forward
            cmd_vel.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd_vel)`,
      description: "An AI agent that uses sensor data to navigate and avoid obstacles."
    },
    arm: {
      title: "Robot Arm Controller",
      code: `import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class RobotArmController(Node):
    def __init__(self):
        super().__init__('robot_arm_controller')
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

    def move_to_position(self, positions):
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3']

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 3  # 3 seconds to reach position

        trajectory_msg.points = [point]
        self.publisher.publish(trajectory_msg)

def main(args=None):
    rclpy.init(args=args)
    controller = RobotArmController()
    controller.move_to_position([0.5, 0.3, -0.2])
    controller.destroy_node()
    rclpy.shutdown()`,
      description: "A controller for managing robot arm joint positions."
    }
  };

  const runExample = () => {
    setCodeOutput('Running example... Output will appear here in a real implementation.');
  };

  const copyToClipboard = () => {
    navigator.clipboard.writeText(examples[activeExample].code);
  };

  return (
    <div className={styles.container}>
      <h3>Python Agent Examples</h3>

      <div className={styles.tabs}>
        {Object.entries(examples).map(([key, example]) => (
          <button
            key={key}
            className={`${styles.tabButton} ${activeExample === key ? styles.activeTab : ''}`}
            onClick={() => setActiveExample(key)}
          >
            {example.title}
          </button>
        ))}
      </div>

      <div className={styles.content}>
        <div className={styles.description}>
          <p>{examples[activeExample].description}</p>
        </div>

        <div className={styles.codeSection}>
          <div className={styles.codeHeader}>
            <span className={styles.fileName}>example.py</span>
            <div className={styles.codeActions}>
              <button className={styles.copyButton} onClick={copyToClipboard}>
                Copy
              </button>
              <button className={styles.runButton} onClick={runExample}>
                Run
              </button>
            </div>
          </div>
          <pre className={styles.codeBlock}>
            <code>{examples[activeExample].code}</code>
          </pre>
        </div>

        {codeOutput && (
          <div className={styles.outputSection}>
            <h4>Output:</h4>
            <pre className={styles.outputBlock}>{codeOutput}</pre>
          </div>
        )}
      </div>

      <div className={styles.explanation}>
        <h4>Key Concepts:</h4>
        <ul>
          <li><strong>rclpy</strong>: The Python client library for ROS 2</li>
          <li><strong>Nodes</strong>: Individual processes that perform computation</li>
          <li><strong>Publishers/Subscribers</strong>: Communication through topics</li>
          <li><strong>Timers</strong>: For creating control loops</li>
          <li><strong>Message Types</strong>: Standardized data structures for communication</li>
        </ul>
      </div>
    </div>
  );
};

export default PythonAgentExample;