import React, { useState, useEffect } from 'react';
import styles from './ROSController.module.css';

const ROSController = () => {
  const [connectionStatus, setConnectionStatus] = useState('disconnected');
  const [robotState, setRobotState] = useState({
    position: { x: 0, y: 0, theta: 0 },
    velocity: { linear: 0, angular: 0 },
    joints: { left_wheel: 0, right_wheel: 0, arm_joint: 0 }
  });
  const [controlMode, setControlMode] = useState('manual'); // manual, autonomous
  const [command, setCommand] = useState({ linear: 0, angular: 0 });

  // Simulate connection to ROS
  useEffect(() => {
    const connectToROS = () => {
      // In a real implementation, this would connect to an actual ROS bridge
      setConnectionStatus('connected');

      // Simulate receiving robot state updates
      const stateInterval = setInterval(() => {
        setRobotState(prev => ({
          ...prev,
          position: {
            x: prev.position.x + prev.velocity.linear * Math.cos(prev.position.theta) * 0.1,
            y: prev.position.y + prev.velocity.linear * Math.sin(prev.position.theta) * 0.1,
            theta: prev.position.theta + prev.velocity.angular * 0.1
          },
          joints: {
            left_wheel: prev.joints.left_wheel + 0.1,
            right_wheel: prev.joints.right_wheel + 0.1,
            arm_joint: Math.sin(Date.now() / 1000) // Oscillating arm
          }
        }));
      }, 100);

      return () => clearInterval(stateInterval);
    };

    // Simulate connection delay
    const connectTimeout = setTimeout(connectToROS, 1000);

    return () => clearTimeout(connectTimeout);
  }, []);

  const sendCommand = (cmd) => {
    // In a real implementation, this would send the command to ROS
    setRobotState(prev => ({
      ...prev,
      velocity: cmd
    }));
    setCommand(cmd);
  };

  const handleMove = (direction) => {
    let linear = 0;
    let angular = 0;

    switch (direction) {
      case 'forward':
        linear = 0.5;
        break;
      case 'backward':
        linear = -0.5;
        break;
      case 'left':
        angular = 0.5;
        break;
      case 'right':
        angular = -0.5;
        break;
      case 'stop':
        linear = 0;
        angular = 0;
        break;
      default:
        break;
    }

    sendCommand({ linear, angular });
  };

  const setAutonomousMode = () => {
    setControlMode('autonomous');
    // In a real implementation, this would engage autonomous navigation
    sendCommand({ linear: 0.3, angular: 0.1 }); // Simple autonomous behavior
  };

  const setManualMode = () => {
    setControlMode('manual');
    sendCommand({ linear: 0, angular: 0 });
  };

  return (
    <div className={styles.container}>
      <div className={styles.header}>
        <h3>ROS Controller Simulation</h3>
        <div className={`${styles.statusIndicator} ${styles[connectionStatus]}`}>
          {connectionStatus === 'connected' ? 'Connected' : 'Disconnected'}
        </div>
      </div>

      <div className={styles.controlPanel}>
        <div className={styles.modeSelector}>
          <button
            className={`${styles.modeButton} ${controlMode === 'manual' ? styles.active : ''}`}
            onClick={setManualMode}
          >
            Manual Control
          </button>
          <button
            className={`${styles.modeButton} ${controlMode === 'autonomous' ? styles.active : ''}`}
            onClick={setAutonomousMode}
          >
            Autonomous
          </button>
        </div>

        {controlMode === 'manual' && (
          <div className={styles.manualControls}>
            <div className={styles.dpad}>
              <button
                className={styles.dpadButton}
                onClick={() => handleMove('forward')}
                title="Move Forward"
              >
                ↑
              </button>
              <div className={styles.dpadMiddle}>
                <button
                  className={styles.dpadButton}
                  onClick={() => handleMove('left')}
                  title="Turn Left"
                >
                  ←
                </button>
                <button
                  className={`${styles.dpadButton} ${styles.center}`}
                  onClick={() => handleMove('stop')}
                  title="Stop"
                >
                  ■
                </button>
                <button
                  className={styles.dpadButton}
                  onClick={() => handleMove('right')}
                  title="Turn Right"
                >
                  →
                </button>
              </div>
              <button
                className={styles.dpadButton}
                onClick={() => handleMove('backward')}
                title="Move Backward"
              >
                ↓
              </button>
            </div>

            <div className={styles.sliderControls}>
              <div className={styles.sliderGroup}>
                <label>Linear Velocity: {command.linear.toFixed(2)}</label>
                <input
                  type="range"
                  min="-1"
                  max="1"
                  step="0.1"
                  value={command.linear}
                  onChange={(e) => sendCommand({...command, linear: parseFloat(e.target.value)})}
                  className={styles.slider}
                />
              </div>
              <div className={styles.sliderGroup}>
                <label>Angular Velocity: {command.angular.toFixed(2)}</label>
                <input
                  type="range"
                  min="-1"
                  max="1"
                  step="0.1"
                  value={command.angular}
                  onChange={(e) => sendCommand({...command, angular: parseFloat(e.target.value)})}
                  className={styles.slider}
                />
              </div>
            </div>
          </div>
        )}

        {controlMode === 'autonomous' && (
          <div className={styles.autonomousInfo}>
            <p>Autonomous mode engaged. The robot is navigating using AI algorithms.</p>
            <div className={styles.behaviorSelector}>
              <label>Navigation Behavior:</label>
              <select className={styles.select}>
                <option>Obstacle Avoidance</option>
                <option>Goal Navigation</option>
                <option>Patrol Route</option>
                <option>Follow Path</option>
              </select>
            </div>
          </div>
        )}
      </div>

      <div className={styles.robotState}>
        <h4>Robot State</h4>
        <div className={styles.stateGrid}>
          <div className={styles.stateItem}>
            <label>Position:</label>
            <span>X: {robotState.position.x.toFixed(2)}m, Y: {robotState.position.y.toFixed(2)}m</span>
          </div>
          <div className={styles.stateItem}>
            <label>Orientation:</label>
            <span>θ: {robotState.position.theta.toFixed(2)} rad</span>
          </div>
          <div className={styles.stateItem}>
            <label>Velocity:</label>
            <span>Linear: {robotState.velocity.linear.toFixed(2)} m/s, Angular: {robotState.velocity.angular.toFixed(2)} rad/s</span>
          </div>
          <div className={styles.stateItem}>
            <label>Joint Positions:</label>
            <span>L: {robotState.joints.left_wheel.toFixed(2)}, R: {robotState.joints.right_wheel.toFixed(2)}, A: {robotState.joints.arm_joint.toFixed(2)}</span>
          </div>
        </div>
      </div>

      <div className={styles.commandHistory}>
        <h4>Command History</h4>
        <div className={styles.historyList}>
          <div className={styles.historyItem}>
            <span>Current Command:</span>
            <span>Linear: {command.linear.toFixed(2)}, Angular: {command.angular.toFixed(2)}</span>
          </div>
        </div>
      </div>
    </div>
  );
};

export default ROSController;