import React, { useState, useEffect, useRef } from 'react';
import styles from './NavigationPlanner.module.css';

/**
 * NavigationPlanner Component
 * Interactive visualization for Nav2 navigation and path planning concepts
 */
const NavigationPlanner = () => {
  const [activeTab, setActiveTab] = useState('overview');
  const [navigationState, setNavigationState] = useState({
    status: 'idle',
    currentGoal: null,
    path: [],
    robotPosition: { x: 0, y: 0 },
    map: null,
    recoveryStatus: 'normal'
  });
  const [isPlanning, setIsPlanning] = useState(false);
  const canvasRef = useRef(null);

  // Mock navigation planning function
  const planPath = (start, goal) => {
    // Simulate path planning algorithm
    const path = [];
    const steps = 20;
    for (let i = 0; i <= steps; i++) {
      const ratio = i / steps;
      path.push({
        x: start.x + (goal.x - start.x) * ratio,
        y: start.y + (goal.y - start.y) * ratio
      });
    }
    return path;
  };

  const executeNavigation = async (goal) => {
    setIsPlanning(true);

    // Simulate navigation execution
    const start = { x: 0, y: 0 };
    const plannedPath = planPath(start, goal);

    setNavigationState(prev => ({
      ...prev,
      status: 'planning',
      currentGoal: goal,
      path: plannedPath
    }));

    // Simulate navigation execution
    for (let i = 0; i < plannedPath.length; i++) {
      setNavigationState(prev => ({
        ...prev,
        robotPosition: plannedPath[i],
        status: 'executing'
      }));

      // Simulate processing delay
      await new Promise(resolve => setTimeout(resolve, 100));
    }

    setNavigationState(prev => ({
      ...prev,
      status: 'completed',
      robotPosition: goal
    }));

    setIsPlanning(false);
  };

  const handleCanvasClick = (event) => {
    if (navigationState.status !== 'idle' && navigationState.status !== 'completed') {
      return; // Don't accept new goals during navigation
    }

    const canvas = canvasRef.current;
    const rect = canvas.getBoundingClientRect();
    const x = event.clientX - rect.left;
    const y = event.clientY - rect.top;

    // Convert to navigation coordinates (inverted Y for canvas)
    const navX = (x - canvas.width / 2) / 20; // Scale down for navigation coords
    const navY = -(y - canvas.height / 2) / 20; // Invert Y and scale down

    executeNavigation({ x: navX, y: navY });
  };

  // Render navigation visualization
  const renderVisualization = () => (
    <div className={styles.visualization}>
      <h3>Nav2 Navigation Visualization</h3>

      <div className={styles.controls}>
        <button
          className={styles.controlButton}
          onClick={() => {
            setNavigationState({
              status: 'idle',
              currentGoal: null,
              path: [],
              robotPosition: { x: 0, y: 0 },
              map: null,
              recoveryStatus: 'normal'
            });
          }}
          disabled={navigationState.status === 'executing'}
        >
          Reset Navigation
        </button>

        <div className={styles.statusIndicator}>
          <span className={styles.statusLabel}>Status:</span>
          <span className={`${styles.statusValue} ${styles[navigationState.status]}`}>
            {navigationState.status.charAt(0).toUpperCase() + navigationState.status.slice(1)}
          </span>
        </div>
      </div>

      <div className={styles.canvasContainer}>
        <canvas
          ref={canvasRef}
          width={600}
          height={400}
          className={styles.navigationCanvas}
          onClick={handleCanvasClick}
        />

        <div className={styles.instructions}>
          Click on the map to set a navigation goal for the humanoid robot
        </div>
      </div>

      <div className={styles.pathInfo}>
        <div className={styles.infoItem}>
          <span className={styles.infoLabel}>Current Position:</span>
          <span className={styles.infoValue}>
            ({navigationState.robotPosition.x.toFixed(2)}, {navigationState.robotPosition.y.toFixed(2)})
          </span>
        </div>

        {navigationState.currentGoal && (
          <div className={styles.infoItem}>
            <span className={styles.infoLabel}>Target Goal:</span>
            <span className={styles.infoValue}>
              ({navigationState.currentGoal.x.toFixed(2)}, {navigationState.currentGoal.y.toFixed(2)})
            </span>
          </div>
        )}

        <div className={styles.infoItem}>
          <span className={styles.infoLabel}>Path Length:</span>
          <span className={styles.infoValue}>{navigationState.path.length} waypoints</span>
        </div>
      </div>
    </div>
  );

  const renderArchitecture = () => (
    <div className={styles.architecture}>
      <h3>Nav2 Architecture for Humanoid Navigation</h3>

      <div className={styles.pipeline}>
        <div className={styles.pipelineStage}>
          <div className={styles.stageHeader}>Goal Input</div>
          <div className={styles.stageContent}>
            <div className={styles.component}>Goal Pose</div>
            <div className={styles.component}>Transforms</div>
          </div>
        </div>

        <div className={styles.arrow}>→</div>

        <div className={styles.pipelineStage}>
          <div className={styles.stageHeader}>Global Planning</div>
          <div className={styles.stageContent}>
            <div className={styles.component}>NavFn/A* Planner</div>
            <div className={styles.component}>Path Optimization</div>
            <div className={styles.component}>Collision Checking</div>
          </div>
        </div>

        <div className={styles.arrow}>→</div>

        <div className={styles.pipelineStage}>
          <div className={styles.stageHeader}>Local Planning</div>
          <div className={styles.stageContent}>
            <div className={styles.component}>MPPIC Controller</div>
            <div className={styles.component}>Obstacle Avoidance</div>
            <div className={styles.component}>Kinematic Constraints</div>
          </div>
        </div>

        <div className={styles.arrow}>→</div>

        <div className={styles.pipelineStage}>
          <div className={styles.stageHeader}>Execution</div>
          <div className={styles.stageContent}>
            <div className={styles.component}>Velocity Control</div>
            <div className={styles.component}>Feedback Control</div>
            <div className={styles.component}>Recovery Behaviors</div>
          </div>
        </div>
      </div>

      <div className={styles.behaviorTree}>
        <h4>Behavior Tree for Humanoid Navigation</h4>
        <div className={styles.treeVisualization}>
          <div className={styles.treeNode}>NavigateWithReplanning</div>
          <div className={styles.treeBranch}>
            <div className={styles.treeNode}>RateController (1Hz)</div>
            <div className={styles.treeNode}>RecoveryNode (6 retries)</div>
            <div className={styles.treeBranch}>
              <div className={styles.treeNode}>ComputeAndExecutePath</div>
              <div className={styles.treeBranch}>
                <div className={styles.treeNode}>GoalReached</div>
                <div className={styles.treeNode}>ComputePathToPose</div>
                <div className={styles.treeNode}>SmoothPath</div>
                <div className={styles.treeNode}>FollowPath</div>
              </div>
            </div>
            <div className={styles.treeBranch}>
              <div className={styles.treeNode}>MoveBaseFallback</div>
              <div className={styles.treeBranch}>
                <div className={styles.treeNode}>GoalUpdated</div>
                <div className={styles.treeNode}>ClearGlobalCostmap</div>
                <div className={styles.treeNode}>RecoveryNode (2 retries)</div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );

  const renderHumanoidConsiderations = () => (
    <div className={styles.humanoidConsiderations}>
      <h3>Humanoid-Specific Navigation Considerations</h3>

      <div className={styles.considerationsGrid}>
        <div className={styles.considerationCard}>
          <h4>Kinematic Constraints</h4>
          <p>Humanoid robots have different movement capabilities than wheeled robots, requiring path planning that accounts for bipedal locomotion patterns and balance requirements.</p>
          <ul className={styles.constraintsList}>
            <li>Maximum step size: 0.3m</li>
            <li>Minimum turning radius: 0.4m</li>
            <li>Max incline angle: 15°</li>
            <li>Balance maintenance requirements</li>
          </ul>
        </div>

        <div className={styles.considerationCard}>
          <h4>Social Navigation</h4>
          <p>Humanoid robots must navigate in human environments respecting social norms and personal space.</p>
          <ul className={styles.constraintsList}>
            <li>Personal space radius: 1.0m</li>
            <li>Social lane width: 1.2m</li>
            <li>Human interaction protocols</li>
            <li>Crowd navigation strategies</li>
          </ul>
        </div>

        <div className={styles.considerationCard}>
          <h4>Stability Requirements</h4>
          <p>Navigation paths must ensure the humanoid robot maintains stability throughout the movement.</p>
          <ul className={styles.constraintsList}>
            <li>ZMP (Zero Moment Point) constraints</li>
            <li>Footstep planning</li>
            <li>Center of mass management</li>
            <li>Dynamical balance control</li>
          </ul>
        </div>
      </div>

      <div className={styles.recoveryBehaviors}>
        <h4>Humanoid Recovery Behaviors</h4>
        <div className={styles.behaviorsGrid}>
          <div className={styles.behavior}>
            <div className={styles.behaviorIcon}>🔄</div>
            <div className={styles.behaviorContent}>
              <h5>Spin Recovery</h5>
              <p>Slow, controlled rotation to reorient when stuck (max 10s duration)</p>
            </div>
          </div>

          <div className={styles.behavior}>
            <div className={styles.behaviorIcon}>🔙</div>
            <div className={styles.behaviorContent}>
              <h5>Backup Recovery</h5>
              <p>Short backward movement for stability (0.2m distance)</p>
            </div>
          </div>

          <div className={styles.behavior}>
            <div className={styles.behaviorIcon}>⏸️</div>
            <div className={styles.behaviorContent}>
              <h5>Wait Recovery</h5>
              <p>Pause to allow dynamic obstacles to clear path</p>
            </div>
          </div>
        </div>
      </div>
    </div>
  );

  const renderContent = () => {
    switch (activeTab) {
      case 'visualization':
        return renderVisualization();
      case 'architecture':
        return renderArchitecture();
      case 'humanoid':
        return renderHumanoidConsiderations();
      default:
        return (
          <div className={styles.overview}>
            <h3>Navigation and Path Planning with Nav2</h3>

            <div className={styles.introText}>
              <p>
                Navigation2 (Nav2) is the standard ROS 2 navigation framework that provides comprehensive
                path planning and execution capabilities for mobile robots. For humanoid robots, Nav2
                requires special configuration to account for bipedal locomotion, balance requirements,
                and social navigation considerations.
              </p>

              <div className={styles.keyFeatures}>
                <div className={styles.feature}>
                  <h4>Global Planning</h4>
                  <p>Computes optimal paths from start to goal using algorithms like A* or Dijkstra</p>
                </div>

                <div className={styles.feature}>
                  <h4>Local Planning</h4>
                  <p>Handles real-time obstacle avoidance and path following with kinematic constraints</p>
                </div>

                <div className={styles.feature}>
                  <h4>Behavior Trees</h4>
                  <p>Modular architecture for complex navigation behaviors and recovery strategies</p>
                </div>

                <div className={styles.feature}>
                  <h4>Recovery Behaviors</h4>
                  <p>Automatic strategies for handling navigation failures and stuck situations</p>
                </div>
              </div>
            </div>
          </div>
        );
    }
  };

  return (
    <div className={styles.navigationPlanner}>
      <div className={styles.header}>
        <h2>Navigation Planner</h2>
        <p>Interactive demonstration of Nav2 navigation and path planning for humanoid robots</p>
      </div>

      <div className={styles.tabs}>
        <button
          className={`${styles.tabButton} ${activeTab === 'overview' ? styles.active : ''}`}
          onClick={() => setActiveTab('overview')}
        >
          Overview
        </button>
        <button
          className={`${styles.tabButton} ${activeTab === 'visualization' ? styles.active : ''}`}
          onClick={() => setActiveTab('visualization')}
        >
          Interactive Planner
        </button>
        <button
          className={`${styles.tabButton} ${activeTab === 'architecture' ? styles.active : ''}`}
          onClick={() => setActiveTab('architecture')}
        >
          Architecture
        </button>
        <button
          className={`${styles.tabButton} ${activeTab === 'humanoid' ? styles.active : ''}`}
          onClick={() => setActiveTab('humanoid')}
        >
          Humanoid Considerations
        </button>
      </div>

      <div className={styles.contentArea}>
        {renderContent()}
      </div>

      <div className={styles.explanation}>
        <h3>How Nav2 Enables Humanoid Robot Autonomy</h3>
        <div className={styles.benefitsGrid}>
          <div className={styles.benefit}>
            <h4>Autonomous Navigation</h4>
            <p>Humanoid robots can navigate complex environments without human intervention using pre-built maps and path planning algorithms.</p>
          </div>

          <div className={styles.benefit}>
            <h4>Socially Aware Movement</h4>
            <p>Navigation behaviors respect human social spaces and movement patterns, enabling safe coexistence.</p>
          </div>

          <div className={styles.benefit}>
            <h4>Robust Path Execution</h4>
            <p>Advanced recovery behaviors handle unexpected obstacles and navigation failures gracefully.</p>
          </div>

          <div className={styles.benefit}>
            <h4>Humanoid-Specific Optimization</h4>
            <p>Parameters tuned specifically for bipedal locomotion and stability requirements.</p>
          </div>
        </div>
      </div>
    </div>
  );
};

export default NavigationPlanner;