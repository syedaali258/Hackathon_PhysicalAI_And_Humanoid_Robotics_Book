import React, { useState, useEffect } from 'react';
import styles from './Nav2Integration.module.css';

/**
 * Nav2Integration Component
 * Visualization and explanation of Nav2 navigation stack integration with Isaac Sim
 */
const Nav2Integration = () => {
  const [activeTab, setActiveTab] = useState('overview');
  const [navState, setNavState] = useState({
    status: 'idle',
    currentGoal: null,
    path: [],
    robotPosition: { x: 0, y: 0 },
    costmap: null,
    recoveryStatus: 'normal'
  });
  const [isSimulating, setIsSimulating] = useState(false);

  // Mock Nav2 costmap generation
  const generateCostmap = (width = 20, height = 20) => {
    const costmap = [];
    for (let y = 0; y < height; y++) {
      const row = [];
      for (let x = 0; x < width; x++) {
        // Generate cost values: 0 (free) to 254 (occupied), 255 (unknown)
        let cost = 0;
        if (x < 2 || x > width - 3 || y < 2 || y > height - 3) {
          // Border - higher cost
          cost = 50;
        } else if (x > 8 && x < 12 && y > 8 && y < 12) {
          // Obstacle in center
          cost = 254;
        } else {
          // Random free space with some obstacles
          cost = Math.random() > 0.8 ? 200 : Math.floor(Math.random() * 50);
        }
        row.push(cost);
      }
      costmap.push(row);
    }
    return costmap;
  };

  const planPath = (start, goal, costmap) => {
    // Simple path planning algorithm (mock implementation)
    const path = [];
    const steps = 10;
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
    setIsSimulating(true);

    // Generate mock costmap
    const costmap = generateCostmap();

    // Plan path
    const start = { x: 2, y: 2 };
    const path = planPath(start, goal, costmap);

    setNavState({
      status: 'executing',
      currentGoal: goal,
      path: path,
      robotPosition: start,
      costmap: costmap,
      recoveryStatus: 'normal'
    });

    // Simulate navigation execution
    for (let i = 0; i < path.length; i++) {
      setNavState(prev => ({
        ...prev,
        robotPosition: path[i],
        status: 'executing'
      }));

      // Simulate processing delay
      await new Promise(resolve => setTimeout(resolve, 200));
    }

    setNavState(prev => ({
      ...prev,
      status: 'completed',
      robotPosition: goal
    }));

    setIsSimulating(false);
  };

  const triggerRecovery = (recoveryType) => {
    setNavState(prev => ({
      ...prev,
      recoveryStatus: recoveryType,
      status: 'recovery'
    }));

    // Simulate recovery behavior
    setTimeout(() => {
      setNavState(prev => ({
        ...prev,
        recoveryStatus: 'normal',
        status: 'ready'
      }));
    }, 2000);
  };

  const renderOverview = () => (
    <div className={styles.overview}>
      <h3>Nav2 Navigation Stack Overview</h3>

      <div className={styles.architecture}>
        <h4>Nav2 Architecture Components</h4>
        <div className={styles.componentGrid}>
          <div className={styles.componentCard}>
            <h5>Navigation Server</h5>
            <p>Central coordinator managing all navigation components and state</p>
            <div className={styles.componentDetails}>
              <div className={styles.detailItem}>
                <span className={styles.detailLabel}>Lifecycle Manager</span>
                <span className={styles.detailValue}>Controls component states</span>
              </div>
              <div className={styles.detailItem}>
                <span className={styles.detailLabel}>Action Server</span>
                <span className={styles.detailValue}>Handles navigation goals</span>
              </div>
            </div>
          </div>

          <div className={styles.componentCard}>
            <h5>Global Planner</h5>
            <p>Generates optimal path from start to goal using global costmap</p>
            <div className={styles.componentDetails}>
              <div className={styles.detailItem}>
                <span className={styles.detailLabel}>Algorithm</span>
                <span className={styles.detailValue}>A*, NavFn, or custom</span>
              </div>
              <div className={styles.detailItem}>
                <span className={styles.detailLabel}>Input</span>
                <span className={styles.detailValue}>Global costmap + goal</span>
              </div>
            </div>
          </div>

          <div className={styles.componentCard}>
            <h5>Local Planner</h5>
            <p>Executes path following and obstacle avoidance in local window</p>
            <div className={styles.componentDetails}>
              <div className={styles.detailItem}>
                <span className={styles.detailLabel}>Algorithm</span>
                <span className={styles.detailValue}>MPPIC, DWA, or TBV</span>
              </div>
              <div className={styles.detailItem}>
                <span className={styles.detailLabel}>Input</span>
                <span className={styles.detailValue}>Local costmap + path</span>
              </div>
            </div>
          </div>

          <div className={styles.componentCard}>
            <h5>Costmap Layers</h5>
            <p>Manages obstacle detection and collision avoidance</p>
            <div className={styles.componentDetails}>
              <div className={styles.detailItem}>
                <span className={styles.detailLabel}>Types</span>
                <span className={styles.detailValue}>Static, Obstacle, Inflation</span>
              </div>
              <div className={styles.detailItem}>
                <span className={styles.detailLabel}>Function</span>
                <span className={styles.detailValue}>Collision detection</span>
              </div>
            </div>
          </div>
        </div>
      </div>

      <div className={styles.behaviorTree}>
        <h4>Behavior Tree for Navigation</h4>
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

  const renderVisualization = () => (
    <div className={styles.visualization}>
      <h3>Nav2 Navigation Visualization</h3>

      <div className={styles.simulationControls}>
        <button
          className={styles.controlButton}
          onClick={() => executeNavigation({ x: 15, y: 15 })}
          disabled={isSimulating}
        >
          {isSimulating ? 'Navigating...' : 'Start Navigation'}
        </button>

        <button
          className={styles.controlButton}
          onClick={() => setNavState({
            status: 'idle',
            currentGoal: null,
            path: [],
            robotPosition: { x: 0, y: 0 },
            costmap: null,
            recoveryStatus: 'normal'
          })}
        >
          Reset
        </button>
      </div>

      <div className={styles.statusPanel}>
        <div className={styles.statusItem}>
          <span className={styles.statusLabel}>Status:</span>
          <span className={styles[`statusValue--${navState.status}`]}>
            {navState.status.charAt(0).toUpperCase() + navState.status.slice(1)}
          </span>
        </div>
        <div className={styles.statusItem}>
          <span className={styles.statusLabel}>Recovery:</span>
          <span className={styles.statusValue}>
            {navState.recoveryStatus}
          </span>
        </div>
        <div className={styles.statusItem}>
          <span className={styles.statusLabel}>Position:</span>
          <span className={styles.statusValue}>
            ({navState.robotPosition.x.toFixed(1)}, {navState.robotPosition.y.toFixed(1)})
          </span>
        </div>
      </div>

      <div className={styles.costmapVisualization}>
        <h4>Costmap Visualization</h4>
        {navState.costmap ? (
          <div className={styles.costmapGrid}>
            {navState.costmap.map((row, y) => (
              <div key={y} className={styles.costmapRow}>
                {row.map((cell, x) => {
                  let cellClass = styles.costmapCell;
                  if (cell === 255) cellClass += ` ${styles.cellUnknown}`;
                  else if (cell >= 200) cellClass += ` ${styles.cellOccupied}`;
                  else if (cell >= 150) cellClass += ` ${styles.cellHighCost}`;
                  else if (cell >= 100) cellClass += ` ${styles.cellMediumCost}`;
                  else if (cell >= 50) cellClass += ` ${styles.cellLowCost}`;
                  else cellClass += ` ${styles.cellFree}`;

                  // Highlight robot position
                  if (Math.floor(navState.robotPosition.x) === x && Math.floor(navState.robotPosition.y) === y) {
                    cellClass += ` ${styles.cellRobot}`;
                  }

                  // Highlight goal position
                  if (navState.currentGoal &&
                      Math.floor(navState.currentGoal.x) === x &&
                      Math.floor(navState.currentGoal.y) === y) {
                    cellClass += ` ${styles.cellGoal}`;
                  }

                  // Highlight path
                  if (navState.path.some(point =>
                    Math.floor(point.x) === x && Math.floor(point.y) === y)) {
                    cellClass += ` ${styles.cellPath}`;
                  }

                  return (
                    <div key={`${x}-${y}`} className={cellClass}>
                      {cell > 200 ? '██' : cell > 150 ? '▓▓' : cell > 100 ? '▒▒' : cell > 50 ? '░░' : '  '}
                    </div>
                  );
                })}
              </div>
            ))}
          </div>
        ) : (
          <div className={styles.emptyCostmap}>No costmap data available</div>
        )}
      </div>

      <div className={styles.recoveryControls}>
        <h4>Recovery Behaviors</h4>
        <div className={styles.recoveryButtons}>
          <button
            className={styles.recoveryButton}
            onClick={() => triggerRecovery('spin')}
          >
            Spin Recovery
          </button>
          <button
            className={styles.recoveryButton}
            onClick={() => triggerRecovery('backup')}
          >
            Backup Recovery
          </button>
          <button
            className={styles.recoveryButton}
            onClick={() => triggerRecovery('wait')}
          >
            Wait Recovery
          </button>
          <button
            className={styles.recoveryButton}
            onClick={() => triggerRecovery('clear_costmap')}
          >
            Clear Costmap
          </button>
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
          <div className={styles.constraintList}>
            <div className={styles.constraintItem}>
              <span className={styles.constraintName}>Max Step Size:</span>
              <span className={styles.constraintValue}>0.3m</span>
            </div>
            <div className={styles.constraintItem}>
              <span className={styles.constraintName}>Min Turning Radius:</span>
              <span className={styles.constraintValue}>0.4m</span>
            </div>
            <div className={styles.constraintItem}>
              <span className={styles.constraintName}>Max Incline Angle:</span>
              <span className={styles.constraintValue}>15°</span>
            </div>
          </div>
        </div>

        <div className={styles.considerationCard}>
          <h4>Social Navigation</h4>
          <p>Humanoid robots must navigate in human environments respecting social norms and personal space.</p>
          <div className={styles.constraintList}>
            <div className={styles.constraintItem}>
              <span className={styles.constraintName}>Personal Space:</span>
              <span className={styles.constraintValue}>1.0m radius</span>
            </div>
            <div className={styles.constraintItem}>
              <span className={styles.constraintName}>Social Lane Width:</span>
              <span className={styles.constraintValue}>1.2m</span>
            </div>
            <div className={styles.constraintItem}>
              <span className={styles.constraintName}>Interaction Protocol:</span>
              <span className={styles.constraintValue}>Yield to humans</span>
            </div>
          </div>
        </div>

        <div className={styles.considerationCard}>
          <h4>Stability Requirements</h4>
          <p>Navigation paths must ensure the humanoid robot maintains stability throughout the movement.</p>
          <div className={styles.constraintList}>
            <div className={styles.constraintItem}>
              <span className={styles.constraintName}>ZMP Limits:</span>
              <span className={styles.constraintValue}>Maintain balance</span>
            </div>
            <div className={styles.constraintItem}>
              <span className={styles.constraintName}>Footstep Planning:</span>
              <span className={styles.constraintValue}>Sequential placement</span>
            </div>
            <div className={styles.constraintItem}>
              <span className={styles.constraintName}>CoM Management:</span>
              <span className={styles.constraintValue}>Controlled movement</span>
            </div>
          </div>
        </div>
      </div>

      <div className={styles.tuningParameters}>
        <h4>Nav2 Parameters Tuning for Humanoids</h4>
        <table className={styles.parametersTable}>
          <thead>
            <tr>
              <th>Parameter</th>
              <th>Wheeled Robot</th>
              <th>Humanoid Robot</th>
              <th>Rationale</th>
            </tr>
          </thead>
          <tbody>
            <tr>
              <td>Max Velocity (linear)</td>
              <td>1.0 m/s</td>
              <td>0.3 m/s</td>
              <td>Slower for balance and stability</td>
            </tr>
            <tr>
              <td>Max Acceleration</td>
              <td>2.5 m/s²</td>
              <td>0.8 m/s²</td>
              <td>Gradual changes for stability</td>
            </tr>
            <tr>
              <td>Min Turning Radius</td>
              <td>0.1 m</td>
              <td>0.4 m</td>
              <td>Account for bipedal mechanics</td>
            </tr>
            <tr>
              <td>Local Planner Window</td>
              <td>3.0 x 3.0 m</td>
              <td>2.0 x 2.0 m</td>
              <td>Smaller for precision</td>
            </tr>
            <tr>
              <td>Social Distance</td>
              <td>N/A</td>
              <td>1.0 m</td>
              <td>Respect human personal space</td>
            </tr>
          </tbody>
        </table>
      </div>
    </div>
  );

  const renderContent = () => {
    switch (activeTab) {
      case 'overview':
        return renderOverview();
      case 'visualization':
        return renderVisualization();
      case 'humanoid':
        return renderHumanoidConsiderations();
      default:
        return renderOverview();
    }
  };

  return (
    <div className={styles.nav2Integration}>
      <div className={styles.header}>
        <h2>Nav2 Integration</h2>
        <p>Understanding the integration of Nav2 navigation stack with Isaac Sim for humanoid robotics</p>
      </div>

      <div className={styles.tabs}>
        <button
          className={`${styles.tabButton} ${activeTab === 'overview' ? styles.active : ''}`}
          onClick={() => setActiveTab('overview')}
        >
          Architecture
        </button>
        <button
          className={`${styles.tabButton} ${activeTab === 'visualization' ? styles.active : ''}`}
          onClick={() => setActiveTab('visualization')}
        >
          Simulation
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
            <h4>Robust Navigation</h4>
            <p>Nav2 provides mature, well-tested navigation capabilities that can be adapted for humanoid-specific requirements with appropriate parameter tuning.</p>
          </div>

          <div className={styles.benefit}>
            <h4>Recovery Behaviors</h4>
            <p>Integrated recovery behaviors help humanoid robots handle navigation failures gracefully, which is critical for bipedal stability.</p>
          </div>

          <div className={styles.benefit}>
            <h4>Behavior Trees</h4>
            <p>Modular behavior trees allow for complex navigation strategies that can account for humanoid-specific constraints and requirements.</p>
          </div>

          <div className={styles.benefit}>
            <h4>Social Compliance</h4>
            <p>Navigation behaviors can be configured to respect human social spaces and movement patterns, essential for humanoid deployment.</p>
          </div>
        </div>
      </div>
    </div>
  );
};

export default Nav2Integration;