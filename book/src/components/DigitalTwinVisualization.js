import React, { useState } from 'react';
import styles from './DigitalTwinVisualization.module.css';

const DigitalTwinVisualization = () => {
  const [activeView, setActiveView] = useState('conceptual');
  const [animationState, setAnimationState] = useState('paused');

  const toggleAnimation = () => {
    setAnimationState(animationState === 'paused' ? 'playing' : 'paused');
  };

  const renderConceptualView = () => {
    return (
      <div className={styles.viewContainer}>
        <div className={styles.element} style={{ left: '20%', top: '40%' }}>
          <div className={`${styles.robotIcon} ${styles.physical}`}></div>
          <div className={styles.label}>Physical Robot</div>
        </div>

        <div className={styles.element} style={{ left: '70%', top: '40%' }}>
          <div className={`${styles.robotIcon} ${styles.virtual}`}></div>
          <div className={styles.label}>Digital Twin</div>
        </div>

        <div className={styles.connectionLine}>
          <div className={styles.arrow}></div>
          <div className={styles.bidirectional}></div>
        </div>

        <div className={styles.explanation}>
          <h4>Key Concepts:</h4>
          <ul>
            <li><strong>Real-time Sync</strong>: Data flows between physical and virtual</li>
            <li><strong>Predictive</strong>: Twin predicts physical behavior</li>
            <li><strong>Iterative</strong>: Updates improve accuracy over time</li>
          </ul>
        </div>
      </div>
    );
  };

  const renderArchitecturalView = () => {
    return (
      <div className={styles.archViewContainer}>
        <div className={styles.layer}>
          <h4>Sensing Layer</h4>
          <div className={`${styles.component} ${styles.sensor}`}>Sensors & Actuators</div>
        </div>

        <div className={styles.layer}>
          <h4>Simulation Layer</h4>
          <div className={`${styles.component} ${styles.physics}`}>Physics Engine (Gazebo)</div>
        </div>

        <div className={styles.layer}>
          <h4>Control Layer</h4>
          <div className={`${styles.component} ${styles.control}`}>Control Systems</div>
        </div>

        <div className={styles.layer}>
          <h4>Intelligence Layer</h4>
          <div className={`${styles.component} ${styles.ai}`}>AI Algorithms</div>
        </div>

        <div className={styles.layer}>
          <h4>Interaction Layer</h4>
          <div className={`${styles.component} ${styles.interface}`}>Human Interface (Unity)</div>
        </div>

        <div className={styles.flowArrows}>
          <div className={styles.arrowDown}>Data Flow</div>
          <div className={styles.arrowUp}>Control Flow</div>
        </div>
      </div>
    );
  };

  const renderLifecycleView = () => {
    const stages = [
      { id: 'design', name: 'Design', description: 'Initial robot design and modeling' },
      { id: 'simulate', name: 'Simulate', description: 'Testing in virtual environment' },
      { id: 'deploy', name: 'Deploy', description: 'Physical robot implementation' },
      { id: 'monitor', name: 'Monitor', description: 'Real-time data collection' },
      { id: 'update', name: 'Update', description: 'Refine digital twin based on data' }
    ];

    return (
      <div className={styles.lifecycleContainer}>
        <div className={styles.lifecycleSteps}>
          {stages.map((stage, index) => (
            <div key={stage.id} className={styles.stage}>
              <div className={styles.stageNumber}>{index + 1}</div>
              <div className={styles.stageName}>{stage.name}</div>
              <div className={styles.stageDesc}>{stage.description}</div>
              {index < stages.length - 1 && <div className={styles.stageArrow}>→</div>}
            </div>
          ))}
        </div>

        <div className={styles.lifecycleLoop}>
          <div className={styles.loopLabel}>Continuous Improvement Loop</div>
        </div>
      </div>
    );
  };

  return (
    <div className={styles.container}>
      <div className={styles.header}>
        <h3>Digital Twin Visualization</h3>
        <div className={styles.controls}>
          <button
            className={`${styles.viewButton} ${activeView === 'conceptual' ? styles.active : ''}`}
            onClick={() => setActiveView('conceptual')}
          >
            Conceptual View
          </button>
          <button
            className={`${styles.viewButton} ${activeView === 'architectural' ? styles.active : ''}`}
            onClick={() => setActiveView('architectural')}
          >
            Architecture
          </button>
          <button
            className={`${styles.viewButton} ${activeView === 'lifecycle' ? styles.active : ''}`}
            onClick={() => setActiveView('lifecycle')}
          >
            Lifecycle
          </button>
          <button
            className={`${styles.animButton} ${animationState === 'playing' ? styles.playing : ''}`}
            onClick={toggleAnimation}
          >
            {animationState === 'playing' ? 'Pause' : 'Play'} Animation
          </button>
        </div>
      </div>

      <div className={styles.visualizationArea}>
        {activeView === 'conceptual' && renderConceptualView()}
        {activeView === 'architectural' && renderArchitecturalView()}
        {activeView === 'lifecycle' && renderLifecycleView()}
      </div>

      <div className={styles.infoPanel}>
        <h4>About Digital Twins in Robotics</h4>
        <p>
          Digital twins in robotics create virtual replicas of physical robots that mirror
          their behavior, physics, and sensory responses. This enables safe testing,
          algorithm development, and operator training without risking expensive hardware.
        </p>
        <div className={styles.keyBenefits}>
          <div className={styles.benefit}>
            <h5>Safe Experimentation</h5>
            <p>Test control algorithms without hardware risk</p>
          </div>
          <div className={styles.benefit}>
            <h5>Cost Effective</h5>
            <p>Reduce prototype costs with virtual testing</p>
          </div>
          <div className={styles.benefit}>
            <h5>Rapid Iteration</h5>
            <p>Quickly test multiple scenarios and configurations</p>
          </div>
        </div>
      </div>
    </div>
  );
};

export default DigitalTwinVisualization;