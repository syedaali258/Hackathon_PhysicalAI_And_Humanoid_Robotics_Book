import React, { useState, useEffect, useRef } from 'react';
import styles from './SLAMVisualization.module.css';

/**
 * SLAMVisualization Component
 * Interactive visualization for Visual SLAM concepts with Isaac ROS
 */
const SLAMVisualization = () => {
  const [currentView, setCurrentView] = useState('overview');
  const [slamState, setSlamState] = useState({
    status: 'idle',
    position: { x: 0, y: 0, z: 0 },
    orientation: { x: 0, y: 0, z: 0, w: 1 },
    features: [],
    map: [],
    confidence: 0.0
  });
  const [isRunning, setIsRunning] = useState(false);
  const canvasRef = useRef(null);

  // Mock SLAM simulation data
  const mockFeatures = [
    { id: 1, x: 1.2, y: 0.5, z: 0.0, type: 'corner', reliability: 0.9 },
    { id: 2, x: -0.8, y: 1.3, z: 0.0, type: 'edge', reliability: 0.85 },
    { id: 3, x: 0.2, y: -1.1, z: 0.0, type: 'landmark', reliability: 0.95 },
    { id: 4, x: 2.1, y: -0.3, z: 0.0, type: 'corner', reliability: 0.75 },
    { id: 5, x: -1.5, y: -1.8, z: 0.0, type: 'edge', reliability: 0.88 }
  ];

  const mockMap = [
    { id: 'wall1', type: 'line', start: { x: -3, y: -3 }, end: { x: 3, y: -3 }, confidence: 0.95 },
    { id: 'wall2', type: 'line', start: { x: 3, y: -3 }, end: { x: 3, y: 3 }, confidence: 0.92 },
    { id: 'wall3', type: 'line', start: { x: 3, y: 3 }, end: { x: -3, y: 3 }, confidence: 0.94 },
    { id: 'wall4', type: 'line', start: { x: -3, y: 3 }, end: { x: -3, y: -3 }, confidence: 0.93 }
  ];

  // Simulate SLAM state updates
  useEffect(() => {
    if (!isRunning) return;

    const interval = setInterval(() => {
      setSlamState(prevState => ({
        ...prevState,
        position: {
          x: prevState.position.x + (Math.random() - 0.5) * 0.1,
          y: prevState.position.y + (Math.random() - 0.5) * 0.1,
          z: 0
        },
        features: mockFeatures,
        map: mockMap,
        confidence: Math.min(1.0, prevState.confidence + 0.01),
        status: 'tracking'
      }));
    }, 200); // Update every 200ms to simulate real-time SLAM

    return () => clearInterval(interval);
  }, [isRunning]);

  const startSLAM = () => {
    setIsRunning(true);
    setSlamState({
      ...slamState,
      status: 'initializing',
      confidence: 0.1
    });

    // Simulate initialization
    setTimeout(() => {
      setSlamState(prev => ({
        ...prev,
        status: 'tracking'
      }));
    }, 1000);
  };

  const stopSLAM = () => {
    setIsRunning(false);
    setSlamState(prev => ({
      ...prev,
      status: 'idle'
    }));
  };

  const resetSLAM = () => {
    setIsRunning(false);
    setSlamState({
      status: 'idle',
      position: { x: 0, y: 0, z: 0 },
      orientation: { x: 0, y: 0, z: 0, w: 1 },
      features: [],
      map: [],
      confidence: 0.0
    });
  };

  const renderOverview = () => (
    <div className={styles.overview}>
      <h3>Visual SLAM Concepts</h3>
      <div className={styles.conceptCards}>
        <div className={styles.conceptCard}>
          <h4>Feature Detection</h4>
          <p>Identifying distinctive visual features (corners, edges, landmarks) in camera images</p>
          <div className={styles.featureExample}>
            <div className={styles.cornerFeature}>⌜⌟ Corner</div>
            <div className={styles.edgeFeature}>| Edge</div>
            <div className={styles.landmarkFeature}>○ Landmark</div>
          </div>
        </div>

        <div className={styles.conceptCard}>
          <h4>Feature Tracking</h4>
          <p>Following features across consecutive frames to estimate motion</p>
          <div className={styles.trackingDiagram}>
            <div className={styles.frame}>Frame t</div>
            <div className={styles.arrow}>→</div>
            <div className={styles.frame}>Frame t+1</div>
          </div>
        </div>

        <div className={styles.conceptCard}>
          <h4>Map Building</h4>
          <p>Constructing a consistent map of the environment from tracked features</p>
          <div className={styles.mapDiagram}>
            <div className={styles.featurePoint}></div>
            <div className={styles.featurePoint}></div>
            <div className={styles.featurePoint}></div>
            <div className={styles.connectionLine}></div>
          </div>
        </div>
      </div>
    </div>
  );

  const renderSimulation = () => (
    <div className={styles.simulation}>
      <h3>SLAM Simulation</h3>

      <div className={styles.controls}>
        <button
          className={`${styles.controlButton} ${isRunning ? styles.active : ''}`}
          onClick={isRunning ? stopSLAM : startSLAM}
          disabled={slamState.status === 'initializing'}
        >
          {isRunning ? 'Stop SLAM' : 'Start SLAM'}
        </button>
        <button
          className={styles.controlButton}
          onClick={resetSLAM}
        >
          Reset
        </button>
      </div>

      <div className={styles.statusPanel}>
        <div className={styles.statusItem}>
          <span className={styles.statusLabel}>Status:</span>
          <span className={styles.statusValue}>{slamState.status}</span>
        </div>
        <div className={styles.statusItem}>
          <span className={styles.statusLabel}>Confidence:</span>
          <span className={styles.statusValue}>{(slamState.confidence * 100).toFixed(1)}%</span>
        </div>
        <div className={styles.statusItem}>
          <span className={styles.statusLabel}>Features:</span>
          <span className={styles.statusValue}>{slamState.features.length}</span>
        </div>
        <div className={styles.statusItem}>
          <span className={styles.statusLabel}>Robot Pos:</span>
          <span className={styles.statusValue}>
            ({slamState.position.x.toFixed(2)}, {slamState.position.y.toFixed(2)})
          </span>
        </div>
      </div>

      <div className={styles.visualizationCanvas}>
        <canvas
          ref={canvasRef}
          width={600}
          height={400}
          className={styles.canvas}
        />

        <div className={styles.mapLegend}>
          <div className={styles.legendItem}>
            <div className={styles.robotMarker}></div>
            <span>Robot Position</span>
          </div>
          <div className={styles.legendItem}>
            <div className={styles.featureMarker}></div>
            <span>Visual Features</span>
          </div>
          <div className={styles.legendItem}>
            <div className={styles.mapMarker}></div>
            <span>Map Features</span>
          </div>
        </div>
      </div>
    </div>
  );

  const renderArchitecture = () => (
    <div className={styles.architecture}>
      <h3>Isaac ROS Visual SLAM Architecture</h3>

      <div className={styles.pipelineDiagram}>
        <div className={styles.pipelineStage}>
          <div className={styles.stageTitle}>Input Sensors</div>
          <div className={styles.stageContent}>
            <div className={styles.sensor}>Stereo Cameras</div>
            <div className={styles.sensor}>IMU</div>
          </div>
        </div>

        <div className={styles.arrow}>→</div>

        <div className={styles.pipelineStage}>
          <div className={styles.stageTitle}>Feature Processing</div>
          <div className={styles.stageContent}>
            <div className={styles.component}>Feature Detection</div>
            <div className={styles.component}>Feature Matching</div>
          </div>
        </div>

        <div className={styles.arrow}>→</div>

        <div className={styles.pipelineStage}>
          <div className={styles.stageTitle}>SLAM Backend</div>
          <div className={styles.stageContent}>
            <div className={styles.component}>Pose Estimation</div>
            <div className={styles.component}>Map Building</div>
            <div className={styles.component}>Optimization</div>
          </div>
        </div>

        <div className={styles.arrow}>→</div>

        <div className={styles.pipelineStage}>
          <div className={styles.stageTitle}>Output</div>
          <div className={styles.stageContent}>
            <div className={styles.output}>Odometry</div>
            <div className={styles.output}>Map</div>
            <div className={styles.output}>Pose Graph</div>
          </div>
        </div>
      </div>

      <div className={styles.performanceMetrics}>
        <h4>Performance Characteristics</h4>
        <div className={styles.metricsGrid}>
          <div className={styles.metric}>
            <div className={styles.metricValue}>30+</div>
            <div className={styles.metricLabel}>FPS Processing</div>
          </div>
          <div className={styles.metric}>
            <div className={styles.metricValue}>cm</div>
            <div className={styles.metricLabel}>Accuracy</div>
          </div>
          <div className={styles.metric}>
            <div className={styles.metricValue}>1000+</div>
            <div className={styles.metricLabel}>Features Tracked</div>
          </div>
          <div className={styles.metric}>
            <div className={styles.metricValue}>GPU</div>
            <div className={styles.metricLabel}>Acceleration</div>
          </div>
        </div>
      </div>
    </div>
  );

  const renderContent = () => {
    switch (currentView) {
      case 'overview':
        return renderOverview();
      case 'simulation':
        return renderSimulation();
      case 'architecture':
        return renderArchitecture();
      default:
        return renderOverview();
    }
  };

  return (
    <div className={styles.slamVisualization}>
      <div className={styles.header}>
        <h2>Visual SLAM Visualization</h2>
        <p>Interactive demonstration of Isaac ROS Visual SLAM for humanoid robot perception</p>
      </div>

      <div className={styles.tabs}>
        <button
          className={`${styles.tabButton} ${currentView === 'overview' ? styles.active : ''}`}
          onClick={() => setCurrentView('overview')}
        >
          Concepts
        </button>
        <button
          className={`${styles.tabButton} ${currentView === 'simulation' ? styles.active : ''}`}
          onClick={() => setCurrentView('simulation')}
        >
          Simulation
        </button>
        <button
          className={`${styles.tabButton} ${currentView === 'architecture' ? styles.active : ''}`}
          onClick={() => setCurrentView('architecture')}
        >
          Architecture
        </button>
      </div>

      <div className={styles.contentArea}>
        {renderContent()}
      </div>

      <div className={styles.explanation}>
        <h3>How Visual SLAM Enables Humanoid Navigation</h3>
        <div className={styles.benefitsGrid}>
          <div className={styles.benefit}>
            <h4>Real-time Mapping</h4>
            <p>Humanoid robots can build maps of unknown environments as they explore, enabling autonomous navigation without pre-existing maps.</p>
          </div>

          <div className={styles.benefit}>
            <h4>Robust Localization</h4>
            <p>Continuous tracking of robot position relative to the map, even in dynamic human environments with moving objects.</p>
          </div>

          <div className={styles.benefit}>
            <h4>Visual Understanding</h4>
            <p>Leveraging visual features abundant in human environments, making it ideal for humanoid robots operating alongside humans.</p>
          </div>

          <div className={styles.benefit}>
            <h4>GPU Acceleration</h4>
            <p>Isaac ROS provides optimized implementations that leverage GPU acceleration for real-time performance on humanoid robot platforms.</p>
          </div>
        </div>
      </div>
    </div>
  );
};

export default SLAMVisualization;