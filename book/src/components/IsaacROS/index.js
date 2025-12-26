import React, { useState, useEffect } from 'react';
import styles from './IsaacROS.module.css';

/**
 * IsaacROS Component
 * Visualization and explanation of Isaac ROS integration for humanoid robotics
 */
const IsaacROS = () => {
  const [activeTab, setActiveTab] = useState('overview');
  const [isSimulating, setIsSimulating] = useState(false);
  const [simulationData, setSimulationData] = useState(null);

  const packages = [
    {
      name: 'isaac_ros_visual_slam',
      description: 'Visual SLAM package for Isaac ROS with GPU acceleration',
      useCase: 'Building maps and localizing humanoid robots using visual sensors',
      status: 'optimized'
    },
    {
      name: 'isaac_ros_pose_graph_localizer',
      description: 'Pose Graph Localizer for global localization in existing maps',
      useCase: 'Finding robot position in pre-built maps for humanoid navigation',
      status: 'optimized'
    },
    {
      name: 'isaac_ros_detect_net',
      description: 'Deep learning-based object detection using TensorRT',
      useCase: 'Detecting humans, objects, and obstacles for humanoid social navigation',
      status: 'optimized'
    },
    {
      name: 'isaac_ros_nitros_image_publisher',
      description: 'High-performance image transport using Nitros messaging',
      useCase: 'Efficient image transport between Isaac ROS nodes',
      status: 'optimized'
    }
  ];

  const simulationSteps = [
    {
      step: 1,
      title: 'Sensor Data Acquisition',
      description: 'Collect synchronized data from cameras, IMU, and other sensors',
      code: `# Isaac ROS Sensor Acquisition
from rclpy import create_node
from sensor_msgs.msg import Image, Imu
from message_filters import ApproximateTimeSynchronizer, Subscriber

class IsaacSensorAcquisition(Node):
    def __init__(self):
        super().__init__('isaac_sensor_acquisition')

        # Create subscribers for sensor data
        self.image_sub = Subscriber(self, Image, '/camera/image_rect_color')
        self.imu_sub = Subscriber(self, Imu, '/imu/data')

        # Synchronize sensor streams
        self.sync = ApproximateTimeSynchronizer(
            [self.image_sub, self.imu_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.sensor_callback)`
    },
    {
      step: 2,
      title: 'Feature Extraction',
      description: 'Extract visual features using GPU-accelerated algorithms',
      code: `# Isaac ROS Feature Extraction
from isaac_ros_visual_slam import FeatureExtractorNode

class IsaacFeatureExtractor(FeatureExtractorNode):
    def __init__(self):
        super().__init__('isaac_feature_extractor')

        # Configure GPU-accelerated feature extraction
        self.set_parameters([
            Parameter('max_features', value=1000),
            Parameter('min_features', value=100),
            Parameter('detection_threshold', value=20)
        ])`
    },
    {
      step: 3,
      title: 'SLAM Optimization',
      description: 'Optimize robot trajectory and map using graph optimization',
      code: `# Isaac ROS SLAM Backend
from isaac_ros_visual_slam import SlambasedBackendNode

class IsaacSlamBackend(SlambasedBackendNode):
    def __init__(self):
        super().__init__('isaac_slam_backend')

        # Configure optimization parameters
        self.set_parameters([
            Parameter('enable_localization', value=True),
            Parameter('enable_mapping', value=True),
            Parameter('enable_loop_closure', value=True)
        ])`
    }
  ];

  const startSimulation = () => {
    setIsSimulating(true);

    // Simulate Isaac ROS pipeline
    const simulationResult = {
      processingTime: '2.3ms',
      featureCount: 847,
      trackingStatus: 'OK',
      localizationAccuracy: '98.7%',
      mapQuality: 'High'
    };

    setTimeout(() => {
      setSimulationData(simulationResult);
      setIsSimulating(false);
    }, 2000);
  };

  const renderOverview = () => (
    <div className={styles.overview}>
      <h3>Isaac ROS Package Ecosystem</h3>

      <div className={styles.packagesGrid}>
        {packages.map((pkg, index) => (
          <div key={index} className={`${styles.packageCard} ${styles[pkg.status]}`}>
            <div className={styles.packageHeader}>
              <h4>{pkg.name}</h4>
              <span className={styles.statusBadge}>{pkg.status}</span>
            </div>
            <p className={styles.packageDescription}>{pkg.description}</p>
            <div className={styles.useCase}>
              <strong>Use Case:</strong> {pkg.useCase}
            </div>
          </div>
        ))}
      </div>

      <div className={styles.integrationDiagram}>
        <h4>Isaac ROS Integration Architecture</h4>
        <div className={styles.diagram}>
          <div className={styles.layer}>Applications</div>
          <div className={styles.arrow}>↓</div>
          <div className={styles.layer}>Isaac ROS Packages</div>
          <div className={styles.arrow}>↓</div>
          <div className={styles.layer}>ROS 2 Ecosystem</div>
          <div className={styles.arrow}>↓</div>
          <div className={styles.layer}>Hardware Abstraction</div>
        </div>
      </div>
    </div>
  );

  const renderPipeline = () => (
    <div className={styles.pipeline}>
      <h3>Isaac ROS Processing Pipeline</h3>

      <div className={styles.simulationControls}>
        <button
          className={styles.simulationButton}
          onClick={startSimulation}
          disabled={isSimulating}
        >
          {isSimulating ? 'Running Isaac ROS Pipeline...' : 'Run Simulation'}
        </button>
      </div>

      {isSimulating && (
        <div className={styles.simulationProgress}>
          <div className={styles.spinner}></div>
          <p>Processing sensor data through Isaac ROS pipeline...</p>
        </div>
      )}

      {simulationData && (
        <div className={styles.simulationResults}>
          <h4>Pipeline Results</h4>
          <div className={styles.resultsGrid}>
            <div className={styles.resultItem}>
              <span className={styles.resultLabel}>Processing Time:</span>
              <span className={styles.resultValue}>{simulationData.processingTime}</span>
            </div>
            <div className={styles.resultItem}>
              <span className={styles.resultLabel}>Features Detected:</span>
              <span className={styles.resultValue}>{simulationData.featureCount}</span>
            </div>
            <div className={styles.resultItem}>
              <span className={styles.resultLabel}>Tracking Status:</span>
              <span className={styles.resultValue}>{simulationData.trackingStatus}</span>
            </div>
            <div className={styles.resultItem}>
              <span className={styles.resultLabel}>Accuracy:</span>
              <span className={styles.resultValue}>{simulationData.localizationAccuracy}</span>
            </div>
          </div>
        </div>
      )}

      <div className={styles.pipelineSteps}>
        {simulationSteps.map((step) => (
          <div key={step.step} className={styles.pipelineStep}>
            <div className={styles.stepHeader}>
              <div className={styles.stepNumber}>{step.step}</div>
              <h4>{step.title}</h4>
            </div>
            <p>{step.description}</p>
            <div className={styles.codeBlock}>
              <pre><code>{step.code}</code></pre>
            </div>
          </div>
        ))}
      </div>
    </div>
  );

  const renderBenefits = () => (
    <div className={styles.benefits}>
      <h3>Benefits of Isaac ROS for Humanoid Robotics</h3>

      <div className={styles.benefitsGrid}>
        <div className={styles.benefitCard}>
          <div className={styles.benefitIcon}>⚡</div>
          <h4>GPU Acceleration</h4>
          <p>Isaac ROS packages leverage GPU acceleration for real-time processing of sensor data, essential for humanoid robots that need immediate responses.</p>
        </div>

        <div className={styles.benefitCard}>
          <div className={styles.benefitIcon}>🎯</div>
          <h4>Optimized Algorithms</h4>
          <p>Algorithms specifically optimized for robotics applications with considerations for humanoid robot kinematics and dynamics.</p>
        </div>

        <div className={styles.benefitCard}>
          <div className={styles.benefitIcon}>🔗</div>
          <h4>Seamless Integration</h4>
          <p>Native integration with ROS 2 ecosystem while providing NVIDIA-specific optimizations and features.</p>
        </div>

        <div className={styles.benefitCard}>
          <div className={styles.benefitIcon}>🔬</div>
          <h4>Research-Ready</h4>
          <p>Well-documented packages suitable for both educational purposes and cutting-edge research in humanoid robotics.</p>
        </div>
      </div>

      <div className={styles.performanceComparison}>
        <h4>Performance Comparison</h4>
        <table className={styles.comparisonTable}>
          <thead>
            <tr>
              <th>Metric</th>
              <th>Traditional ROS Package</th>
              <th>Isaac ROS Package</th>
            </tr>
          </thead>
          <tbody>
            <tr>
              <td>Processing Speed</td>
              <td>CPU-bound, ~10Hz</td>
              <td>GPU-accelerated, ~60Hz</td>
            </tr>
            <tr>
              <td>Accuracy</td>
              <td>Standard algorithms</td>
              <td>Optimized for robotics</td>
            </tr>
            <tr>
              <td>Memory Usage</td>
              <td>Higher CPU usage</td>
              <td>Efficient GPU utilization</td>
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
      case 'pipeline':
        return renderPipeline();
      case 'benefits':
        return renderBenefits();
      default:
        return renderOverview();
    }
  };

  return (
    <div className={styles.isaacROS}>
      <div className={styles.header}>
        <h2>Isaac ROS Integration</h2>
        <p>Understanding the integration of NVIDIA Isaac packages with ROS 2 for humanoid robotics</p>
      </div>

      <div className={styles.tabs}>
        <button
          className={`${styles.tabButton} ${activeTab === 'overview' ? styles.active : ''}`}
          onClick={() => setActiveTab('overview')}
        >
          Package Overview
        </button>
        <button
          className={`${styles.tabButton} ${activeTab === 'pipeline' ? styles.active : ''}`}
          onClick={() => setActiveTab('pipeline')}
        >
          Processing Pipeline
        </button>
        <button
          className={`${styles.tabButton} ${activeTab === 'benefits' ? styles.active : ''}`}
          onClick={() => setActiveTab('benefits')}
        >
          Benefits & Performance
        </button>
      </div>

      <div className={styles.contentArea}>
        {renderContent()}
      </div>

      <div className={styles.explanation}>
        <h3>Isaac ROS for Humanoid Robot Perception</h3>
        <p>
          Isaac ROS bridges the gap between NVIDIA's GPU-accelerated computing capabilities and the flexibility of ROS 2.
          For humanoid robots, this combination is particularly powerful as it enables real-time processing of complex
          sensor data streams that would be computationally prohibitive on traditional CPU-only systems.
        </p>

        <div className={styles.keyPoints}>
          <div className={styles.point}>
            <h4>GPU-Accelerated Perception</h4>
            <p>Isaac ROS packages utilize CUDA and TensorRT for high-performance computer vision and machine learning tasks.</p>
          </div>

          <div className={styles.point}>
            <h4>Real-time SLAM</h4>
            <p>Visual-inertial SLAM algorithms optimized for humanoid robot mobility patterns and social navigation.</p>
          </div>

          <div className={styles.point}>
            <h4>Simulation-to-Reality Transfer</h4>
            <p>Tight integration with Isaac Sim ensures that perception algorithms trained in simulation transfer effectively to real robots.</p>
          </div>
        </div>
      </div>
    </div>
  );
};

export default IsaacROS;