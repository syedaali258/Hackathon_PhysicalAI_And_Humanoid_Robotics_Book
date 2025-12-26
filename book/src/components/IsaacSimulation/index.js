import React, { useState, useEffect } from 'react';
import styles from './IsaacSimulation.module.css';

/**
 * IsaacSimulation Component
 * Interactive visualization component for Isaac Sim and synthetic data generation concepts
 */
const IsaacSimulation = () => {
  const [currentView, setCurrentView] = useState('overview');
  const [simulationData, setSimulationData] = useState(null);
  const [isLoading, setIsLoading] = useState(false);

  // Mock data for Isaac Sim visualization
  const mockSimulationData = {
    overview: {
      title: "Isaac Sim Overview",
      description: "NVIDIA's reference application for robotics simulation and synthetic data generation",
      features: [
        "Photorealistic Rendering",
        "Physically Accurate Simulation",
        "Advanced Sensor Simulation",
        "Synthetic Data Generation"
      ],
      benefits: [
        "Reduced hardware costs",
        "Safe testing environment",
        "Diverse scenario creation",
        "Automatic ground truth generation"
      ]
    },
    sensors: {
      title: "Sensor Simulation in Isaac Sim",
      description: "Accurate modeling of various sensor types for realistic data generation",
      sensorTypes: [
        {
          name: "RGB Camera",
          description: "High-fidelity camera simulation with realistic optics",
          useCase: "Computer vision training"
        },
        {
          name: "LiDAR",
          description: "Light Detection and Ranging with accurate beam modeling",
          useCase: "3D mapping and obstacle detection"
        },
        {
          name: "IMU",
          description: "Inertial Measurement Unit with realistic noise models",
          useCase: "Localization and balance control"
        },
        {
          name: "Depth Camera",
          description: "Stereo and structured light depth sensing",
          useCase: "3D scene understanding"
        }
      ]
    },
    syntheticData: {
      title: "Synthetic Data Generation Pipeline",
      description: "Process for creating training datasets using Isaac Sim",
      steps: [
        {
          step: 1,
          title: "Environment Setup",
          description: "Configure scene with objects, lighting, and physics properties"
        },
        {
          step: 2,
          title: "Sensor Configuration",
          description: "Set up virtual sensors with realistic parameters"
        },
        {
          step: 3,
          title: "Data Collection",
          description: "Run simulation and collect sensor data"
        },
        {
          step: 4,
          title: "Annotation Generation",
          description: "Automatically generate ground truth labels"
        },
        {
          step: 5,
          title: "Dataset Assembly",
          description: "Organize data into training-ready format"
        }
      ]
    }
  };

  useEffect(() => {
    setIsLoading(true);
    // Simulate data loading
    setTimeout(() => {
      setSimulationData(mockSimulationData[currentView]);
      setIsLoading(false);
    }, 300);
  }, [currentView]);

  const renderOverview = () => (
    <div className={styles.contentSection}>
      <h3>{simulationData?.title}</h3>
      <p className={styles.description}>{simulationData?.description}</p>

      <div className={styles.featuresGrid}>
        <div className={styles.column}>
          <h4>Key Features</h4>
          <ul className={styles.featureList}>
            {simulationData?.features.map((feature, index) => (
              <li key={index} className={styles.featureItem}>
                <span className={styles.featureIcon}>✓</span>
                {feature}
              </li>
            ))}
          </ul>
        </div>
        <div className={styles.column}>
          <h4>Benefits</h4>
          <ul className={styles.benefitList}>
            {simulationData?.benefits.map((benefit, index) => (
              <li key={index} className={styles.benefitItem}>
                <span className={styles.benefitIcon}>★</span>
                {benefit}
              </li>
            ))}
          </ul>
        </div>
      </div>
    </div>
  );

  const renderSensors = () => (
    <div className={styles.contentSection}>
      <h3>{simulationData?.title}</h3>
      <p className={styles.description}>{simulationData?.description}</p>

      <div className={styles.sensorsGrid}>
        {simulationData?.sensorTypes.map((sensor, index) => (
          <div key={index} className={styles.sensorCard}>
            <h4>{sensor.name}</h4>
            <p className={styles.sensorDescription}>{sensor.description}</p>
            <div className={styles.useCase}>
              <strong>Use Case:</strong> {sensor.useCase}
            </div>
          </div>
        ))}
      </div>
    </div>
  );

  const renderSyntheticData = () => (
    <div className={styles.contentSection}>
      <h3>{simulationData?.title}</h3>
      <p className={styles.description}>{simulationData?.description}</p>

      <div className={styles.stepsContainer}>
        {simulationData?.steps.map((step) => (
          <div key={step.step} className={styles.stepItem}>
            <div className={styles.stepNumber}>{step.step}</div>
            <div className={styles.stepContent}>
              <h4>{step.title}</h4>
              <p>{step.description}</p>
            </div>
          </div>
        ))}
      </div>
    </div>
  );

  const renderContent = () => {
    if (isLoading) {
      return (
        <div className={styles.loading}>
          <div className={styles.spinner}>Loading...</div>
        </div>
      );
    }

    switch (currentView) {
      case 'overview':
        return renderOverview();
      case 'sensors':
        return renderSensors();
      case 'syntheticData':
        return renderSyntheticData();
      default:
        return renderOverview();
    }
  };

  return (
    <div className={styles.isaacSimulation}>
      <div className={styles.header}>
        <h2>Isaac Sim Visualization</h2>
        <p>Interactive demonstration of Isaac Sim capabilities for humanoid robotics</p>
      </div>

      <div className={styles.controls}>
        <button
          className={`${styles.controlButton} ${currentView === 'overview' ? styles.active : ''}`}
          onClick={() => setCurrentView('overview')}
        >
          Overview
        </button>
        <button
          className={`${styles.controlButton} ${currentView === 'sensors' ? styles.active : ''}`}
          onClick={() => setCurrentView('sensors')}
        >
          Sensor Simulation
        </button>
        <button
          className={`${styles.controlButton} ${currentView === 'syntheticData' ? styles.active : ''}`}
          onClick={() => setCurrentView('syntheticData')}
        >
          Data Generation
        </button>
      </div>

      <div className={styles.visualizationArea}>
        {renderContent()}
      </div>

      <div className={styles.explanation}>
        <h3>How Isaac Sim Benefits Humanoid Robotics</h3>
        <p>
          Isaac Sim enables the development of humanoid robots by providing a safe, cost-effective
          environment for testing AI algorithms. The synthetic data generated can be used to train
          perception, navigation, and manipulation systems without requiring expensive physical
          hardware or risking damage to real robots.
        </p>
      </div>
    </div>
  );
};

export default IsaacSimulation;