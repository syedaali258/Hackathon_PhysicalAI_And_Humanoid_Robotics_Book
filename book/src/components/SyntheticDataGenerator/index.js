import React, { useState, useEffect } from 'react';
import styles from './SyntheticDataGenerator.module.css';

/**
 * SyntheticDataGenerator Component
 * Interactive tool for demonstrating synthetic data generation concepts with Isaac Sim
 */
const SyntheticDataGenerator = () => {
  const [currentStep, setCurrentStep] = useState(0);
  const [generationParams, setGenerationParams] = useState({
    environment: 'indoor',
    sensorType: 'camera',
    quantity: 1000,
    resolution: 'high',
    lighting: 'random'
  });
  const [isGenerating, setIsGenerating] = useState(false);
  const [generatedData, setGeneratedData] = useState(null);

  const steps = [
    {
      title: "Environment Setup",
      description: "Configure the virtual environment for synthetic data generation"
    },
    {
      title: "Sensor Configuration",
      description: "Set up virtual sensors with realistic parameters"
    },
    {
      title: "Data Collection",
      description: "Run simulation and collect synthetic sensor data"
    },
    {
      title: "Annotation Generation",
      description: "Automatically generate ground truth labels"
    },
    {
      title: "Dataset Assembly",
      description: "Organize data into training-ready format"
    }
  ];

  const environments = [
    { id: 'indoor', name: 'Indoor Office', description: 'Office environment with furniture and people' },
    { id: 'outdoor', name: 'Outdoor Park', description: 'Park environment with trees and pathways' },
    { id: 'warehouse', name: 'Warehouse', description: 'Industrial warehouse with shelves and equipment' },
    { id: 'home', name: 'Home Environment', description: 'Residential setting with household items' }
  ];

  const sensorTypes = [
    { id: 'camera', name: 'RGB Camera', description: 'Photorealistic images with depth information' },
    { id: 'lidar', name: 'LiDAR', description: '3D point cloud data with accurate geometry' },
    { id: 'imu', name: 'IMU', description: 'Inertial measurement unit data' },
    { id: 'depth', name: 'Depth Camera', description: 'Stereo vision with depth maps' }
  ];

  const handleParamChange = (param, value) => {
    setGenerationParams(prev => ({
      ...prev,
      [param]: value
    }));
  };

  const handleGenerate = async () => {
    setIsGenerating(true);

    // Simulate data generation process
    const mockData = {
      dataType: generationParams.sensorType,
      environment: generationParams.environment,
      quantity: generationParams.quantity,
      resolution: generationParams.resolution,
      timestamp: new Date().toISOString(),
      filesGenerated: [],
      annotations: {
        boundingBoxes: generationParams.quantity * 0.8, // 80% of samples have bounding boxes
        segmentation: generationParams.quantity * 0.9,  // 90% have segmentation
        depthMaps: generationParams.quantity * 0.7,     // 70% have depth maps
        poses: generationParams.quantity * 0.6          // 60% have pose estimates
      },
      qualityMetrics: {
        realismScore: Math.random() * 0.3 + 0.7, // 0.7-1.0
        diversityScore: Math.random() * 0.4 + 0.6, // 0.6-1.0
        annotationAccuracy: 1.0 // Synthetic data has perfect annotations
      }
    };

    // Generate mock filenames
    for (let i = 0; i < Math.min(generationParams.quantity, 100); i++) {
      const extension = generationParams.sensorType === 'lidar' ? 'pcd' :
                       generationParams.sensorType === 'imu' ? 'csv' :
                       'png';
      mockData.filesGenerated.push(`synthetic_${generationParams.sensorType}_${i.toString().padStart(4, '0')}.${extension}`);
    }

    // Simulate generation delay
    setTimeout(() => {
      setGeneratedData(mockData);
      setIsGenerating(false);
    }, 2000);
  };

  const renderStepIndicator = () => (
    <div className={styles.stepIndicator}>
      {steps.map((step, index) => (
        <div
          key={index}
          className={`${styles.step} ${index <= currentStep ? styles.completed : ''} ${index === currentStep ? styles.active : ''}`}
          onClick={() => setCurrentStep(index)}
        >
          <div className={styles.stepNumber}>
            {index < currentStep ? '✓' : index + 1}
          </div>
          <div className={styles.stepLabel}>
            <div className={styles.stepTitle}>{step.title}</div>
            <div className={styles.stepDesc}>{step.description}</div>
          </div>
        </div>
      ))}
    </div>
  );

  const renderConfiguration = () => (
    <div className={styles.configuration}>
      <h3>Synthetic Data Generation Parameters</h3>

      <div className={styles.paramGroup}>
        <label className={styles.label}>Environment</label>
        <select
          className={styles.select}
          value={generationParams.environment}
          onChange={(e) => handleParamChange('environment', e.target.value)}
        >
          {environments.map(env => (
            <option key={env.id} value={env.id}>{env.name}</option>
          ))}
        </select>
        <div className={styles.paramDesc}>
          {environments.find(e => e.id === generationParams.environment)?.description}
        </div>
      </div>

      <div className={styles.paramGroup}>
        <label className={styles.label}>Sensor Type</label>
        <select
          className={styles.select}
          value={generationParams.sensorType}
          onChange={(e) => handleParamChange('sensorType', e.target.value)}
        >
          {sensorTypes.map(sensor => (
            <option key={sensor.id} value={sensor.id}>{sensor.name}</option>
          ))}
        </select>
        <div className={styles.paramDesc}>
          {sensorTypes.find(s => s.id === generationParams.sensorType)?.description}
        </div>
      </div>

      <div className={styles.paramGroup}>
        <label className={styles.label}>
          Quantity: {generationParams.quantity} samples
        </label>
        <input
          type="range"
          className={styles.slider}
          min="100"
          max="10000"
          step="100"
          value={generationParams.quantity}
          onChange={(e) => handleParamChange('quantity', parseInt(e.target.value))}
        />
        <div className={styles.paramDesc}>
          Number of synthetic data samples to generate
        </div>
      </div>

      <div className={styles.paramGroup}>
        <label className={styles.label}>Resolution</label>
        <select
          className={styles.select}
          value={generationParams.resolution}
          onChange={(e) => handleParamChange('resolution', e.target.value)}
        >
          <option value="low">Low (320x240)</option>
          <option value="medium">Medium (640x480)</option>
          <option value="high">High (1280x720)</option>
          <option value="ultra">Ultra (1920x1080)</option>
        </select>
      </div>

      <button
        className={styles.generateButton}
        onClick={handleGenerate}
        disabled={isGenerating}
      >
        {isGenerating ? 'Generating...' : 'Generate Synthetic Data'}
      </button>
    </div>
  );

  const renderResults = () => {
    if (!generatedData) return null;

    return (
      <div className={styles.results}>
        <h3>Generation Results</h3>

        <div className={styles.metricsGrid}>
          <div className={styles.metricCard}>
            <div className={styles.metricValue}>{generatedData.quantity}</div>
            <div className={styles.metricLabel}>Samples Generated</div>
          </div>

          <div className={styles.metricCard}>
            <div className={styles.metricValue}>
              {Math.round(generatedData.qualityMetrics.realismScore * 100)}%
            </div>
            <div className={styles.metricLabel}>Realism Score</div>
          </div>

          <div className={styles.metricCard}>
            <div className={styles.metricValue}>
              {Math.round(generatedData.qualityMetrics.diversityScore * 100)}%
            </div>
            <div className={styles.metricLabel}>Diversity Score</div>
          </div>

          <div className={styles.metricCard}>
            <div className={styles.metricValue}>100%</div>
            <div className={styles.metricLabel}>Annotation Accuracy</div>
          </div>
        </div>

        <div className={styles.annotationsGrid}>
          <div className={styles.annotationCard}>
            <div className={styles.annotationValue}>{generatedData.annotations.boundingBoxes}</div>
            <div className={styles.annotationLabel}>Bounding Boxes</div>
          </div>

          <div className={styles.annotationCard}>
            <div className={styles.annotationValue}>{generatedData.annotations.segmentation}</div>
            <div className={styles.annotationLabel}>Segmentation Masks</div>
          </div>

          <div className={styles.annotationCard}>
            <div className={styles.annotationValue}>{generatedData.annotations.depthMaps}</div>
            <div className={styles.annotationLabel}>Depth Maps</div>
          </div>

          <div className={styles.annotationCard}>
            <div className={styles.annotationValue}>{generatedData.annotations.poses}</div>
            <div className={styles.annotationLabel}>Pose Estimates</div>
          </div>
        </div>

        <div className={styles.fileList}>
          <h4>Sample Generated Files:</h4>
          <div className={styles.fileGrid}>
            {generatedData.filesGenerated.slice(0, 12).map((file, index) => (
              <div key={index} className={styles.fileItem}>
                <span className={styles.fileIcon}>📄</span>
                {file}
              </div>
            ))}
            {generatedData.filesGenerated.length > 12 && (
              <div className={styles.fileItem}>
                ... and {generatedData.filesGenerated.length - 12} more files
              </div>
            )}
          </div>
        </div>

        <div className={styles.downloadSection}>
          <button className={styles.downloadButton}>
            Download Dataset (ZIP, {(generatedData.quantity * 0.5).toFixed(1)} MB)
          </button>
        </div>
      </div>
    );
  };

  return (
    <div className={styles.syntheticDataGenerator}>
      <div className={styles.header}>
        <h2>Synthetic Data Generator</h2>
        <p>Interactive demonstration of Isaac Sim's synthetic data generation capabilities</p>
      </div>

      {renderStepIndicator()}

      <div className={styles.contentArea}>
        {renderConfiguration()}

        {isGenerating && (
          <div className={styles.generatingOverlay}>
            <div className={styles.spinner}>Generating synthetic data...</div>
            <div className={styles.progressText}>
              Creating {generationParams.quantity} synthetic {generationParams.sensorType} samples
            </div>
          </div>
        )}

        {renderResults()}
      </div>

      <div className={styles.explanation}>
        <h3>Benefits of Synthetic Data for Humanoid Robotics</h3>
        <div className={styles.benefitsGrid}>
          <div className={styles.benefitItem}>
            <div className={styles.benefitIcon}>💰</div>
            <div className={styles.benefitContent}>
              <h4>Cost Reduction</h4>
              <p>Eliminate expensive hardware and data collection campaigns</p>
            </div>
          </div>

          <div className={styles.benefitItem}>
            <div className={styles.benefitIcon}>🛡️</div>
            <div className={styles.benefitContent}>
              <h4>Safety</h4>
              <p>Train AI without risk to physical robots or humans</p>
            </div>
          </div>

          <div className={styles.benefitItem}>
            <div className={styles.benefitIcon}>🌍</div>
            <div className={styles.benefitContent}>
              <h4>Variety</h4>
              <p>Create diverse scenarios and edge cases impossible in real world</p>
            </div>
          </div>

          <div className={styles.benefitItem}>
            <div className={styles.benefitIcon}>✅</div>
            <div className={styles.benefitContent}>
              <h4>Ground Truth</h4>
              <p>Automatic generation of perfectly accurate annotations</p>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default SyntheticDataGenerator;