import React, { useState, useEffect } from 'react';
import styles from './URDFViewer.module.css';

const URDFViewer = () => {
  const [selectedRobot, setSelectedRobot] = useState('simple_humanoid');
  const [jointValues, setJointValues] = useState({});
  const [visualizationMode, setVisualizationMode] = useState('3d'); // '3d', 'tree', 'properties'

  // Define a simple humanoid robot structure for visualization
  const robotModels = {
    simple_humanoid: {
      name: 'Simple Humanoid',
      links: [
        { id: 'torso', name: 'Torso', type: 'box', size: [0.3, 0.2, 0.5], position: [0, 0, 0.25], color: '#888888' },
        { id: 'head', name: 'Head', type: 'sphere', radius: 0.1, position: [0, 0, 0.6], color: '#FFD7CC' },
        { id: 'left_upper_arm', name: 'Left Upper Arm', type: 'cylinder', size: [0.05, 0.3], position: [0.15, -0.1, 0.25], rotation: [0, 0, -Math.PI/2], color: '#888888' },
        { id: 'left_lower_arm', name: 'Left Lower Arm', type: 'cylinder', size: [0.04, 0.25], position: [0.15, -0.1, -0.05], rotation: [0, 0, -Math.PI/2], color: '#888888' },
        { id: 'right_upper_arm', name: 'Right Upper Arm', type: 'cylinder', size: [0.05, 0.3], position: [0.15, 0.1, 0.25], rotation: [0, 0, Math.PI/2], color: '#888888' },
        { id: 'right_lower_arm', name: 'Right Lower Arm', type: 'cylinder', size: [0.04, 0.25], position: [0.15, 0.1, -0.05], rotation: [0, 0, Math.PI/2], color: '#888888' },
        { id: 'left_upper_leg', name: 'Left Upper Leg', type: 'cylinder', size: [0.06, 0.4], position: [-0.05, -0.05, -0.2], rotation: [Math.PI/2, 0, 0], color: '#888888' },
        { id: 'left_lower_leg', name: 'Left Lower Leg', type: 'cylinder', size: [0.05, 0.35], position: [-0.05, -0.05, -0.55], rotation: [Math.PI/2, 0, 0], color: '#888888' },
        { id: 'right_upper_leg', name: 'Right Upper Leg', type: 'cylinder', size: [0.06, 0.4], position: [-0.05, 0.05, -0.2], rotation: [Math.PI/2, 0, 0], color: '#888888' },
        { id: 'right_lower_leg', name: 'Right Lower Leg', type: 'cylinder', size: [0.05, 0.35], position: [-0.05, 0.05, -0.55], rotation: [Math.PI/2, 0, 0], color: '#888888' }
      ],
      joints: [
        { id: 'torso_to_head', name: 'Neck Joint', parent: 'torso', child: 'head', type: 'fixed', limits: null },
        { id: 'torso_to_left_shoulder', name: 'Left Shoulder', parent: 'torso', child: 'left_upper_arm', type: 'revolute', limits: { lower: -1.57, upper: 1.57 } },
        { id: 'left_shoulder_to_elbow', name: 'Left Elbow', parent: 'left_upper_arm', child: 'left_lower_arm', type: 'revolute', limits: { lower: -1.57, upper: 1.57 } },
        { id: 'torso_to_right_shoulder', name: 'Right Shoulder', parent: 'torso', child: 'right_upper_arm', type: 'revolute', limits: { lower: -1.57, upper: 1.57 } },
        { id: 'right_shoulder_to_elbow', name: 'Right Elbow', parent: 'right_upper_arm', child: 'right_lower_arm', type: 'revolute', limits: { lower: -1.57, upper: 1.57 } },
        { id: 'torso_to_left_hip', name: 'Left Hip', parent: 'torso', child: 'left_upper_leg', type: 'revolute', limits: { lower: -1.57, upper: 1.57 } },
        { id: 'left_hip_to_knee', name: 'Left Knee', parent: 'left_upper_leg', child: 'left_lower_leg', type: 'revolute', limits: { lower: -1.57, upper: 0 } },
        { id: 'torso_to_right_hip', name: 'Right Hip', parent: 'torso', child: 'right_upper_leg', type: 'revolute', limits: { lower: -1.57, upper: 1.57 } },
        { id: 'right_hip_to_knee', name: 'Right Knee', parent: 'right_upper_leg', child: 'right_lower_leg', type: 'revolute', limits: { lower: -1.57, upper: 0 } }
      ]
    }
  };

  // Initialize joint values
  useEffect(() => {
    const initialJointValues = {};
    robotModels[selectedRobot].joints.forEach(joint => {
      if (joint.limits) {
        initialJointValues[joint.id] = (joint.limits.lower + joint.limits.upper) / 2;
      }
    });
    setJointValues(initialJointValues);
  }, [selectedRobot]);

  const handleJointChange = (jointId, value) => {
    setJointValues(prev => ({
      ...prev,
      [jointId]: parseFloat(value)
    }));
  };

  const resetJoints = () => {
    const resetValues = {};
    robotModels[selectedRobot].joints.forEach(joint => {
      if (joint.limits) {
        resetValues[joint.id] = (joint.limits.lower + joint.limits.upper) / 2;
      }
    });
    setJointValues(resetValues);
  };

  const currentRobot = robotModels[selectedRobot];

  return (
    <div className={styles.container}>
      <div className={styles.header}>
        <h3>URDF Robot Model Viewer</h3>
        <div className={styles.modelSelector}>
          <label>Robot Model:</label>
          <select
            value={selectedRobot}
            onChange={(e) => setSelectedRobot(e.target.value)}
            className={styles.select}
          >
            {Object.entries(robotModels).map(([key, model]) => (
              <option key={key} value={key}>{model.name}</option>
            ))}
          </select>
        </div>
      </div>

      <div className={styles.visualizationPanel}>
        <div className={styles.modeSelector}>
          <button
            className={`${styles.modeButton} ${visualizationMode === '3d' ? styles.active : ''}`}
            onClick={() => setVisualizationMode('3d')}
          >
            3D View
          </button>
          <button
            className={`${styles.modeButton} ${visualizationMode === 'tree' ? styles.active : ''}`}
            onClick={() => setVisualizationMode('tree')}
          >
            Kinematic Tree
          </button>
          <button
            className={`${styles.modeButton} ${visualizationMode === 'properties' ? styles.active : ''}`}
            onClick={() => setVisualizationMode('properties')}
          >
            Properties
          </button>
        </div>

        {visualizationMode === '3d' && (
          <div className={styles.viewer3d}>
            <div className={styles.robotCanvas}>
              {/* Simplified 3D representation using CSS */}
              <div className={styles.robotContainer}>
                {currentRobot.links.map((link, index) => (
                  <div
                    key={link.id}
                    className={styles.link}
                    style={{
                      position: 'absolute',
                      left: `${50 + link.position[0] * 100}%`,
                      top: `${50 - link.position[1] * 100}%`,
                      width: link.type === 'sphere' ? `${link.radius * 200}px` :
                             link.type === 'cylinder' ? `${link.size[0] * 200}px` :
                             `${link.size[0] * 200}px`,
                      height: link.type === 'sphere' ? `${link.radius * 200}px` :
                              link.type === 'cylinder' ? `${link.size[1] * 200}px` :
                              `${link.size[2] * 200}px`,
                      backgroundColor: link.color,
                      borderRadius: link.type === 'sphere' ? '50%' : '4px',
                      transform: `translate(-50%, -50%) rotate(${(link.rotation ? link.rotation[2] : 0) * 180 / Math.PI}deg)`,
                      border: '1px solid #333',
                      display: 'flex',
                      alignItems: 'center',
                      justifyContent: 'center',
                      fontSize: '10px',
                      color: '#333',
                      fontWeight: 'bold'
                    }}
                  >
                    {link.name.split(' ')[0]}
                  </div>
                ))}
              </div>
            </div>
          </div>
        )}

        {visualizationMode === 'tree' && (
          <div className={styles.kinematicTree}>
            <h4>Kinematic Structure</h4>
            <div className={styles.treeContainer}>
              <div className={styles.treeNode}>
                <div className={styles.linkNode}>{currentRobot.links[0].name}</div>
                <div className={styles.treeChildren}>
                  {currentRobot.joints
                    .filter(joint => joint.parent === currentRobot.links[0].id)
                    .map(joint => {
                      const childLink = currentRobot.links.find(l => l.id === joint.child);
                      return (
                        <div key={joint.id} className={styles.treeBranch}>
                          <div className={styles.jointNode}>
                            {joint.name} ({joint.type})
                          </div>
                          <div className={styles.linkNode}>{childLink?.name}</div>
                        </div>
                      );
                    })}
                </div>
              </div>
            </div>
          </div>
        )}

        {visualizationMode === 'properties' && (
          <div className={styles.propertiesPanel}>
            <h4>Robot Properties</h4>
            <div className={styles.propertiesGrid}>
              <div className={styles.propertyGroup}>
                <h5>Links ({currentRobot.links.length})</h5>
                <ul className={styles.propertyList}>
                  {currentRobot.links.map(link => (
                    <li key={link.id} className={styles.propertyItem}>
                      <strong>{link.name}</strong> ({link.type})
                    </li>
                  ))}
                </ul>
              </div>
              <div className={styles.propertyGroup}>
                <h5>Joints ({currentRobot.joints.length})</h5>
                <ul className={styles.propertyList}>
                  {currentRobot.joints.map(joint => (
                    <li key={joint.id} className={styles.propertyItem}>
                      <strong>{joint.name}</strong> ({joint.type})
                      {joint.limits && (
                        <span className={styles.limits}>
                          [{joint.limits.lower}, {joint.limits.upper}]
                        </span>
                      )}
                    </li>
                  ))}
                </ul>
              </div>
            </div>
          </div>
        )}
      </div>

      <div className={styles.controlsPanel}>
        <div className={styles.controlsHeader}>
          <h4>Joint Controls</h4>
          <button className={styles.resetButton} onClick={resetJoints}>
            Reset Joints
          </button>
        </div>

        <div className={styles.jointControls}>
          {currentRobot.joints
            .filter(joint => joint.limits) // Only show joints with limits (movable)
            .map(joint => (
              <div key={joint.id} className={styles.jointControl}>
                <label>
                  {joint.name} ({joint.id}):
                </label>
                <div className={styles.sliderContainer}>
                  <input
                    type="range"
                    min={joint.limits.lower}
                    max={joint.limits.upper}
                    step="0.01"
                    value={jointValues[joint.id] || 0}
                    onChange={(e) => handleJointChange(joint.id, e.target.value)}
                    className={styles.slider}
                  />
                  <span className={styles.valueDisplay}>
                    {(jointValues[joint.id] || 0).toFixed(2)} rad
                  </span>
                </div>
              </div>
            ))}
        </div>
      </div>

      <div className={styles.urdfCode}>
        <h4>URDF XML Preview</h4>
        <pre className={styles.codeBlock}>
          <code>{`<!-- Simplified URDF for ${currentRobot.name} -->
<robot name="${selectedRobot}">
  <!-- Links -->
  ${currentRobot.links.map(link =>
    `  <link name="${link.id}">...</link>`
  ).join('\n  ')}

  <!-- Joints -->
  ${currentRobot.joints.map(joint =>
    `  <joint name="${joint.id}" type="${joint.type}">
    <parent link="${joint.parent}"/>
    <child link="${joint.child}"/>
  </joint>`
  ).join('\n  ')}

</robot>`}</code>
        </pre>
      </div>
    </div>
  );
};

export default URDFViewer;