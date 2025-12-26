import React, { useState, useEffect } from 'react';
import styles from './CapstoneDemo.module.css';

const CapstoneDemo = () => {
  const [currentStep, setCurrentStep] = useState(0);
  const [isRunning, setIsRunning] = useState(false);
  const [demoComplete, setDemoComplete] = useState(false);
  const [robotState, setRobotState] = useState({
    position: { x: 0, y: 0 },
    status: 'idle',
    action: 'Waiting for command'
  });

  const demoSteps = [
    {
      title: "Voice Command Received",
      description: "User says 'Find the red ball and bring it to me'",
      action: "listen",
      robotAction: "Listening for voice command"
    },
    {
      title: "Speech Recognition",
      description: "Converting speech to text: 'find red ball and bring to user'",
      action: "recognize",
      robotAction: "Processing audio input"
    },
    {
      title: "Language Understanding",
      description: "LLM interprets command and creates action plan",
      action: "understand",
      robotAction: "Generating action sequence"
    },
    {
      title: "Environmental Perception",
      description: "Robot scans environment to locate the red ball",
      action: "perceive",
      robotAction: "Detecting objects in environment"
    },
    {
      title: "Action Planning",
      description: "Creating detailed plan: navigate → detect → grasp → return",
      action: "plan",
      robotAction: "Planning navigation path"
    },
    {
      title: "Action Execution",
      description: "Robot executes the planned sequence of actions",
      action: "execute",
      robotAction: "Executing planned actions"
    },
    {
      title: "Task Completion",
      description: "Robot successfully brings the red ball to the user",
      action: "complete",
      robotAction: "Task completed successfully"
    }
  ];

  useEffect(() => {
    if (isRunning && currentStep < demoSteps.length) {
      const timer = setTimeout(() => {
        setRobotState({
          position: { x: currentStep * 10, y: currentStep * 5 }, // Simulate movement
          status: demoSteps[currentStep].action,
          action: demoSteps[currentStep].robotAction
        });
        setCurrentStep(prev => prev + 1);
      }, 2000);

      return () => clearTimeout(timer);
    } else if (currentStep >= demoSteps.length) {
      setDemoComplete(true);
      setIsRunning(false);
    }
  }, [isRunning, currentStep]);

  const startDemo = () => {
    setCurrentStep(0);
    setDemoComplete(false);
    setIsRunning(true);
    setRobotState({
      position: { x: 0, y: 0 },
      status: 'idle',
      action: 'Initializing demo...'
    });
  };

  const resetDemo = () => {
    setCurrentStep(0);
    setDemoComplete(false);
    setIsRunning(false);
    setRobotState({
      position: { x: 0, y: 0 },
      status: 'idle',
      action: 'Waiting for command'
    });
  };

  const getStepStatus = (index) => {
    if (index < currentStep) return 'completed';
    if (index === currentStep) return 'current';
    return 'pending';
  };

  return (
    <div className={styles.container}>
      <h3>Capstone: Autonomous Humanoid Demo</h3>
      <p>Experience the complete VLA (Vision-Language-Action) pipeline in action.</p>

      <div className={styles.demoArea}>
        <div className={styles.robotVisualization}>
          <div className={styles.environment}>
            <div className={styles.robot} style={{
              left: `${50 + robotState.position.x}px`,
              top: `${100 - robotState.position.y}px`
            }}>
              <div className={styles.robotBody}></div>
              <div className={styles.robotHead}></div>
              <div className={styles.robotArmLeft}></div>
              <div className={styles.robotArmRight}></div>
            </div>

            {currentStep > 3 && (
              <div className={`${styles.object} ${styles.ball}`} style={{
                left: '200px',
                top: '150px'
              }}></div>
            )}

            <div className={styles.user} style={{
              left: '300px',
              top: '180px'
            }}></div>
          </div>

          <div className={styles.robotStatus}>
            <div><strong>Status:</strong> {robotState.status}</div>
            <div><strong>Action:</strong> {robotState.action}</div>
            <div><strong>Position:</strong> ({robotState.position.x}, {robotState.position.y})</div>
          </div>
        </div>

        <div className={styles.controls}>
          <button
            onClick={isRunning ? null : startDemo}
            disabled={isRunning}
            className={`${styles.startButton} ${isRunning ? styles.running : ''}`}
          >
            {isRunning ? 'Demo Running...' : '▶️ Start Capstone Demo'}
          </button>

          <button
            onClick={resetDemo}
            className={styles.resetButton}
            disabled={isRunning}
          >
            Reset Demo
          </button>
        </div>
      </div>

      <div className={styles.timeline}>
        <h4>Demo Timeline:</h4>
        <div className={styles.stepsContainer}>
          {demoSteps.map((step, index) => (
            <div
              key={index}
              className={`${styles.step} ${styles[getStepStatus(index)]}`}
            >
              <div className={styles.stepHeader}>
                <div className={styles.stepNumber}>{index + 1}</div>
                <div className={styles.stepTitle}>{step.title}</div>
                <div className={styles.stepStatus}>
                  {getStepStatus(index) === 'completed' && '✓'}
                  {getStepStatus(index) === 'current' && '▶'}
                  {getStepStatus(index) === 'pending' && '○'}
                </div>
              </div>
              <div className={styles.stepDescription}>
                {step.description}
              </div>
              {currentStep > index && (
                <div className={styles.stepResult}>
                  <strong>Result:</strong> {step.robotAction}
                </div>
              )}
            </div>
          ))}
        </div>
      </div>

      {demoComplete && (
        <div className={styles.completionMessage}>
          <h4>🎉 Demo Complete!</h4>
          <p>The autonomous humanoid successfully completed the task using the complete VLA pipeline.</p>
          <p>Summary: Voice input was processed through speech recognition, language understanding with LLMs, environmental perception, and action execution.</p>
        </div>
      )}

      <div className={styles.explanation}>
        <h4>Complete VLA Architecture:</h4>
        <ol>
          <li><strong>Vision System:</strong> Environmental perception and object recognition</li>
          <li><strong>Language System:</strong> Natural language understanding and task planning</li>
          <li><strong>Action System:</strong> Low-level robot control and execution</li>
          <li><strong>Integration:</strong> Seamless coordination between all components</li>
          <li><strong>Safety:</strong> Validation and constraint enforcement throughout</li>
        </ol>
      </div>
    </div>
  );
};

export default CapstoneDemo;