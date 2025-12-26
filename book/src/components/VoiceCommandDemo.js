import React, { useState, useEffect } from 'react';
import styles from './VoiceCommandDemo.module.css';

const VoiceCommandDemo = () => {
  const [isListening, setIsListening] = useState(false);
  const [transcript, setTranscript] = useState('');
  const [commandResult, setCommandResult] = useState('');
  const [robotStatus, setRobotStatus] = useState('idle');
  const [isProcessing, setIsProcessing] = useState(false);

  // Mock recognition function - in a real implementation, this would use the Web Speech API
  const startListening = async () => {
    setIsListening(true);
    setTranscript('');
    setCommandResult('');

    // Simulate voice recognition
    setTimeout(() => {
      // In a real implementation, this would come from actual speech recognition
      const mockCommands = [
        "move forward",
        "turn left",
        "turn right",
        "raise left arm",
        "stop",
        "navigate to kitchen"
      ];

      const randomCommand = mockCommands[Math.floor(Math.random() * mockCommands.length)];
      setTranscript(randomCommand);
      setIsListening(false);
      processCommand(randomCommand);
    }, 2000); // 2 second simulated recognition time
  };

  const processCommand = async (command) => {
    setIsProcessing(true);
    setRobotStatus('processing');

    // Simulate sending command to backend and getting response
    setTimeout(() => {
      const actions = {
        "move forward": "Robot moving forward 1 meter",
        "turn left": "Robot turning left 90 degrees",
        "turn right": "Robot turning right 90 degrees",
        "raise left arm": "Robot raising left arm",
        "stop": "Robot stopping all movement",
        "navigate to kitchen": "Robot navigating to kitchen area"
      };

      const result = actions[command] || `Robot executing: ${command}`;
      setCommandResult(result);
      setRobotStatus('executing');

      // Simulate action completion
      setTimeout(() => {
        setRobotStatus('completed');
        setIsProcessing(false);
      }, 1500);
    }, 1000);
  };

  const resetDemo = () => {
    setTranscript('');
    setCommandResult('');
    setRobotStatus('idle');
    setIsProcessing(false);
  };

  return (
    <div className={styles.container}>
      <h3>Voice Command Demo</h3>
      <p>Click the button below to simulate speaking a command to the robot.</p>

      <div className={styles.demoArea}>
        <div className={styles.robotVisualization}>
          <div className={`${styles.robot} ${styles[robotStatus]}`}>
            <div className={styles.robotBody}></div>
            <div className={styles.robotHead}></div>
            <div className={styles.robotArmLeft}></div>
            <div className={styles.robotArmRight}></div>
          </div>
          <div className={styles.robotStatus}>
            Status: <span className={styles.statusText}>{robotStatus}</span>
          </div>
        </div>

        <div className={styles.controls}>
          <button
            onClick={isListening ? null : startListening}
            disabled={isListening || isProcessing}
            className={`${styles.listenButton} ${isListening ? styles.listening : ''}`}
          >
            {isListening ? (
              <span className={styles.listeningIndicator}>
                <span className={styles.dot}></span>
                Listening...
              </span>
            ) : (
              '🎤 Start Voice Command'
            )}
          </button>

          <button
            onClick={resetDemo}
            className={styles.resetButton}
            disabled={isProcessing}
          >
            Reset Demo
          </button>
        </div>

        <div className={styles.commandOutput}>
          {transcript && (
            <div className={styles.transcript}>
              <strong>Recognized Command:</strong> "{transcript}"
            </div>
          )}

          {commandResult && (
            <div className={styles.result}>
              <strong>Robot Action:</strong> {commandResult}
            </div>
          )}

          {isProcessing && (
            <div className={styles.processing}>
              Processing command...
            </div>
          )}
        </div>
      </div>

      <div className={styles.explanation}>
        <h4>How Voice-to-Action Works:</h4>
        <ol>
          <li><strong>Voice Input:</strong> User speaks a command to the robot</li>
          <li><strong>Speech Recognition:</strong> Converts speech to text</li>
          <li><strong>Command Interpretation:</strong> Understands the intent</li>
          <li><strong>Action Mapping:</strong> Maps command to robot action</li>
          <li><strong>Execution:</strong> Robot performs the action in simulation</li>
        </ol>
      </div>
    </div>
  );
};

export default VoiceCommandDemo;