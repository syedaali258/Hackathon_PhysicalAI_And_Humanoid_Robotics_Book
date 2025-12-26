import React, { useState } from 'react';
import styles from './LLMPlannerDemo.module.css';

const LLMPlannerDemo = () => {
  const [command, setCommand] = useState('');
  const [isProcessing, setIsProcessing] = useState(false);
  const [plan, setPlan] = useState(null);
  const [robotActions, setRobotActions] = useState([]);

  const sampleCommands = [
    "Navigate to the kitchen and find a red cup",
    "Go to the living room and turn on the light",
    "Move to the table and pick up the book",
    "Find the blue ball and bring it to me"
  ];

  const generatePlan = async () => {
    if (!command.trim()) return;

    setIsProcessing(true);
    setPlan(null);
    setRobotActions([]);

    // Simulate API call delay
    setTimeout(() => {
      // Mock planning logic
      const mockPlans = {
        "navigate to the kitchen and find a red cup": {
          interpreted_meaning: "User wants robot to navigate to kitchen and locate a red cup",
          action_sequence: [
            { action: "navigate_to", parameters: { target: "kitchen" }, description: "Navigate to kitchen area" },
            { action: "detect_object", parameters: { type: "cup", color: "red" }, description: "Look for red cup" },
            { action: "approach_object", parameters: { object: "red_cup" }, description: "Move closer to cup" }
          ]
        },
        "go to the living room and turn on the light": {
          interpreted_meaning: "User wants robot to go to living room and activate the light",
          action_sequence: [
            { action: "navigate_to", parameters: { target: "living_room" }, description: "Navigate to living room" },
            { action: "locate_light_switch", parameters: {}, description: "Find the light switch" },
            { action: "activate_switch", parameters: { switch_id: "main_light" }, description: "Turn on the light" }
          ]
        },
        "move to the table and pick up the book": {
          interpreted_meaning: "User wants robot to go to table and grasp the book",
          action_sequence: [
            { action: "navigate_to", parameters: { target: "table" }, description: "Navigate to table" },
            { action: "detect_object", parameters: { type: "book" }, description: "Locate the book" },
            { action: "grasp_object", parameters: { object: "book" }, description: "Pick up the book" }
          ]
        },
        "find the blue ball and bring it to me": {
          interpreted_meaning: "User wants robot to locate blue ball and deliver it",
          action_sequence: [
            { action: "detect_object", parameters: { type: "ball", color: "blue" }, description: "Search for blue ball" },
            { action: "navigate_to", parameters: { target: "ball_location" }, description: "Go to ball location" },
            { action: "grasp_object", parameters: { object: "blue_ball" }, description: "Pick up the ball" },
            { action: "navigate_to", parameters: { target: "user" }, description: "Return to user" }
          ]
        }
      };

      const selectedPlan = mockPlans[command.toLowerCase()] || {
        interpreted_meaning: `Planning to execute command: "${command}"`,
        action_sequence: [
          { action: "interpret_command", parameters: { cmd: command }, description: `Processing command: ${command}` },
          { action: "plan_actions", parameters: {}, description: "Generating action sequence" },
          { action: "execute_plan", parameters: {}, description: "Executing planned actions" }
        ]
      };

      setPlan(selectedPlan);
      setRobotActions(selectedPlan.action_sequence);
      setIsProcessing(false);
    }, 2000);
  };

  const handleSampleCommand = (sampleCommand) => {
    setCommand(sampleCommand);
  };

  return (
    <div className={styles.container}>
      <h3>LLM-Based Planning Demo</h3>
      <p>Enter a natural language command and see how the LLM interprets it and generates an action plan.</p>

      <div className={styles.inputSection}>
        <label htmlFor="commandInput">
          <strong>Enter a command:</strong>
        </label>
        <div className={styles.inputGroup}>
          <input
            id="commandInput"
            type="text"
            value={command}
            onChange={(e) => setCommand(e.target.value)}
            placeholder="e.g., 'Navigate to the kitchen and find a red cup'"
            className={styles.commandInput}
          />
          <button
            onClick={generatePlan}
            disabled={isProcessing || !command.trim()}
            className={styles.generateButton}
          >
            {isProcessing ? 'Processing...' : 'Generate Plan'}
          </button>
        </div>

        <div className={styles.sampleCommands}>
          <p><strong>Try these sample commands:</strong></p>
          <div className={styles.commandButtons}>
            {sampleCommands.map((sample, index) => (
              <button
                key={index}
                onClick={() => handleSampleCommand(sample)}
                className={styles.sampleButton}
                disabled={isProcessing}
              >
                {sample}
              </button>
            ))}
          </div>
        </div>
      </div>

      {isProcessing && (
        <div className={styles.processingIndicator}>
          <div className={styles.spinner}></div>
          <p>Processing with LLM...</p>
        </div>
      )}

      {plan && (
        <div className={styles.resultSection}>
          <h4>Generated Plan</h4>

          <div className={styles.interpretation}>
            <strong>Interpretation:</strong> {plan.interpreted_meaning}
          </div>

          <div className={styles.actionSequence}>
            <h5>Action Sequence:</h5>
            <ol className={styles.actionList}>
              {robotActions.map((action, index) => (
                <li key={index} className={styles.actionItem}>
                  <div className={styles.actionHeader}>
                    <span className={styles.actionType}>{action.action}</span>
                    <span className={styles.actionDescription}>{action.description}</span>
                  </div>
                  {action.parameters && Object.keys(action.parameters).length > 0 && (
                    <div className={styles.actionParams}>
                      <strong>Parameters:</strong> {JSON.stringify(action.parameters)}
                    </div>
                  )}
                </li>
              ))}
            </ol>
          </div>
        </div>
      )}

      <div className={styles.explanation}>
        <h4>How LLM-Based Planning Works:</h4>
        <ol>
          <li><strong>Command Input:</strong> Natural language command from user</li>
          <li><strong>LLM Processing:</strong> Large Language Model interprets the command</li>
          <li><strong>Plan Generation:</strong> LLM creates a sequence of robot actions</li>
          <li><strong>Validation:</strong> Plan is validated for safety and feasibility</li>
          <li><strong>Execution:</strong> Robot executes the planned action sequence</li>
        </ol>
      </div>
    </div>
  );
};

export default LLMPlannerDemo;