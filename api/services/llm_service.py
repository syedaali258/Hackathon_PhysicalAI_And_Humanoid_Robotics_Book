"""
LLM Service for Language-Based Planning in VLA Module
"""
import asyncio
import logging
from typing import Dict, List, Any, Optional
from datetime import datetime
import os

# Import OpenAI library
try:
    from openai import AsyncOpenAI
except ImportError:
    AsyncOpenAI = None

logger = logging.getLogger(__name__)


class LLMService:
    """
    Service for handling LLM-based language planning
    """

    def __init__(self):
        # Initialize OpenAI client if available
        api_key = os.getenv("OPENAI_API_KEY")
        model = os.getenv("OPENAI_MODEL", "gpt-4-turbo")

        if AsyncOpenAI and api_key:
            self.client = AsyncOpenAI(api_key=api_key)
            self.model = model
        else:
            self.client = None
            logger.warning("OpenAI client not available, using mock responses")

        # Default timeout for LLM requests
        self.timeout = float(os.getenv("VLA_PLAN_TIMEOUT", "30.0"))

    async def generate_action_plan(self, command: str, context: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """
        Generate an action plan based on the natural language command

        Args:
            command: Natural language command from user
            context: Additional context information (environment, robot state, etc.)

        Returns:
            Dictionary containing the action plan
        """
        try:
            if not self.client:
                # Return mock response if OpenAI is not available
                return await self._generate_mock_plan(command, context)

            # Prepare the prompt for the LLM
            prompt = self._create_planning_prompt(command, context)

            # Call the LLM
            response = await self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": self._get_system_prompt()},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.3,  # Lower temperature for more consistent planning
                max_tokens=1000,
                timeout=self.timeout
            )

            # Extract and parse the response
            plan_text = response.choices[0].message.content

            # Parse the LLM response into structured action plan
            action_plan = self._parse_plan_response(plan_text, command)

            logger.info(f"Generated action plan for command: '{command[:50]}...'")

            return action_plan

        except Exception as e:
            logger.error(f"Error generating action plan: {str(e)}")
            # Return a basic plan in case of error
            return {
                "command": command,
                "interpreted_meaning": f"Could not interpret command: {command}",
                "action_sequence": [],
                "confidence": 0.0,
                "error": str(e)
            }

    def _create_planning_prompt(self, command: str, context: Optional[Dict[str, Any]] = None) -> str:
        """
        Create a detailed prompt for the LLM to generate action plans
        """
        prompt = f"""
        You are an AI assistant that converts natural language commands into robot action sequences.
        The robot operates in a simulation environment and can perform various physical actions.

        Command: "{command}"

        Convert this command into a sequence of specific robot actions.
        Each action should be represented as a dictionary with:
        - "action": the type of action (e.g., "move_forward", "turn_left", "raise_arm", "grasp_object")
        - "parameters": specific parameters for the action

        The action sequence should be logical and executable in order.
        Consider the physical constraints of a humanoid robot.

        Respond with a JSON object containing:
        - "interpreted_meaning": your interpretation of what the user wants
        - "action_sequence": list of action dictionaries
        - "estimated_duration": estimated time in seconds to complete the sequence

        Example response format:
        {{
            "interpreted_meaning": "User wants the robot to move forward and then turn left",
            "action_sequence": [
                {{"action": "move_forward", "parameters": {{"distance": 1.0, "speed": 0.5}}}},
                {{"action": "turn_left", "parameters": {{"angle": 90.0}}}}
            ],
            "estimated_duration": 10.5
        }}
        """

        if context:
            prompt += f"\n\nAdditional context: {context}"

        return prompt

    def _get_system_prompt(self) -> str:
        """
        Get the system prompt that defines the LLM's role and constraints
        """
        return """
        You are an AI planning assistant for a humanoid robot operating in a simulation environment.
        Your role is to interpret natural language commands and convert them into sequences of robot actions.
        Be precise, logical, and consider the physical constraints of humanoid robots.
        Always respond with properly formatted JSON.
        """

    def _parse_plan_response(self, response_text: str, original_command: str) -> Dict[str, Any]:
        """
        Parse the LLM response into a structured action plan
        """
        try:
            import json
            # Try to extract JSON from the response
            # This is a simplified approach - in production, use proper JSON parsing
            start_idx = response_text.find('{')
            end_idx = response_text.rfind('}') + 1

            if start_idx != -1 and end_idx != 0:
                json_str = response_text[start_idx:end_idx]
                plan = json.loads(json_str)

                # Add metadata
                plan["original_command"] = original_command
                plan["generated_at"] = datetime.now().isoformat()
                plan["confidence"] = 0.9  # Default high confidence for successful parsing

                return plan
            else:
                # If no JSON found, create a basic plan
                return {
                    "original_command": original_command,
                    "interpreted_meaning": f"Could not parse LLM response for: {original_command}",
                    "action_sequence": [],
                    "estimated_duration": 0,
                    "confidence": 0.1,
                    "raw_response": response_text
                }
        except Exception as e:
            logger.error(f"Error parsing LLM response: {str(e)}")
            return {
                "original_command": original_command,
                "interpreted_meaning": f"Error parsing LLM response: {str(e)}",
                "action_sequence": [],
                "estimated_duration": 0,
                "confidence": 0.0,
                "error": str(e)
            }

    async def _generate_mock_plan(self, command: str, context: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """
        Generate a mock action plan when OpenAI is not available
        """
        await asyncio.sleep(0.2)  # Simulate processing time

        # Define some common command patterns and their corresponding actions
        command_patterns = {
            "move": [{"action": "move_forward", "parameters": {"distance": 1.0, "speed": 0.5}}],
            "forward": [{"action": "move_forward", "parameters": {"distance": 1.0, "speed": 0.5}}],
            "backward": [{"action": "move_backward", "parameters": {"distance": 1.0, "speed": 0.5}}],
            "left": [{"action": "turn_left", "parameters": {"angle": 90.0}}],
            "right": [{"action": "turn_right", "parameters": {"angle": 90.0}}],
            "turn": [{"action": "turn_left", "parameters": {"angle": 90.0}}],
            "raise": [{"action": "raise_arm", "parameters": {"arm": "left", "angle": 45.0}}],
            "lower": [{"action": "lower_arm", "parameters": {"arm": "right", "angle": 45.0}}],
            "grasp": [{"action": "grasp_object", "parameters": {"object_id": "unknown"}}],
            "pick": [{"action": "grasp_object", "parameters": {"object_id": "unknown"}}],
            "stop": [{"action": "stop", "parameters": {}}],
            "navigate": [{"action": "navigate_to", "parameters": {"target": "kitchen"}}],
            "find": [{"action": "detect_object", "parameters": {"object_type": "unknown"}}]
        }

        # Find matching actions based on command keywords
        action_sequence = []
        command_lower = command.lower()

        for keyword, actions in command_patterns.items():
            if keyword in command_lower:
                action_sequence.extend(actions)
                break  # For simplicity, just use first match

        # If no keywords matched, create a default action
        if not action_sequence:
            action_sequence = [{"action": "unknown_command", "parameters": {"command": command}}]

        return {
            "original_command": command,
            "interpreted_meaning": f"Interpreted command: {command}",
            "action_sequence": action_sequence,
            "estimated_duration": len(action_sequence) * 2,  # 2 seconds per action
            "confidence": 0.8,
            "generated_at": datetime.now().isoformat(),
            "mock_response": True
        }

    async def validate_plan(self, action_plan: Dict[str, Any]) -> Dict[str, Any]:
        """
        Validate an action plan for safety and feasibility

        Args:
            action_plan: The action plan to validate

        Returns:
            Dictionary with validation results
        """
        try:
            action_sequence = action_plan.get("action_sequence", [])
            issues = []

            for i, action in enumerate(action_sequence):
                action_type = action.get("action")

                # Basic validation rules
                if action_type in ["move_forward", "move_backward"]:
                    distance = action.get("parameters", {}).get("distance", 0)
                    if distance > 10:  # Max 10m movement
                        issues.append(f"Action {i}: Movement distance too large: {distance}m")

                elif action_type in ["turn_left", "turn_right"]:
                    angle = action.get("parameters", {}).get("angle", 0)
                    if angle > 180:  # Max 180 degree turn
                        issues.append(f"Action {i}: Turn angle too large: {angle} degrees")

            return {
                "is_valid": len(issues) == 0,
                "issues": issues,
                "action_count": len(action_sequence)
            }
        except Exception as e:
            logger.error(f"Error validating plan: {str(e)}")
            return {
                "is_valid": False,
                "issues": [f"Validation error: {str(e)}"],
                "action_count": 0
            }


# Singleton instance with error handling
try:
    llm_service = LLMService()
except (ValueError, TypeError) as e:
    import logging
    logger = logging.getLogger(__name__)
    logger.warning(f"Could not initialize LLM service: {e}. Using fallback functionality.")
    llm_service = None