"""
Language-Based Planning Service for Vision-Language-Action (VLA) Module

This service uses LLMs to generate detailed action plans from natural language commands.
"""
import asyncio
import logging
from typing import List, Dict, Any, Optional
from dataclasses import dataclass
from enum import Enum
import json

from openai import AsyncOpenAI
from pydantic import BaseModel, Field

from .voice_to_action import ActionCommand, ActionType

logger = logging.getLogger(__name__)


class PlanStep(BaseModel):
    """Represents a single step in an action plan"""
    action_type: ActionType
    parameters: Dict[str, Any] = Field(default_factory=dict)
    description: str
    estimated_duration: float = 1.0  # in seconds


class ActionPlan(BaseModel):
    """Represents a complete action plan"""
    steps: List[PlanStep]
    description: str
    estimated_total_duration: float = 0.0


class WorldState(BaseModel):
    """Represents the current state of the world/environment"""
    robot_position: Dict[str, float] = Field(default_factory=lambda: {"x": 0.0, "y": 0.0, "z": 0.0})
    robot_orientation: Dict[str, float] = Field(default_factory=lambda: {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0})
    objects: List[Dict[str, Any]] = Field(default_factory=list)
    locations: List[Dict[str, Any]] = Field(default_factory=list)
    available_actions: List[ActionType] = Field(default_factory=list)


class LanguagePlanningService:
    """Service for generating action plans using LLMs"""

    def __init__(self, openai_api_key: Optional[str] = None):
        if openai_api_key:
            self.client = AsyncOpenAI(api_key=openai_api_key)
        else:
            # Use a mock client for testing
            self.client = None
        self.model = "gpt-4o-mini"  # Default model

    async def generate_plan_from_command(self, command_text: str, world_state: WorldState) -> ActionPlan:
        """
        Generate an action plan from a natural language command using LLM

        Args:
            command_text: The natural language command
            world_state: Current state of the world/environment

        Returns:
            ActionPlan representing the sequence of actions to execute
        """
        logger.info(f"Generating plan for command: {command_text}")

        if self.client is not None:
            # Use real LLM for planning
            return await self._generate_plan_with_llm(command_text, world_state)
        else:
            # Use mock implementation for testing
            return await self._generate_plan_mock(command_text, world_state)

    async def _generate_plan_with_llm(self, command_text: str, world_state: WorldState) -> ActionPlan:
        """Generate plan using real LLM"""
        prompt = self._create_planning_prompt(command_text, world_state)

        try:
            response = await self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": self._get_system_prompt()},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.1,
                response_format={"type": "json_object"}
            )

            # Parse the response
            plan_data = json.loads(response.choices[0].message.content)
            return self._parse_plan_response(plan_data)

        except Exception as e:
            logger.error(f"Error generating plan with LLM: {e}")
            # Fall back to mock implementation
            return await self._generate_plan_mock(command_text, world_state)

    async def _generate_plan_mock(self, command_text: str, world_state: WorldState) -> ActionPlan:
        """Generate plan using mock implementation (for testing without API key)"""
        logger.warning("Using mock planning implementation - no API key provided")

        # Simple rule-based planning for demonstration
        steps = []

        # Convert command to lower case for processing
        lower_command = command_text.lower()

        if 'navigate' in lower_command or 'go to' in lower_command or 'move to' in lower_command:
            # Extract location
            location = 'kitchen'  # Default for demo
            if 'kitchen' in lower_command:
                location = 'kitchen'
            elif 'office' in lower_command:
                location = 'office'
            elif 'living room' in lower_command:
                location = 'living_room'

            steps.append(PlanStep(
                action_type=ActionType.MOVE_TO_LOCATION,
                parameters={'location': location},
                description=f"Navigate to {location.replace('_', ' ')}",
                estimated_duration=5.0
            ))

        elif 'pick up' in lower_command or 'grasp' in lower_command or 'take' in lower_command:
            # Extract object
            obj = 'object'  # Default for demo
            if 'cup' in lower_command:
                obj = 'cup'
            elif 'ball' in lower_command:
                obj = 'ball'
            elif 'book' in lower_command:
                obj = 'book'

            steps.append(PlanStep(
                action_type=ActionType.PICK_UP_OBJECT,
                parameters={'object': obj},
                description=f"Pick up the {obj}",
                estimated_duration=3.0
            ))

        elif 'place' in lower_command or 'put' in lower_command or 'set' in lower_command:
            # Extract object and location
            obj = 'object'  # Default for demo
            location = 'table'  # Default for demo

            if 'cup' in lower_command:
                obj = 'cup'
            elif 'ball' in lower_command:
                obj = 'ball'

            if 'table' in lower_command:
                location = 'table'
            elif 'box' in lower_command:
                location = 'box'

            steps.append(PlanStep(
                action_type=ActionType.PLACE_OBJECT,
                parameters={'object': obj, 'location': location},
                description=f"Place the {obj} on the {location}",
                estimated_duration=3.0
            ))

        elif 'move' in lower_command or 'go' in lower_command:
            # Simple movement command
            if 'forward' in lower_command:
                steps.append(PlanStep(
                    action_type=ActionType.MOVE_FORWARD,
                    parameters={'distance': 1.0},
                    description="Move forward 1 meter",
                    estimated_duration=2.0
                ))
            elif 'backward' in lower_command:
                steps.append(PlanStep(
                    action_type=ActionType.MOVE_BACKWARD,
                    parameters={'distance': 1.0},
                    description="Move backward 1 meter",
                    estimated_duration=2.0
                ))
            else:
                # Default movement
                steps.append(PlanStep(
                    action_type=ActionType.MOVE_FORWARD,
                    parameters={'distance': 1.0},
                    description="Move forward 1 meter",
                    estimated_duration=2.0
                ))

        else:
            # Default to stop if command is unclear
            steps.append(PlanStep(
                action_type=ActionType.STOP,
                parameters={},
                description="Stop robot",
                estimated_duration=1.0
            ))

        total_duration = sum(step.estimated_duration for step in steps)

        return ActionPlan(
            steps=steps,
            description=f"Plan for: {command_text}",
            estimated_total_duration=total_duration
        )

    def _create_planning_prompt(self, command_text: str, world_state: WorldState) -> str:
        """Create the prompt for the LLM planning"""
        return f"""
        Command: {command_text}

        Current World State:
        - Robot Position: {world_state.robot_position}
        - Robot Orientation: {world_state.robot_orientation}
        - Objects: {world_state.objects}
        - Locations: {world_state.locations}
        - Available Actions: {[action.value for action in world_state.available_actions]}

        Generate a detailed action plan to execute the given command. The plan should be a sequence of simple actions that the robot can execute. Each action should have:
        - action_type: The type of action to perform
        - parameters: Any parameters needed for the action
        - description: A human-readable description of the step
        - estimated_duration: Estimated time to complete the step in seconds

        Return the plan as a JSON object with a "steps" array containing the action sequence.
        """

    def _get_system_prompt(self) -> str:
        """Get the system prompt for the LLM"""
        return """
        You are an expert robotic planning system. Your task is to convert natural language commands into detailed action plans for a humanoid robot. The robot can perform basic actions like moving, turning, navigating to locations, picking up objects, and placing objects.

        The robot operates in a simulated environment with known locations (kitchen, office, living room, etc.) and objects (cups, balls, books, etc.).

        Generate action plans that are:
        1. Sequential and logical
        2. Safe and executable
        3. Appropriate for the given command
        4. Consistent with the world state

        Return the plan as a JSON object with the following structure:
        {
          "steps": [
            {
              "action_type": "move_to_location|pick_up_object|place_object|move_forward|move_backward|turn_left|turn_right|stop",
              "parameters": {"location": "kitchen", "object": "cup", "distance": 1.0, "angle": 90.0, etc.},
              "description": "Human-readable description of the step",
              "estimated_duration": 2.5
            }
          ]
        }
        """

    def _parse_plan_response(self, plan_data: Dict[str, Any]) -> ActionPlan:
        """Parse the LLM response into an ActionPlan"""
        steps = []
        total_duration = 0.0

        for step_data in plan_data.get('steps', []):
            try:
                action_type = ActionType(step_data['action_type'])
                step = PlanStep(
                    action_type=action_type,
                    parameters=step_data.get('parameters', {}),
                    description=step_data.get('description', ''),
                    estimated_duration=step_data.get('estimated_duration', 1.0)
                )
                steps.append(step)
                total_duration += step.estimated_duration
            except (ValueError, KeyError) as e:
                logger.warning(f"Could not parse plan step: {step_data}, error: {e}")
                continue

        return ActionPlan(
            steps=steps,
            description=plan_data.get('description', 'Generated plan'),
            estimated_total_duration=total_duration
        )


# Global instance of the service
language_planning_service = LanguagePlanningService()