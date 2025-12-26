"""
Main Vision-Language-Action (VLA) Service

This service integrates voice processing, language understanding, and action planning
to create a complete VLA system.
"""
import asyncio
import logging
from typing import Dict, Any, Optional, Union
from dataclasses import dataclass

from .voice_to_action import VoiceToActionService, VoiceCommand, ActionCommand
from .language_planning import LanguagePlanningService, WorldState, ActionPlan

logger = logging.getLogger(__name__)


@dataclass
class VLARequest:
    """Represents a VLA system request"""
    command_text: str
    audio_data: Optional[bytes] = None
    world_state: Optional[WorldState] = None


@dataclass
class VLAResponse:
    """Represents a VLA system response"""
    success: bool
    message: str
    action_plan: Optional[ActionPlan] = None
    action_command: Optional[ActionCommand] = None
    confidence: float = 0.0


class VLAMainService:
    """Main service that orchestrates the VLA system"""

    def __init__(self, openai_api_key: Optional[str] = None):
        self.voice_service = VoiceToActionService()
        self.planning_service = LanguagePlanningService(openai_api_key=openai_api_key)
        self.default_world_state = WorldState()

    async def process_request(self, request: VLARequest) -> VLAResponse:
        """
        Process a VLA request and return the appropriate response

        Args:
            request: The VLA request containing command and world state

        Returns:
            VLAResponse with the action plan or command
        """
        logger.info(f"Processing VLA request: {request.command_text}")

        try:
            # Determine the appropriate processing approach based on command complexity
            if self._is_complex_command(request.command_text):
                # Use language-based planning for complex commands
                return await self._process_complex_command(request)
            else:
                # Use direct voice-to-action for simple commands
                return await self._process_simple_command(request)

        except Exception as e:
            logger.error(f"Error processing VLA request: {e}")
            return VLAResponse(
                success=False,
                message=f"Error processing command: {str(e)}",
                confidence=0.0
            )

    def _is_complex_command(self, command_text: str) -> bool:
        """
        Determine if a command requires complex planning or simple action

        Args:
            command_text: The command text to evaluate

        Returns:
            True if the command is complex, False if simple
        """
        # Commands that typically require multi-step planning
        complex_indicators = [
            'and',  # Multiple actions
            'then',  # Sequential actions
            'after',  # Temporal dependencies
            'while',  # Concurrent actions
            'navigate to',  # Multi-step navigation
            'pick up',  # Manipulation tasks
            'place',  # Manipulation tasks
            'go to',  # Navigation tasks
            'find',  # Search tasks
        ]

        text_lower = command_text.lower()
        return any(indicator in text_lower for indicator in complex_indicators)

    async def _process_simple_command(self, request: VLARequest) -> VLAResponse:
        """Process a simple command using direct voice-to-action"""
        # Create voice command object
        voice_command = VoiceCommand(
            text=request.command_text,
            audio_data=request.audio_data
        )

        # Process with voice service
        action_command = await self.voice_service.process_voice_command(voice_command)

        return VLAResponse(
            success=True,
            message=f"Simple command processed: {action_command.action_type.value}",
            action_command=action_command,
            confidence=action_command.confidence
        )

    async def _process_complex_command(self, request: VLARequest) -> VLAResponse:
        """Process a complex command using language-based planning"""
        # Use the provided world state or default
        world_state = request.world_state or self.default_world_state

        # Generate action plan using LLM
        action_plan = await self.planning_service.generate_plan_from_command(
            request.command_text,
            world_state
        )

        return VLAResponse(
            success=True,
            message=f"Complex command planned: {len(action_plan.steps)} steps",
            action_plan=action_plan,
            confidence=0.8  # Default confidence for planned actions
        )

    async def execute_action_plan(self, plan: ActionPlan) -> Dict[str, Any]:
        """
        Execute an action plan (simulated execution)

        Args:
            plan: The action plan to execute

        Returns:
            Execution results
        """
        logger.info(f"Executing action plan with {len(plan.steps)} steps")

        results = {
            'completed_steps': 0,
            'successful_steps': 0,
            'failed_steps': 0,
            'execution_log': []
        }

        for i, step in enumerate(plan.steps):
            logger.info(f"Executing step {i+1}: {step.description}")

            # Simulate execution (in a real system, this would interface with ROS 2)
            execution_result = await self._execute_single_step(step)

            results['execution_log'].append({
                'step': i+1,
                'action_type': step.action_type.value,
                'description': step.description,
                'success': execution_result['success'],
                'details': execution_result.get('details', '')
            })

            results['completed_steps'] += 1
            if execution_result['success']:
                results['successful_steps'] += 1
            else:
                results['failed_steps'] += 1

                # In a real system, we might implement recovery strategies here
                logger.warning(f"Step {i+1} failed: {execution_result.get('error', 'Unknown error')}")
                break  # For now, stop on first failure

        return results

    async def _execute_single_step(self, step) -> Dict[str, Any]:
        """
        Execute a single action step (simulated)

        Args:
            step: The step to execute

        Returns:
            Execution result
        """
        # Simulate execution time
        await asyncio.sleep(step.estimated_duration * 0.1)  # Faster simulation

        # In a real system, this would interface with ROS 2 controllers
        # For simulation, we'll just return success
        return {
            'success': True,
            'details': f"Simulated execution of {step.action_type.value}",
            'duration': step.estimated_duration * 0.1
        }

    async def process_voice_request(self, command_text: str, audio_data: Optional[bytes] = None) -> VLAResponse:
        """
        Process a voice request directly (convenience method)

        Args:
            command_text: The voice command text
            audio_data: Optional audio data

        Returns:
            VLAResponse with the result
        """
        request = VLARequest(
            command_text=command_text,
            audio_data=audio_data
        )

        return await self.process_request(request)


# Global instance of the main service
vla_service = VLAMainService()