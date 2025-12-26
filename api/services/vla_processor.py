"""
VLA (Vision-Language-Action) Processor Service
"""
import asyncio
import logging
from typing import Dict, List, Any, Optional
from datetime import datetime

from models.vla import (
    VoiceCommand, LanguagePlan, RobotAction,
    SimulationEnvironment, VLAPipeline, VLAPipelineStatus
)
from services.speech_service import speech_service
from services.llm_service import llm_service

logger = logging.getLogger(__name__)


class VLAProcessor:
    """
    Service for processing the complete VLA pipeline: Vision, Language, Action
    """

    def __init__(self):
        self.speech_service = speech_service
        self.llm_service = llm_service

    async def process_voice_command(self, audio_data: Optional[str] = None, text_input: Optional[str] = None) -> VLAPipeline:
        """
        Process a voice command through the complete VLA pipeline

        Args:
            audio_data: Base64 encoded audio data (optional if text_input provided)
            text_input: Text input (optional if audio_data provided)

        Returns:
            VLAPipeline with the complete processing result
        """
        try:
            # Create pipeline
            pipeline = VLAPipeline(
                id=f"vla_{int(datetime.now().timestamp())}_{hash(audio_data or text_input or '') % 10000}",
                voice_command=None,
                status=VLAPipelineStatus.PENDING
            )

            # Step 1: Process voice command
            if audio_data:
                recognized_text, confidence = await self.speech_service.recognize_speech(audio_data)
                pipeline.voice_command = VoiceCommand(
                    id=f"vc_{pipeline.id}",
                    transcribed_text=recognized_text,
                    confidence=confidence,
                    audio_data=audio_data
                )
            elif text_input:
                pipeline.voice_command = VoiceCommand(
                    id=f"vc_{pipeline.id}",
                    transcribed_text=text_input,
                    confidence=1.0
                )
            else:
                raise ValueError("Either audio_data or text_input must be provided")

            pipeline.status = VLAPipelineStatus.PROCESSING_SPEECH
            logger.info(f"Voice command processed: '{pipeline.voice_command.transcribed_text}'")

            # Step 2: Generate language plan
            plan_data = await self.llm_service.generate_action_plan(
                pipeline.voice_command.transcribed_text
            )

            if plan_data.get("action_sequence"):
                pipeline.language_plan = LanguagePlan(
                    id=f"plan_{pipeline.id}",
                    command_id=pipeline.voice_command.id,
                    interpreted_meaning=plan_data.get("interpreted_meaning", ""),
                    action_sequence=plan_data["action_sequence"],
                    generated_by_llm=plan_data.get("model", "unknown")
                )

                # Convert plan actions to RobotAction objects
                for i, action_dict in enumerate(plan_data["action_sequence"]):
                    robot_action = RobotAction(
                        id=f"action_{pipeline.id}_{i}",
                        action_type=action_dict.get("action", "unknown"),
                        parameters=action_dict.get("parameters", {})
                    )
                    pipeline.robot_actions.append(robot_action)

            pipeline.status = VLAPipelineStatus.GENERATING_PLAN
            logger.info(f"Language plan generated with {len(pipeline.robot_actions)} actions")

            # Step 3: For now, mark as completed (execution would happen in simulation)
            pipeline.status = VLAPipelineStatus.COMPLETED
            pipeline.completed_at = datetime.now()

            logger.info(f"VLA pipeline completed: {pipeline.id}")
            return pipeline

        except Exception as e:
            logger.error(f"Error in VLA processing: {str(e)}")
            pipeline.status = VLAPipelineStatus.FAILED
            pipeline.error_message = str(e)
            pipeline.completed_at = datetime.now()
            return pipeline

    async def execute_action_sequence(
        self,
        action_sequence: List[RobotAction],
        environment: Optional[SimulationEnvironment] = None
    ) -> Dict[str, Any]:
        """
        Execute a sequence of robot actions in the simulation environment

        Args:
            action_sequence: List of RobotAction objects to execute
            environment: Simulation environment context

        Returns:
            Dictionary with execution results
        """
        try:
            execution_log = []
            successful_actions = 0
            failed_actions = 0

            for i, action in enumerate(action_sequence):
                try:
                    # In a real implementation, this would interface with the simulation
                    # For now, we'll simulate execution
                    result = await self._execute_single_action(action, environment)
                    execution_log.append(f"Action {i+1}: {action.action_type} - {result}")
                    successful_actions += 1
                    logger.info(f"Executed action: {action.action_type}")

                    # Small delay between actions for realism
                    await asyncio.sleep(0.1)

                except Exception as e:
                    execution_log.append(f"Action {i+1}: {action.action_type} - FAILED: {str(e)}")
                    failed_actions += 1
                    logger.error(f"Failed to execute action {action.action_type}: {str(e)}")

            return {
                "successful_actions": successful_actions,
                "failed_actions": failed_actions,
                "total_actions": len(action_sequence),
                "execution_log": execution_log,
                "success_rate": successful_actions / len(action_sequence) if action_sequence else 0
            }

        except Exception as e:
            logger.error(f"Error executing action sequence: {str(e)}")
            return {
                "successful_actions": 0,
                "failed_actions": len(action_sequence),
                "total_actions": len(action_sequence),
                "execution_log": [f"Sequence execution failed: {str(e)}"],
                "success_rate": 0.0,
                "error": str(e)
            }

    async def _execute_single_action(
        self,
        action: RobotAction,
        environment: Optional[SimulationEnvironment] = None
    ) -> str:
        """
        Execute a single robot action in the simulation

        Args:
            action: RobotAction to execute
            environment: Simulation environment context

        Returns:
            Execution result message
        """
        try:
            # In a real implementation, this would call the simulation API
            # For now, return a mock result based on action type
            action_type = action.action_type.lower()

            if action_type in ["move_forward", "move_backward", "move_left", "move_right"]:
                distance = action.parameters.get("distance", 1.0)
                return f"Moved {action_type.replace('move_', '')} by {distance}m"
            elif action_type in ["turn_left", "turn_right"]:
                angle = action.parameters.get("angle", 90.0)
                return f"Turned {action_type.replace('turn_', '')} by {angle} degrees"
            elif action_type in ["raise_arm", "lower_arm"]:
                arm = action.parameters.get("arm", "both")
                angle = action.parameters.get("angle", 45.0)
                return f"{action_type.replace('_', ' ').title()} {arm} by {angle} degrees"
            elif action_type == "grasp_object":
                obj_id = action.parameters.get("object_id", "unknown")
                return f"Grasped object {obj_id}"
            elif action_type == "navigate_to":
                target = action.parameters.get("target", "unknown")
                return f"Navigated to {target}"
            elif action_type == "detect_object":
                obj_type = action.parameters.get("object_type", "unknown")
                return f"Detected {obj_type}"
            elif action_type == "stop":
                return "Stopped all movement"
            else:
                return f"Executed unknown action: {action_type}"

        except Exception as e:
            logger.error(f"Error executing single action: {str(e)}")
            return f"Failed: {str(e)}"

    async def validate_vla_pipeline(self, pipeline: VLAPipeline) -> Dict[str, Any]:
        """
        Validate a complete VLA pipeline for safety and feasibility

        Args:
            pipeline: VLAPipeline to validate

        Returns:
            Dictionary with validation results
        """
        try:
            issues = []

            # Validate voice command
            if not pipeline.voice_command or not pipeline.voice_command.transcribed_text.strip():
                issues.append("Voice command is empty or invalid")

            # Validate language plan
            if pipeline.language_plan and pipeline.language_plan.action_sequence:
                for i, action in enumerate(pipeline.language_plan.action_sequence):
                    if not action.get("action"):
                        issues.append(f"Action {i} is missing action type")
                    if "parameters" not in action:
                        issues.append(f"Action {i} is missing parameters")

            # Validate robot actions
            for i, action in enumerate(pipeline.robot_actions):
                if not action.action_type:
                    issues.append(f"Robot action {i} has no action type")

            return {
                "is_valid": len(issues) == 0,
                "issues": issues,
                "pipeline_id": pipeline.id
            }

        except Exception as e:
            logger.error(f"Error validating VLA pipeline: {str(e)}")
            return {
                "is_valid": False,
                "issues": [f"Validation error: {str(e)}"],
                "pipeline_id": pipeline.id if pipeline else "unknown"
            }

    async def create_simulation_environment(self, name: str, description: str, objects: List[Dict[str, Any]]) -> SimulationEnvironment:
        """
        Create a simulation environment for VLA pipeline execution

        Args:
            name: Name of the environment
            description: Description of the environment
            objects: List of objects in the environment

        Returns:
            SimulationEnvironment object
        """
        try:
            env = SimulationEnvironment(
                id=f"env_{int(datetime.now().timestamp())}",
                name=name,
                description=description,
                objects=objects,
                spatial_info={"origin": [0, 0, 0], "bounds": {"min": [-10, -10, 0], "max": [10, 10, 3]}}
            )

            logger.info(f"Created simulation environment: {env.name}")
            return env

        except Exception as e:
            logger.error(f"Error creating simulation environment: {str(e)}")
            raise


# Singleton instance
vla_processor = VLAProcessor()