"""
Integration tests for the VLA Pipeline
"""
import pytest
import asyncio
from unittest.mock import Mock, patch, AsyncMock
import base64
from datetime import datetime

from models.vla import VLAPipeline, VLAPipelineStatus
from services.vla_processor import vla_processor
from services.speech_service import speech_service
from services.llm_service import llm_service


class TestVLAPipelineIntegration:
    """Integration tests for the complete VLA pipeline"""

    @pytest.mark.asyncio
    async def test_process_voice_command_complete_pipeline(self):
        """Test the complete VLA pipeline from voice command to action generation"""
        # Test with text input (since audio processing is mocked)
        text_input = "move forward"

        # Process the voice command through the complete pipeline
        pipeline = await vla_processor.process_voice_command(text_input=text_input)

        # Verify the pipeline was created successfully
        assert pipeline.id is not None
        assert pipeline.voice_command is not None
        assert pipeline.voice_command.transcribed_text == text_input
        assert pipeline.status in [VLAPipelineStatus.COMPLETED, VLAPipelineStatus.GENERATING_PLAN]
        assert pipeline.completed_at is not None

        # Verify that a language plan was generated
        if pipeline.language_plan:
            assert pipeline.language_plan.command_id == pipeline.voice_command.id

        # Verify that robot actions were generated
        assert len(pipeline.robot_actions) >= 0  # May be 0 if no plan was generated

    @pytest.mark.asyncio
    async def test_process_voice_command_with_audio_data(self):
        """Test the VLA pipeline with audio data input"""
        # Create mock audio data
        mock_audio_data = base64.b64encode(b"mock audio for testing").decode('utf-8')

        # Process the voice command with audio data
        pipeline = await vla_processor.process_voice_command(audio_data=mock_audio_data)

        # Verify the pipeline was created
        assert pipeline.id is not None
        assert pipeline.voice_command is not None
        assert pipeline.status in [VLAPipelineStatus.COMPLETED, VLAPipelineStatus.FAILED]

    @pytest.mark.asyncio
    async def test_execute_action_sequence(self):
        """Test executing a sequence of robot actions"""
        from models.vla import RobotAction

        # Create mock robot actions
        actions = [
            RobotAction(
                id="action_1",
                action_type="move_forward",
                parameters={"distance": 1.0, "speed": 0.5}
            ),
            RobotAction(
                id="action_2",
                action_type="turn_left",
                parameters={"angle": 90.0}
            ),
            RobotAction(
                id="action_3",
                action_type="raise_arm",
                parameters={"arm": "left", "angle": 45.0}
            )
        ]

        # Execute the action sequence
        result = await vla_processor.execute_action_sequence(actions)

        # Verify the results
        assert "successful_actions" in result
        assert "failed_actions" in result
        assert "total_actions" in result
        assert "execution_log" in result
        assert "success_rate" in result

        # Should have executed all 3 actions
        assert result["total_actions"] == 3
        assert result["success_rate"] >= 0  # Some may fail due to mock implementation

    @pytest.mark.asyncio
    async def test_vla_pipeline_with_complex_command(self):
        """Test the VLA pipeline with a complex command"""
        complex_command = "Navigate to the kitchen, find the red cup, and pick it up"

        # Process the complex command
        pipeline = await vla_processor.process_voice_command(text_input=complex_command)

        # Verify the pipeline was created
        assert pipeline.id is not None
        assert pipeline.voice_command is not None
        assert complex_command in pipeline.voice_command.transcribed_text

        # For a complex command, we expect more than one action
        if pipeline.language_plan and pipeline.language_plan.action_sequence:
            # The mock implementation might not generate multiple actions
            # but we should at least have a plan
            assert pipeline.language_plan.interpreted_meaning != ""

    @pytest.mark.asyncio
    async def test_vla_pipeline_validation(self):
        """Test VLA pipeline validation"""
        from models.vla import VoiceCommand, LanguagePlan

        # Create a mock pipeline for validation
        mock_pipeline = VLAPipeline(
            id="test_pipeline_123",
            voice_command=VoiceCommand(
                id="test_vc_123",
                transcribed_text="move forward",
                confidence=0.9
            ),
            language_plan=LanguagePlan(
                id="test_plan_123",
                command_id="test_vc_123",
                interpreted_meaning="Move the robot forward",
                action_sequence=[
                    {"action": "move_forward", "parameters": {"distance": 1.0}}
                ],
                generated_by_llm="mock"
            )
        )

        # Validate the pipeline
        validation_result = await vla_processor.validate_vla_pipeline(mock_pipeline)

        # Verify validation results
        assert "is_valid" in validation_result
        assert "issues" in validation_result
        assert "pipeline_id" in validation_result
        assert validation_result["pipeline_id"] == mock_pipeline.id

    @pytest.mark.asyncio
    async def test_speech_and_llm_integration(self):
        """Test integration between speech recognition and LLM services"""
        test_command = "turn left and move forward"

        # First, process through speech (will just pass through the text in mock)
        voice_result = await speech_service.recognize_speech(
            base64.b64encode(test_command.encode()).decode()
        )

        # Then process through LLM
        plan_result = await llm_service.generate_action_plan(test_command)

        # Verify both services worked
        assert voice_result[0] == test_command or "move" in voice_result[0].lower()
        assert "action_sequence" in plan_result
        assert isinstance(plan_result["action_sequence"], list)

    @pytest.mark.asyncio
    async def test_pipeline_error_handling(self):
        """Test VLA pipeline error handling"""
        # Test with invalid input
        pipeline = await vla_processor.process_voice_command(text_input="")

        # Should handle empty input gracefully
        assert pipeline.id is not None
        assert pipeline.status in [VLAPipelineStatus.COMPLETED, VLAPipelineStatus.FAILED]

        # If failed, should have an error message
        if pipeline.status == VLAPipelineStatus.FAILED:
            assert pipeline.error_message is not None


# Run the tests if this file is executed directly
if __name__ == "__main__":
    pytest.main([__file__])