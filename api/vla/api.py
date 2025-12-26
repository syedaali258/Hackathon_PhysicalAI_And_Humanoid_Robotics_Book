"""
API Endpoints for Vision-Language-Action (VLA) Module

This module provides REST API endpoints for the VLA functionality.
"""
import os
import logging
from typing import Dict, Any, Optional
from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel

from .main_service import vla_service, VLARequest, VLAResponse, WorldState

logger = logging.getLogger(__name__)

# Create API router
router = APIRouter(prefix="/vla", tags=["vla"])

# Get OpenAI API key from environment
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")


class VLARequestModel(BaseModel):
    """Request model for VLA API"""
    command_text: str
    world_state: Optional[WorldState] = None


class VLAPlanRequestModel(BaseModel):
    """Request model for VLA plan execution"""
    command_text: str
    world_state: Optional[WorldState] = None


@router.post("/process", response_model=Dict[str, Any])
async def process_vla_request(request: VLARequestModel) -> Dict[str, Any]:
    """
    Process a VLA request and return the action plan or command

    Args:
        request: The VLA request containing command and optional world state

    Returns:
        Dictionary containing the response with action plan or command
    """
    try:
        vla_request = VLARequest(
            command_text=request.command_text,
            world_state=request.world_state
        )

        response: VLAResponse = await vla_service.process_request(vla_request)

        result = {
            "success": response.success,
            "message": response.message,
            "confidence": response.confidence
        }

        if response.action_command:
            result["action_command"] = {
                "action_type": response.action_command.action_type.value,
                "parameters": response.action_command.parameters,
                "confidence": response.action_command.confidence
            }

        if response.action_plan:
            result["action_plan"] = {
                "description": response.action_plan.description,
                "estimated_total_duration": response.action_plan.estimated_total_duration,
                "steps": [
                    {
                        "action_type": step.action_type.value,
                        "parameters": step.parameters,
                        "description": step.description,
                        "estimated_duration": step.estimated_duration
                    }
                    for step in response.action_plan.steps
                ]
            }

        logger.info(f"VLA request processed: {request.command_text[:50]}...")
        return result

    except Exception as e:
        logger.error(f"Error processing VLA request: {e}")
        raise HTTPException(status_code=500, detail=f"Error processing VLA request: {str(e)}")


@router.post("/plan", response_model=Dict[str, Any])
async def generate_action_plan(request: VLAPlanRequestModel) -> Dict[str, Any]:
    """
    Generate an action plan from a natural language command

    Args:
        request: The request containing command and world state

    Returns:
        Dictionary containing the action plan
    """
    try:
        world_state = request.world_state or WorldState()

        action_plan = await vla_service.planning_service.generate_plan_from_command(
            request.command_text,
            world_state
        )

        result = {
            "success": True,
            "message": f"Generated plan with {len(action_plan.steps)} steps",
            "action_plan": {
                "description": action_plan.description,
                "estimated_total_duration": action_plan.estimated_total_duration,
                "steps": [
                    {
                        "action_type": step.action_type.value,
                        "parameters": step.parameters,
                        "description": step.description,
                        "estimated_duration": step.estimated_duration
                    }
                    for step in action_plan.steps
                ]
            }
        }

        logger.info(f"Action plan generated: {request.command_text[:50]}...")
        return result

    except Exception as e:
        logger.error(f"Error generating action plan: {e}")
        raise HTTPException(status_code=500, detail=f"Error generating action plan: {str(e)}")


@router.post("/execute-plan", response_model=Dict[str, Any])
async def execute_action_plan(request: VLAPlanRequestModel) -> Dict[str, Any]:
    """
    Execute an action plan (simulated execution)

    Args:
        request: The request containing command and world state

    Returns:
        Dictionary containing execution results
    """
    try:
        # First generate the plan
        world_state = request.world_state or WorldState()
        action_plan = await vla_service.planning_service.generate_plan_from_command(
            request.command_text,
            world_state
        )

        # Then execute the plan
        execution_results = await vla_service.execute_action_plan(action_plan)

        result = {
            "success": True,
            "message": "Plan executed successfully",
            "execution_results": execution_results
        }

        logger.info(f"Action plan executed: {request.command_text[:50]}...")
        return result

    except Exception as e:
        logger.error(f"Error executing action plan: {e}")
        raise HTTPException(status_code=500, detail=f"Error executing action plan: {str(e)}")


@router.get("/health", response_model=Dict[str, str])
async def vla_health_check() -> Dict[str, str]:
    """
    Health check endpoint for VLA service

    Returns:
        Health status information
    """
    return {
        "status": "healthy",
        "module": "Vision-Language-Action (VLA)",
        "api_key_configured": str(OPENAI_API_KEY is not None)
    }


# Example usage endpoints for demonstration
@router.post("/demo/simple-command", response_model=Dict[str, Any])
async def demo_simple_command(command: str) -> Dict[str, Any]:
    """
    Demo endpoint for simple voice-to-action commands

    Args:
        command: A simple command string

    Returns:
        Dictionary containing the action command
    """
    try:
        response = await vla_service.process_voice_request(command)

        if response.action_command:
            result = {
                "command": command,
                "action_type": response.action_command.action_type.value,
                "parameters": response.action_command.parameters,
                "confidence": response.action_command.confidence
            }
        else:
            result = {
                "command": command,
                "error": "Could not process command"
            }

        return result

    except Exception as e:
        logger.error(f"Error in demo simple command: {e}")
        raise HTTPException(status_code=500, detail=f"Error processing demo command: {str(e)}")


@router.post("/demo/complex-command", response_model=Dict[str, Any])
async def demo_complex_command(command: str) -> Dict[str, Any]:
    """
    Demo endpoint for complex language-based planning

    Args:
        command: A complex command string

    Returns:
        Dictionary containing the action plan
    """
    try:
        # Use a default world state for demo
        world_state = WorldState(
            robot_position={"x": 0.0, "y": 0.0, "z": 0.0},
            robot_orientation={"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            objects=[
                {"name": "red_cup", "type": "cup", "color": "red", "position": {"x": 1.0, "y": 1.0, "z": 0.0}},
                {"name": "blue_ball", "type": "ball", "color": "blue", "position": {"x": 2.0, "y": 0.5, "z": 0.0}}
            ],
            locations=[
                {"name": "kitchen", "position": {"x": 5.0, "y": 0.0, "z": 0.0}},
                {"name": "office", "position": {"x": -3.0, "y": 2.0, "z": 0.0}},
                {"name": "living_room", "position": {"x": 0.0, "y": -3.0, "z": 0.0}}
            ],
            available_actions=["move_to_location", "pick_up_object", "place_object", "move_forward", "turn_left", "turn_right"]
        )

        request = VLARequest(
            command_text=command,
            world_state=world_state
        )

        response = await vla_service.process_request(request)

        if response.action_plan:
            result = {
                "command": command,
                "plan_steps": len(response.action_plan.steps),
                "estimated_duration": response.action_plan.estimated_total_duration,
                "steps": [
                    {
                        "action_type": step.action_type.value,
                        "description": step.description,
                        "parameters": step.parameters
                    }
                    for step in response.action_plan.steps
                ]
            }
        else:
            result = {
                "command": command,
                "error": "Could not generate plan"
            }

        return result

    except Exception as e:
        logger.error(f"Error in demo complex command: {e}")
        raise HTTPException(status_code=500, detail=f"Error processing demo command: {str(e)}")