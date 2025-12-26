"""
Test suite for Vision-Language-Action (VLA) Module

This file contains tests to demonstrate and validate the VLA functionality.
"""
import asyncio
import os
from typing import Dict, Any

from .main_service import vla_service, VLARequest, WorldState


async def test_simple_commands():
    """Test simple voice-to-action commands"""
    print("Testing Simple Commands...")

    simple_commands = [
        "Move forward 2 meters",
        "Turn left 90 degrees",
        "Stop",
        "Go to the kitchen"
    ]

    for command in simple_commands:
        print(f"\nTesting: '{command}'")
        response = await vla_service.process_voice_request(command)

        print(f"Success: {response.success}")
        print(f"Message: {response.message}")
        if response.action_command:
            print(f"Action Type: {response.action_command.action_type.value}")
            print(f"Parameters: {response.action_command.parameters}")
            print(f"Confidence: {response.action_command.confidence}")
        print("-" * 50)


async def test_complex_commands():
    """Test complex language-based planning commands"""
    print("\n\nTesting Complex Commands...")

    # Create a sample world state for testing
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

    complex_commands = [
        "Navigate to the kitchen",
        "Pick up the red cup",
        "Go to the kitchen and pick up the red cup",
        "Find the blue ball and bring it to the office"
    ]

    for command in complex_commands:
        print(f"\nTesting: '{command}'")
        request = VLARequest(
            command_text=command,
            world_state=world_state
        )
        response = await vla_service.process_request(request)

        print(f"Success: {response.success}")
        print(f"Message: {response.message}")
        if response.action_plan:
            print(f"Plan Steps: {len(response.action_plan.steps)}")
            for i, step in enumerate(response.action_plan.steps, 1):
                print(f"  {i}. {step.description}")
                print(f"     Action: {step.action_type.value}")
                print(f"     Parameters: {step.parameters}")
                print(f"     Duration: {step.estimated_duration}s")
        print("-" * 50)


async def test_action_execution():
    """Test action plan execution"""
    print("\n\nTesting Action Execution...")

    # Create a simple plan for testing
    world_state = WorldState(
        robot_position={"x": 0.0, "y": 0.0, "z": 0.0},
        robot_orientation={"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        objects=[{"name": "test_object", "type": "cup", "position": {"x": 1.0, "y": 0.0, "z": 0.0}}],
        locations=[{"name": "test_location", "position": {"x": 2.0, "y": 0.0, "z": 0.0}}],
        available_actions=["move_forward", "pick_up_object"]
    )

    request = VLARequest(
        command_text="Move forward and pick up the cup",
        world_state=world_state
    )

    response = await vla_service.process_request(request)

    if response.action_plan:
        print(f"Executing plan with {len(response.action_plan.steps)} steps...")
        execution_results = await vla_service.execute_action_plan(response.action_plan)

        print(f"Completed Steps: {execution_results['completed_steps']}")
        print(f"Successful Steps: {execution_results['successful_steps']}")
        print(f"Failed Steps: {execution_results['failed_steps']}")

        for log_entry in execution_results['execution_log']:
            print(f"  Step {log_entry['step']}: {log_entry['description']} - {'PASS' if log_entry['success'] else 'FAIL'}")


async def run_all_tests():
    """Run all VLA tests"""
    print("=" * 60)
    print("VISION-LANGUAGE-ACTION (VLA) MODULE TESTS")
    print("=" * 60)

    # Test simple commands
    await test_simple_commands()

    # Test complex commands
    await test_complex_commands()

    # Test action execution
    await test_action_execution()

    print("\n" + "=" * 60)
    print("VLA TESTS COMPLETED")
    print("=" * 60)


if __name__ == "__main__":
    # Get OpenAI API key from environment if available
    openai_key = os.getenv("OPENAI_API_KEY")

    if openai_key:
        print("Using OpenAI API key from environment")
        # Reinitialize the service with the API key
        from .main_service import VLAMainService
        vla_service = VLAMainService(openai_api_key=openai_key)
    else:
        print("No OpenAI API key found - using mock implementation")

    # Run the tests
    asyncio.run(run_all_tests())