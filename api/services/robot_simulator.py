"""
Robot Simulator Service for VLA Module
"""
import asyncio
import logging
from typing import Dict, List, Any, Optional
from datetime import datetime
import json
import os

from models.vla import RobotAction, SimulationEnvironment

logger = logging.getLogger(__name__)


class RobotSimulator:
    """
    Service for simulating robot actions in a virtual environment
    """

    def __init__(self):
        self.simulation_url = os.getenv("SIMULATION_URL", "http://localhost:11345")
        self.robot_state = {
            "position": {"x": 0.0, "y": 0.0, "z": 0.0},
            "orientation": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
            "arm_positions": {"left": 0.0, "right": 0.0},
            "gripper_state": {"left": "open", "right": "open"},
            "status": "idle"
        }
        self.is_connected = False

    async def connect_to_simulation(self) -> bool:
        """
        Connect to the simulation environment

        Returns:
            True if connection successful, False otherwise
        """
        try:
            # In a real implementation, this would connect to the simulation API
            # For now, we'll simulate the connection
            await asyncio.sleep(0.1)  # Simulate connection time
            self.is_connected = True
            logger.info(f"Connected to simulation at {self.simulation_url}")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to simulation: {str(e)}")
            self.is_connected = False
            return False

    async def execute_action(self, action: RobotAction, environment: Optional[SimulationEnvironment] = None) -> Dict[str, Any]:
        """
        Execute a robot action in the simulation

        Args:
            action: RobotAction to execute
            environment: Optional simulation environment context

        Returns:
            Dictionary with execution results
        """
        if not self.is_connected:
            success = await self.connect_to_simulation()
            if not success:
                return {
                    "success": False,
                    "error": "Not connected to simulation",
                    "action_id": action.id
                }

        try:
            # Update robot state based on action
            result = await self._apply_action_to_robot(action)

            # Simulate execution time based on action type
            await self._simulate_execution_time(action)

            # Validate action against environment constraints if provided
            if environment:
                validation = await self._validate_action_in_environment(action, environment)
                if not validation["valid"]:
                    return {
                        "success": False,
                        "error": f"Action not valid in environment: {validation['reason']}",
                        "action_id": action.id,
                        "validation": validation
                    }

            logger.info(f"Executed action: {action.action_type} with parameters {action.parameters}")

            return {
                "success": True,
                "action_id": action.id,
                "action_type": action.action_type,
                "result": result,
                "new_robot_state": self.robot_state.copy(),
                "execution_time": result.get("execution_time", 0)
            }

        except Exception as e:
            logger.error(f"Error executing action {action.action_type}: {str(e)}")
            return {
                "success": False,
                "action_id": action.id,
                "error": str(e),
                "robot_state": self.robot_state.copy()
            }

    async def _apply_action_to_robot(self, action: RobotAction) -> Dict[str, Any]:
        """
        Apply an action to update the robot's state

        Args:
            action: RobotAction to apply

        Returns:
            Dictionary with action result
        """
        action_type = action.action_type.lower()
        params = action.parameters

        result = {
            "action_applied": True,
            "previous_state": self.robot_state.copy(),
            "execution_time": 0.0
        }

        if action_type == "move_forward":
            distance = params.get("distance", 1.0)
            speed = params.get("speed", 0.5)
            self.robot_state["position"]["y"] += distance
            result["execution_time"] = abs(distance) / speed
            result["message"] = f"Moved forward {distance}m"

        elif action_type == "move_backward":
            distance = params.get("distance", 1.0)
            speed = params.get("speed", 0.5)
            self.robot_state["position"]["y"] -= distance
            result["execution_time"] = abs(distance) / speed
            result["message"] = f"Moved backward {distance}m"

        elif action_type == "move_left":
            distance = params.get("distance", 1.0)
            speed = params.get("speed", 0.5)
            self.robot_state["position"]["x"] -= distance
            result["execution_time"] = abs(distance) / speed
            result["message"] = f"Moved left {distance}m"

        elif action_type == "move_right":
            distance = params.get("distance", 1.0)
            speed = params.get("speed", 0.5)
            self.robot_state["position"]["x"] += distance
            result["execution_time"] = abs(distance) / speed
            result["message"] = f"Moved right {distance}m"

        elif action_type == "turn_left":
            angle = params.get("angle", 90.0)
            self.robot_state["orientation"]["yaw"] -= angle
            result["execution_time"] = abs(angle) / 90.0  # Normalize to 1 sec per 90 degrees
            result["message"] = f"Turned left {angle} degrees"

        elif action_type == "turn_right":
            angle = params.get("angle", 90.0)
            self.robot_state["orientation"]["yaw"] += angle
            result["execution_time"] = abs(angle) / 90.0  # Normalize to 1 sec per 90 degrees
            result["message"] = f"Turned right {angle} degrees"

        elif action_type == "raise_arm":
            arm = params.get("arm", "left").lower()
            angle = params.get("angle", 45.0)
            if arm in ["left", "both"]:
                self.robot_state["arm_positions"]["left"] = min(90.0, self.robot_state["arm_positions"]["left"] + angle)
            if arm in ["right", "both"]:
                self.robot_state["arm_positions"]["right"] = min(90.0, self.robot_state["arm_positions"]["right"] + angle)
            result["execution_time"] = abs(angle) / 45.0  # Normalize to 1 sec per 45 degrees
            result["message"] = f"Raised {arm} arm by {angle} degrees"

        elif action_type == "lower_arm":
            arm = params.get("arm", "left").lower()
            angle = params.get("angle", 45.0)
            if arm in ["left", "both"]:
                self.robot_state["arm_positions"]["left"] = max(0.0, self.robot_state["arm_positions"]["left"] - angle)
            if arm in ["right", "both"]:
                self.robot_state["arm_positions"]["right"] = max(0.0, self.robot_state["arm_positions"]["right"] - angle)
            result["execution_time"] = abs(angle) / 45.0  # Normalize to 1 sec per 45 degrees
            result["message"] = f"Lowered {arm} arm by {angle} degrees"

        elif action_type == "grasp_object":
            arm = params.get("arm", "left").lower()
            object_id = params.get("object_id", "unknown")
            if arm in ["left", "both"]:
                self.robot_state["gripper_state"]["left"] = "closed"
            if arm in ["right", "both"]:
                self.robot_state["gripper_state"]["right"] = "closed"
            result["execution_time"] = 1.0
            result["message"] = f"Grasped object {object_id} with {arm} gripper"

        elif action_type == "release_object":
            arm = params.get("arm", "left").lower()
            if arm in ["left", "both"]:
                self.robot_state["gripper_state"]["left"] = "open"
            if arm in ["right", "both"]:
                self.robot_state["gripper_state"]["right"] = "open"
            result["execution_time"] = 1.0
            result["message"] = f"Released object with {arm} gripper"

        elif action_type == "navigate_to":
            target_x = params.get("x", self.robot_state["position"]["x"])
            target_y = params.get("y", self.robot_state["position"]["y"])
            target_z = params.get("z", self.robot_state["position"]["z"])

            # Calculate distance to target
            distance = ((target_x - self.robot_state["position"]["x"])**2 +
                       (target_y - self.robot_state["position"]["y"])**2 +
                       (target_z - self.robot_state["position"]["z"])**2)**0.5

            self.robot_state["position"]["x"] = target_x
            self.robot_state["position"]["y"] = target_y
            self.robot_state["position"]["z"] = target_z

            result["execution_time"] = distance / 0.5  # Assume 0.5 m/s speed
            result["message"] = f"Navigated to position ({target_x}, {target_y}, {target_z})"

        elif action_type == "detect_object":
            obj_type = params.get("object_type", "unknown")
            result["execution_time"] = 2.0
            result["message"] = f"Detected {obj_type} in environment"
            result["detected_objects"] = [{"type": obj_type, "distance": 2.0, "angle": 0.0}]

        elif action_type == "stop":
            self.robot_state["status"] = "idle"
            result["execution_time"] = 0.1
            result["message"] = "Robot stopped"

        else:
            result["action_applied"] = False
            result["message"] = f"Unknown action type: {action_type}"
            result["execution_time"] = 0.0

        # Update status
        if action_type != "stop":
            self.robot_state["status"] = "active"

        return result

    async def _simulate_execution_time(self, action: RobotAction):
        """
        Simulate the time it takes to execute an action
        """
        action_type = action.action_type.lower()
        params = action.parameters

        # Determine execution time based on action type and parameters
        if action_type in ["move_forward", "move_backward", "move_left", "move_right"]:
            distance = params.get("distance", 1.0)
            speed = params.get("speed", 0.5)
            sleep_time = abs(distance) / speed
        elif action_type in ["turn_left", "turn_right"]:
            angle = params.get("angle", 90.0)
            sleep_time = abs(angle) / 90.0  # 1 second per 90 degrees
        elif action_type in ["raise_arm", "lower_arm"]:
            angle = params.get("angle", 45.0)
            sleep_time = abs(angle) / 45.0  # 1 second per 45 degrees
        elif action_type in ["grasp_object", "release_object"]:
            sleep_time = 1.0
        elif action_type == "navigate_to":
            # Simulate based on distance
            target_x = params.get("x", self.robot_state["position"]["x"])
            target_y = params.get("y", self.robot_state["position"]["y"])
            distance = ((target_x - self.robot_state["position"]["x"])**2 +
                       (target_y - self.robot_state["position"]["y"])**2)**0.5
            sleep_time = distance / 0.5  # Assume 0.5 m/s speed
        elif action_type == "detect_object":
            sleep_time = 2.0
        else:
            sleep_time = 0.5  # Default execution time

        # Add some randomness to make it more realistic
        sleep_time = max(0.1, sleep_time * (0.8 + 0.4 * asyncio.get_event_loop().time() % 1))

        await asyncio.sleep(min(sleep_time, 5.0))  # Cap at 5 seconds

    async def _validate_action_in_environment(self, action: RobotAction, environment: SimulationEnvironment) -> Dict[str, Any]:
        """
        Validate if an action is valid in the given environment

        Args:
            action: RobotAction to validate
            environment: SimulationEnvironment to validate against

        Returns:
            Dictionary with validation result
        """
        try:
            action_type = action.action_type.lower()
            params = action.parameters

            # Check for collisions or invalid positions
            if action_type in ["move_forward", "move_backward", "move_left", "move_right", "navigate_to"]:
                # For navigate_to, get the target position
                if action_type == "navigate_to":
                    target_x = params.get("x", self.robot_state["position"]["x"])
                    target_y = params.get("y", self.robot_state["position"]["y"])
                    new_x, new_y = target_x, target_y
                else:
                    # For movement actions, calculate new position
                    new_x, new_y = self.robot_state["position"]["x"], self.robot_state["position"]["y"]
                    distance = params.get("distance", 1.0)

                    if action_type == "move_forward":
                        new_y += distance
                    elif action_type == "move_backward":
                        new_y -= distance
                    elif action_type == "move_left":
                        new_x -= distance
                    elif action_type == "move_right":
                        new_x += distance

                # Check if new position is within environment bounds (if specified)
                if environment.spatial_info and "bounds" in environment.spatial_info:
                    bounds = environment.spatial_info["bounds"]
                    min_x, min_y = bounds["min"][0], bounds["min"][1]
                    max_x, max_y = bounds["max"][0], bounds["max"][1]

                    if not (min_x <= new_x <= max_x and min_y <= new_y <= max_y):
                        return {
                            "valid": False,
                            "reason": f"Position ({new_x}, {new_y}) is outside environment bounds ({min_x}, {min_y}) to ({max_x}, {max_y})"
                        }

            # Check for object interaction validity
            if action_type == "grasp_object":
                obj_id = params.get("object_id", "unknown")
                # Check if object exists in environment
                obj_exists = any(obj.get("id") == obj_id for obj in environment.objects)
                if not obj_exists:
                    # Check if any object of the right type exists nearby
                    obj_type = params.get("object_type")
                    if obj_type:
                        nearby_objects = [obj for obj in environment.objects
                                        if obj.get("type") == obj_type]
                        if not nearby_objects:
                            return {
                                "valid": False,
                                "reason": f"No {obj_type} object found in environment for grasping"
                            }

            return {"valid": True, "reason": "Action is valid in environment"}

        except Exception as e:
            logger.error(f"Error validating action in environment: {str(e)}")
            return {"valid": False, "reason": f"Validation error: {str(e)}"}

    async def get_robot_state(self) -> Dict[str, Any]:
        """
        Get the current state of the robot

        Returns:
            Dictionary with robot state information
        """
        return self.robot_state.copy()

    async def reset_robot(self):
        """
        Reset the robot to its initial state
        """
        self.robot_state = {
            "position": {"x": 0.0, "y": 0.0, "z": 0.0},
            "orientation": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
            "arm_positions": {"left": 0.0, "right": 0.0},
            "gripper_state": {"left": "open", "right": "open"},
            "status": "idle"
        }
        logger.info("Robot state reset to initial position")

    async def execute_action_sequence(
        self,
        actions: List[RobotAction],
        environment: Optional[SimulationEnvironment] = None
    ) -> Dict[str, Any]:
        """
        Execute a sequence of robot actions

        Args:
            actions: List of RobotAction objects to execute
            environment: Optional simulation environment context

        Returns:
            Dictionary with execution results
        """
        results = []
        successful_actions = 0
        failed_actions = 0

        for i, action in enumerate(actions):
            try:
                result = await self.execute_action(action, environment)
                results.append(result)

                if result["success"]:
                    successful_actions += 1
                else:
                    failed_actions += 1

                logger.info(f"Action {i+1}/{len(actions)}: {action.action_type} - {'Success' if result['success'] else 'Failed'}")

                # Small delay between actions
                await asyncio.sleep(0.1)

            except Exception as e:
                error_result = {
                    "success": False,
                    "action_id": action.id,
                    "error": str(e),
                    "index": i
                }
                results.append(error_result)
                failed_actions += 1
                logger.error(f"Failed to execute action {i+1}: {str(e)}")

        return {
            "total_actions": len(actions),
            "successful_actions": successful_actions,
            "failed_actions": failed_actions,
            "success_rate": successful_actions / len(actions) if actions else 0,
            "results": results,
            "final_robot_state": await self.get_robot_state()
        }


# Singleton instance
robot_simulator = RobotSimulator()