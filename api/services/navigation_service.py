"""
Navigation Service for the AI/Robotics Book API

This service handles Nav2 navigation and path planning operations
for Module 3 content on The AI-Robot Brain (NVIDIA Isaac).
"""

from typing import Dict, Any, Optional, List
from datetime import datetime
import uuid
import logging
import asyncio
import numpy as np
from api.models.simulation import SimulationState, SimulationCommand
from api.services.isaac_service import isaac_service
from api.utils.errors import SimulationException


class NavigationService:
    """
    Service class for handling Nav2 navigation and path planning operations.
    """

    def __init__(self):
        self.logger = logging.getLogger(__name__)
        self.navigation_sessions: Dict[str, Dict[str, Any]] = {}
        self.default_nav_params = {
            "planner_frequency": 5.0,
            "controller_frequency": 20.0,
            "planner_patience": 5.0,
            "controller_patience": 15.0,
            "max_planning_retries": 5,
            "humanoid_specific": {
                "max_velocity": 0.3,  # Slower for humanoid stability
                "min_turning_radius": 0.4,
                "balance_margin": 0.1,
                "social_distance": 1.0  # Maintain distance from humans
            }
        }

    async def create_navigation_session(self, user_id: Optional[str], nav_type: str, environment: str) -> Dict[str, Any]:
        """
        Create a new navigation session with Nav2 integration.

        Args:
            user_id: Optional user identifier
            nav_type: Type of navigation (global_planning, local_planning, recovery)
            environment: Specific navigation environment

        Returns:
            Dictionary containing navigation session information
        """
        session_id = str(uuid.uuid4())

        # Validate navigation type
        valid_types = ["global_planning", "local_planning", "recovery", "path_following", "exploration"]
        if nav_type not in valid_types:
            raise SimulationException(f"Invalid navigation type. Must be one of: {valid_types}")

        # Create initial navigation state
        initial_state = SimulationState(
            robot_position={
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "rotation": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0,
                    "w": 1.0
                }
            },
            sensors={},
            physics={
                "gravity": -9.81,
                "collision_count": 0,
                "simulation_speed": 1.0
            },
            isaac_specific={
                "navigation_status": "idle",
                "current_goal": None,
                "path_remaining": 0.0,
                "navigation_success_rate": 0.0,
                "recovery_count": 0,
                "social_compliance": 1.0  # 1.0 = fully compliant
            }
        )

        session = SimulationSession(
            id=session_id,
            user_id=user_id,
            simulation_type="navigation",
            environment=environment,
            state=initial_state,
            created_at=datetime.now(),
            last_updated=datetime.now()
        )

        self.navigation_sessions[session_id] = session

        self.logger.info(
            f"Created new navigation session",
            extra={
                "session_id": session_id,
                "nav_type": nav_type,
                "environment": environment
            }
        )

        return session

    async def set_navigation_goal(self, session_id: str, goal: Dict[str, Any]) -> Dict[str, Any]:
        """
        Set a navigation goal for the robot in the simulation.

        Args:
            session_id: Unique identifier for the navigation session
            goal: Dictionary containing goal coordinates and orientation

        Returns:
            Dictionary containing navigation planning result
        """
        if session_id not in self.navigation_sessions:
            raise SimulationException(f"Navigation session {session_id} not found")

        session = self.navigation_sessions[session_id]

        # Validate goal structure
        required_keys = ["x", "y", "z"]
        for key in required_keys:
            if key not in goal:
                raise SimulationException(f"Goal must contain '{key}' coordinate")

        # Update session state with goal
        session.state.isaac_specific["current_goal"] = goal
        session.state.isaac_specific["navigation_status"] = "planning"

        # Simulate path planning (in real implementation, this would call Nav2 planner)
        path_result = {
            "status": "path_found",
            "path": self._generate_mock_path(session.state.robot_position, goal),
            "path_length": self._calculate_path_length(goal, session.state.robot_position),
            "planning_time_ms": np.random.uniform(50, 200),  # Simulate planning time
            "expected_travel_time": self._estimate_travel_time(goal, session.state.robot_position)
        }

        # Update session with path information
        session.state.isaac_specific["planned_path"] = path_result["path"]
        session.state.isaac_specific["path_length"] = path_result["path_length"]
        session.state.isaac_specific["navigation_status"] = "path_found"

        session.last_updated = datetime.now()

        self.logger.info(
            f"Set navigation goal",
            extra={
                "session_id": session_id,
                "goal": goal,
                "path_length": path_result["path_length"],
                "planning_time": path_result["planning_time_ms"]
            }
        )

        return path_result

    def _generate_mock_path(self, start: Dict[str, Any], goal: Dict[str, Any]) -> List[Dict[str, Any]]:
        """
        Generate a mock path between start and goal positions.

        Args:
            start: Starting position
            goal: Goal position

        Returns:
            List of path waypoints
        """
        # Calculate straight-line path with intermediate waypoints
        steps = 20
        path = []
        for i in range(steps + 1):
            ratio = i / steps
            waypoint = {
                "x": start["x"] + (goal["x"] - start["x"]) * ratio,
                "y": start["y"] + (goal["y"] - start["y"]) * ratio,
                "z": start["z"] + (goal["z"] - start["z"]) * ratio
            }
            path.append(waypoint)

        return path

    def _calculate_path_length(self, goal: Dict[str, Any], start: Dict[str, Any]) -> float:
        """
        Calculate Euclidean distance between start and goal.

        Args:
            goal: Goal position
            start: Start position

        Returns:
            Distance between points
        """
        dx = goal["x"] - start["x"]
        dy = goal["y"] - start["y"]
        dz = goal["z"] - start["z"]
        return np.sqrt(dx*dx + dy*dy + dz*dz)

    def _estimate_travel_time(self, goal: Dict[str, Any], start: Dict[str, Any]) -> float:
        """
        Estimate travel time based on distance and humanoid-specific parameters.

        Args:
            goal: Goal position
            start: Start position

        Returns:
            Estimated travel time in seconds
        """
        distance = self._calculate_path_length(goal, start)
        # Humanoid moves at 0.3 m/s on average
        base_speed = self.default_nav_params["humanoid_specific"]["max_velocity"]
        return distance / base_speed

    async def execute_navigation(self, session_id: str, recovery_behavior: Optional[str] = None) -> Dict[str, Any]:
        """
        Execute navigation to the set goal with optional recovery behavior.

        Args:
            session_id: Unique identifier for the navigation session
            recovery_behavior: Optional recovery behavior to apply

        Returns:
            Dictionary containing navigation execution result
        """
        if session_id not in self.navigation_sessions:
            raise SimulationException(f"Navigation session {session_id} not found")

        session = self.navigation_sessions[session_id]

        # Check if a goal is set
        current_goal = session.state.isaac_specific.get("current_goal")
        if not current_goal:
            raise SimulationException("No navigation goal set. Call set_navigation_goal first.")

        # Update navigation status
        session.state.isaac_specific["navigation_status"] = "executing"
        if recovery_behavior:
            session.state.isaac_specific["active_recovery"] = recovery_behavior

        # Simulate navigation execution
        # In a real implementation, this would call Nav2 controller
        execution_result = {
            "status": "executing",
            "current_position": session.state.robot_position,
            "remaining_distance": self._calculate_path_length(current_goal, session.state.robot_position),
            "execution_time": 0.0,
            "recovery_applied": recovery_behavior or "none",
            "social_compliance": session.state.isaac_specific.get("social_compliance", 1.0)
        }

        # Simulate robot movement toward goal
        await asyncio.sleep(0.1)  # Simulate processing time

        # Update robot position (move 10% of the way toward goal)
        if execution_result["remaining_distance"] > 0.1:  # Not at goal yet
            progress_ratio = 0.1
            new_x = session.state.robot_position["x"] + (current_goal["x"] - session.state.robot_position["x"]) * progress_ratio
            new_y = session.state.robot_position["y"] + (current_goal["y"] - session.state.robot_position["y"]) * progress_ratio
            new_z = session.state.robot_position["z"] + (current_goal["z"] - session.state.robot_position["z"]) * progress_ratio

            session.state.robot_position["x"] = new_x
            session.state.robot_position["y"] = new_y
            session.state.robot_position["z"] = new_z

        # Update session state
        session.state.isaac_specific["navigation_status"] = "executing"
        session.last_updated = datetime.now()

        self.logger.info(
            f"Executed navigation step",
            extra={
                "session_id": session_id,
                "remaining_distance": execution_result["remainingDistance"],
                "recovery_behavior": recovery_behavior
            }
        )

        return execution_result

    async def get_navigation_state(self, session_id: str) -> Dict[str, Any]:
        """
        Get the current state of the navigation system.

        Args:
            session_id: Unique identifier for the navigation session

        Returns:
            Dictionary containing navigation state information
        """
        if session_id not in self.navigation_sessions:
            raise SimulationException(f"Navigation session {session_id} not found")

        session = self.navigation_sessions[session_id]
        session.last_updated = datetime.now()

        navigation_state = {
            "status": session.state.isaac_specific.get("navigation_status", "idle"),
            "current_position": session.state.robot_position,
            "current_goal": session.state.isaac_specific.get("current_goal"),
            "path_remaining": session.state.isaac_specific.get("path_remaining", 0.0),
            "travelled_path": session.state.isaac_specific.get("travelled_path", []),
            "recovery_count": session.state.isaac_specific.get("recovery_count", 0),
            "social_compliance": session.state.isaac_specific.get("social_compliance", 1.0),
            "collision_avoidance_active": session.state.isaac_specific.get("collision_avoidance_active", False),
            "localization_accuracy": session.state.isaac_specific.get("localization_accuracy", 0.0),
            "timestamp": datetime.now().isoformat()
        }

        return navigation_state

    async def cancel_navigation(self, session_id: str) -> Dict[str, Any]:
        """
        Cancel the current navigation goal.

        Args:
            session_id: Unique identifier for the navigation session

        Returns:
            Dictionary confirming cancellation
        """
        if session_id not in self.navigation_sessions:
            raise SimulationException(f"Navigation session {session_id} not found")

        session = self.navigation_sessions[session_id]

        # Update session state to cancelled
        session.state.isaac_specific["navigation_status"] = "cancelled"
        session.state.isaac_specific["current_goal"] = None
        session.last_updated = datetime.now()

        self.logger.info(
            f"Cancelled navigation",
            extra={"session_id": session_id}
        )

        return {
            "message": "Navigation cancelled successfully",
            "previous_goal": session.state.isaac_specific.get("current_goal")
        }

    async def execute_recovery_behavior(self, session_id: str, behavior: str) -> Dict[str, Any]:
        """
        Execute a specific recovery behavior.

        Args:
            session_id: Unique identifier for the navigation session
            behavior: Recovery behavior to execute (spin, backup, wait)

        Returns:
            Dictionary containing recovery execution result
        """
        if session_id not in self.navigation_sessions:
            raise SimulationException(f"Navigation session {session_id} not found")

        valid_behaviors = ["spin", "backup", "wait", "clear_costmap", "assisted_teleop"]
        if behavior not in valid_behaviors:
            raise SimulationException(f"Invalid recovery behavior. Must be one of: {valid_behaviors}")

        session = self.navigation_sessions[session_id]

        # Update recovery count
        recovery_count = session.state.isaac_specific.get("recovery_count", 0)
        session.state.isaac_specific["recovery_count"] = recovery_count + 1
        session.state.isaac_specific["active_recovery"] = behavior

        # Simulate recovery behavior execution
        recovery_result = {
            "behavior": behavior,
            "status": "completed",
            "duration": 0.0,
            "success": True,
            "new_state": "ready_to_navigate"  # After recovery, robot is ready for new goals
        }

        if behavior == "spin":
            # Simulate spinning in place
            recovery_result["duration"] = np.random.uniform(1.0, 3.0)
            # Add slight position change due to spin maneuver
            session.state.robot_position["z"] += np.random.uniform(-0.05, 0.05)
        elif behavior == "backup":
            # Simulate backing up slightly
            recovery_result["duration"] = np.random.uniform(0.5, 1.5)
            # Move robot back a bit
            session.state.robot_position["x"] -= np.random.uniform(0.1, 0.3)
        elif behavior == "wait":
            # Simulate waiting behavior
            recovery_result["duration"] = np.random.uniform(1.0, 5.0)
            # No position change during wait
        elif behavior == "clear_costmap":
            # Simulate clearing costmap
            recovery_result["duration"] = np.random.uniform(0.5, 1.0)
            # Clear any virtual obstacles
        elif behavior == "assisted_teleop":
            # Simulate assisted teleoperation
            recovery_result["duration"] = np.random.uniform(2.0, 5.0)
            # Human takes control for navigation assistance

        # Update navigation status after recovery
        session.state.isaac_specific["navigation_status"] = "ready"
        session.last_updated = datetime.now()

        self.logger.info(
            f"Executed recovery behavior",
            extra={
                "session_id": session_id,
                "behavior": behavior,
                "duration": recovery_result["duration"]
            }
        )

        return recovery_result

    async def reset_navigation_session(self, session_id: str) -> SimulationState:
        """
        Reset the navigation session to its initial state.

        Args:
            session_id: Unique identifier for the navigation session

        Returns:
            SimulationState: Initial state of the navigation session
        """
        if session_id not in self.navigation_sessions:
            raise SimulationException(f"Navigation session {session_id} not found")

        # Create initial navigation state
        initial_state = SimulationState(
            robot_position={
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "rotation": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": 0.0,
                    "w": 1.0
                }
            },
            sensors={},
            physics={
                "gravity": -9.81,
                "collision_count": 0,
                "simulation_speed": 1.0
            },
            isaac_specific={
                "navigation_status": "idle",
                "current_goal": None,
                "path_remaining": 0.0,
                "navigation_success_rate": 0.0,
                "recovery_count": 0,
                "social_compliance": 1.0,
                "collision_avoidance_active": False,
                "localization_accuracy": 0.0
            }
        )

        # Update the session
        session = self.navigation_sessions[session_id]
        session.state = initial_state
        session.last_updated = datetime.now()

        self.logger.info(
            f"Reset navigation session",
            extra={"session_id": session_id}
        )

        return initial_state

    async def end_navigation_session(self, session_id: str) -> Dict[str, Any]:
        """
        End a navigation session and clean up resources.

        Args:
            session_id: Unique identifier for the navigation session

        Returns:
            Dict: Session summary information
        """
        if session_id not in self.navigation_sessions:
            raise SimulationException(f"Navigation session {session_id} not found")

        session = self.navigation_sessions[session_id]

        # Calculate session metrics
        duration = (datetime.now() - session.created_at).total_seconds()

        # Get final statistics
        final_stats = {
            "navigationAttempts": session.state.isaac_specific.get("navigation_attempts", 0),
            "successfulNavigations": session.state.isaac_specific.get("successful_navigations", 0),
            "recoveryExecutions": session.state.isaac_specific.get("recovery_count", 0),
            "totalDistanceTraveled": session.state.isaac_specific.get("total_distance_traveled", 0.0),
            "averageTravelTime": session.state.isaac_specific.get("average_travel_time", 0.0),
            "socialComplianceAverage": session.state.isaac_specific.get("social_compliance", 1.0)
        }

        # Remove the session
        del self.navigation_sessions[session_id]

        self.logger.info(
            f"Ended navigation session",
            extra={
                "session_id": session_id,
                "duration": duration,
                "final_stats": final_stats
            }
        )

        return {
            "message": "Navigation session ended successfully",
            "duration": int(duration),
            "final_statistics": final_stats
        }


# Global instance of the service
navigation_service = NavigationService()