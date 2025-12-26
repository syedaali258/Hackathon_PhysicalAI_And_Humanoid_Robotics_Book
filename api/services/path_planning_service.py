"""
Path Planning Service for the AI/Robotics Book API

This service handles path planning operations for Nav2 integration
for Module 3 content on The AI-Robot Brain (NVIDIA Isaac).
"""

from typing import Dict, Any, Optional, List
from datetime import datetime
import uuid
import logging
import numpy as np
from api.models.simulation import SimulationState
from api.services.navigation_service import navigation_service
from api.utils.errors import SimulationException


class PathPlanningService:
    """
    Service class for handling Nav2 path planning operations.
    """

    def __init__(self):
        self.logger = logging.getLogger(__name__)
        self.path_sessions: Dict[str, Dict[str, Any]] = {}
        self.default_planning_params = {
            "planner_frequency": 5.0,
            "max_planning_retries": 5,
            "humanoid_specific": {
                "max_step_size": 0.3,  # Humanoid step constraint
                "min_turning_radius": 0.4,  # Humanoid turning constraint
                "balance_margin": 0.1,  # Extra margin for balance
                "footstep_planning": True  # Enable footstep planning
            },
            "global_planner": {
                "plugin": "nav2_navfn_planner/NavfnPlanner",
                "tolerance": 0.5,
                "use_astar": False,
                "allow_unknown": True
            },
            "local_planner": {
                "plugin": "nav2_mppi_controller/MppiController",
                "frequency": 20.0,
                "max_vel_x": 0.5,  # Slower for humanoid stability
                "max_vel_theta": 1.0,
                "acc_lim_x": 2.5,
                "acc_lim_theta": 3.2
            }
        }

    async def create_path_planning_session(self, user_id: Optional[str], environment: str, planning_type: str) -> Dict[str, Any]:
        """
        Create a new path planning session with Nav2 integration.

        Args:
            user_id: Optional user identifier
            environment: Navigation environment for planning
            planning_type: Type of path planning (global, local, hybrid)

        Returns:
            Dictionary containing path planning session information
        """
        session_id = str(uuid.uuid4())

        # Validate planning type
        valid_types = ["global", "local", "hybrid", "footstep", "social"]
        if planning_type not in valid_types:
            raise SimulationException(f"Invalid planning type. Must be one of: {valid_types}")

        # Create initial path planning state
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
                "path_planning_status": "idle",
                "current_goal": None,
                "planned_path": [],
                "path_length": 0.0,
                "planning_success_rate": 0.0,
                "path_smoothing_enabled": True,
                "obstacle_avoidance_active": False,
                "social_navigation_enabled": False
            }
        )

        session = SimulationSession(
            id=session_id,
            user_id=user_id,
            simulation_type="path_planning",
            environment=environment,
            state=initial_state,
            created_at=datetime.now(),
            last_updated=datetime.now()
        )

        self.path_sessions[session_id] = session

        self.logger.info(
            f"Created new path planning session",
            extra={
                "session_id": session_id,
                "planning_type": planning_type,
                "environment": environment
            }
        )

        return session

    async def plan_path(self, session_id: str, start: Dict[str, float], goal: Dict[str, float], constraints: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """
        Plan a path from start to goal position with optional constraints.

        Args:
            session_id: Unique identifier for the path planning session
            start: Starting position {x, y, z}
            goal: Goal position {x, y, z}
            constraints: Optional path planning constraints

        Returns:
            Dictionary containing path planning results
        """
        if session_id not in self.path_sessions:
            raise SimulationException(f"Path planning session {session_id} not found")

        session = self.path_sessions[session_id]

        # Validate required coordinates
        for pos_name, pos in [("start", start), ("goal", goal)]:
            if not all(coord in pos for coord in ["x", "y"]):
                raise SimulationException(f"{pos_name} position must contain x and y coordinates")

        # Apply constraints if provided, otherwise use defaults
        if constraints is None:
            constraints = self.default_planning_params["humanoid_specific"]

        # Generate a mock path based on start and goal
        # In a real implementation, this would call Nav2's path planner
        path_points = self._generate_path_with_constraints(start, goal, constraints)

        # Calculate path metrics
        path_length = self._calculate_path_length(path_points)
        planning_time = np.random.uniform(0.05, 0.2)  # Simulate planning time

        # Create planning result
        planning_result = {
            "status": "path_found",
            "path": path_points,
            "pathLength": path_length,
            "planningTimeSec": planning_time,
            "waypointCount": len(path_points),
            "constraintsApplied": constraints,
            "smoothingApplied": True,
            "obstacleAvoidanceUsed": True
        }

        # Update session state with planning results
        session.state.isaac_specific["planned_path"] = path_points
        session.state.isaac_specific["path_length"] = path_length
        session.state.isaac_specific["path_planning_status"] = "path_found"
        session.state.isaac_specific["current_goal"] = goal
        session.last_updated = datetime.now()

        self.logger.info(
            f"Planned path from ({start['x']}, {start['y']}) to ({goal['x']}, {goal['y']})",
            extra={
                "session_id": session_id,
                "path_length": path_length,
                "waypoint_count": len(path_points),
                "planning_time": planning_time
            }
        )

        return planning_result

    def _generate_path_with_constraints(self, start: Dict[str, float], goal: Dict[str, float], constraints: Dict[str, Any]) -> List[Dict[str, float]]:
        """
        Generate a path considering humanoid-specific constraints.

        Args:
            start: Starting position
            goal: Goal position
            constraints: Path planning constraints

        Returns:
            List of path waypoints
        """
        # Calculate straight-line distance
        dx = goal["x"] - start["x"]
        dy = goal["y"] - start["y"]
        distance = np.sqrt(dx*dx + dy*dy)

        # Generate waypoints based on constraints
        # For humanoid robots, ensure path respects step size and turning constraints
        max_step_size = constraints.get("max_step_size", 0.3)
        steps = max(int(distance / max_step_size), 5)  # Ensure minimum number of waypoints

        path = []
        for i in range(steps + 1):
            ratio = i / steps
            waypoint = {
                "x": start["x"] + dx * ratio,
                "y": start["y"] + dy * ratio,
                "z": start.get("z", 0.0)  # Maintain z coordinate
            }

            # Add minor variations to simulate realistic humanoid path
            if 0 < i < steps:  # Not start or end point
                # Add small random variations to simulate natural movement
                waypoint["x"] += np.random.normal(0, 0.05)
                waypoint["y"] += np.random.normal(0, 0.05)

            path.append(waypoint)

        return path

    def _calculate_path_length(self, path: List[Dict[str, float]]) -> float:
        """
        Calculate the total length of the path.

        Args:
            path: List of path waypoints

        Returns:
            Total path length in meters
        """
        if len(path) < 2:
            return 0.0

        total_length = 0.0
        for i in range(1, len(path)):
            prev_point = path[i-1]
            curr_point = path[i]

            dx = curr_point["x"] - prev_point["x"]
            dy = curr_point["y"] - prev_point["y"]
            dz = curr_point["z"] - prev_point["z"]

            segment_length = np.sqrt(dx*dx + dy*dy + dz*dz)
            total_length += segment_length

        return total_length

    async def smooth_path(self, session_id: str, path: List[Dict[str, float]], method: str = "bspline") -> List[Dict[str, float]]:
        """
        Apply smoothing to the planned path.

        Args:
            session_id: Unique identifier for the path planning session
            path: Original path to smooth
            method: Smoothing method to use (bspline, cubic, dubins)

        Returns:
            Smoothed path as a list of waypoints
        """
        if session_id not in self.path_sessions:
            raise SimulationException(f"Path planning session {session_id} not found")

        valid_methods = ["bspline", "cubic", "dubins", "none"]
        if method not in valid_methods:
            raise SimulationException(f"Invalid smoothing method. Must be one of: {valid_methods}")

        if len(path) < 2:
            return path  # Can't smooth a path with less than 2 points

        # Apply smoothing based on method
        if method == "bspline":
            smoothed_path = self._apply_bspline_smoothing(path)
        elif method == "cubic":
            smoothed_path = self._apply_cubic_smoothing(path)
        elif method == "dubins":
            smoothed_path = self._apply_dubins_path_smoothing(path)
        else:
            smoothed_path = path  # No smoothing

        # Update session state
        session = self.path_sessions[session_id]
        session.state.isaac_specific["smoothing_method"] = method
        session.last_updated = datetime.now()

        self.logger.info(
            f"Applied {method} smoothing to path",
            extra={
                "session_id": session_id,
                "original_waypoints": len(path),
                "smoothed_waypoints": len(smoothed_path),
                "smoothing_method": method
            }
        )

        return smoothed_path

    def _apply_bspline_smoothing(self, path: List[Dict[str, float]]) -> List[Dict[str, float]]:
        """
        Apply B-spline smoothing to the path.

        Args:
            path: Original path

        Returns:
            B-spline smoothed path
        """
        # This is a simplified smoothing algorithm
        # In a real implementation, this would use proper B-spline interpolation
        if len(path) < 3:
            return path

        smoothed_path = [path[0]]  # Keep start point

        # Add smoothed intermediate points
        for i in range(1, len(path) - 1):
            prev_point = path[i-1]
            curr_point = path[i]
            next_point = path[i+1]

            # Calculate weighted average with neighbors for smoothing
            smoothed_point = {
                "x": 0.25 * prev_point["x"] + 0.5 * curr_point["x"] + 0.25 * next_point["x"],
                "y": 0.25 * prev_point["y"] + 0.5 * curr_point["y"] + 0.25 * next_point["y"],
                "z": 0.25 * prev_point["z"] + 0.5 * curr_point["z"] + 0.25 * next_point["z"]
            }

            smoothed_path.append(smoothed_point)

        smoothed_path.append(path[-1])  # Keep end point
        return smoothed_path

    def _apply_cubic_smoothing(self, path: List[Dict[str, float]]) -> List[Dict[str, float]]:
        """
        Apply cubic smoothing to the path.

        Args:
            path: Original path

        Returns:
            Cubic smoothed path
        """
        # Simplified cubic smoothing - in reality would use cubic spline interpolation
        return self._apply_bspline_smoothing(path)  # Using same approach for simplicity

    def _apply_dubins_path_smoothing(self, path: List[Dict[str, float]]) -> List[Dict[str, float]]:
        """
        Apply Dubins path smoothing (for vehicles with turning constraints).

        Args:
            path: Original path

        Returns:
            Dubins path smoothed path
        """
        # For humanoid robots, Dubins paths would consider turning radius constraints
        # Simplified implementation for demonstration
        if len(path) < 2:
            return path

        # In a real implementation, this would generate Dubins curves between waypoints
        # respecting the minimum turning radius constraint
        return path

    async def get_path_metrics(self, session_id: str) -> Dict[str, Any]:
        """
        Get metrics about the planned path.

        Args:
            session_id: Unique identifier for the path planning session

        Returns:
            Dictionary containing path metrics
        """
        if session_id not in self.path_sessions:
            raise SimulationException(f"Path planning session {session_id} not found")

        session = self.path_sessions[session_id]

        planned_path = session.state.isaac_specific.get("planned_path", [])
        path_length = session.state.isaac_specific.get("path_length", 0.0)

        # Calculate additional metrics
        if len(planned_path) > 1:
            # Calculate path curvature (simplified)
            total_curvature = 0.0
            for i in range(1, len(planned_path) - 1):
                p1 = planned_path[i-1]
                p2 = planned_path[i]
                p3 = planned_path[i+1]

                # Calculate angle between segments (simplified)
                v1 = (p2["x"] - p1["x"], p2["y"] - p1["y"])
                v2 = (p3["x"] - p2["x"], p3["y"] - p2["y"])

                # Dot product calculation
                dot_product = v1[0]*v2[0] + v1[1]*v2[1]
                magnitudes = np.sqrt(v1[0]**2 + v1[1]**2) * np.sqrt(v2[0]**2 + v2[1]**2)

                if magnitudes > 0:
                    cos_angle = max(-1, min(1, dot_product / magnitudes))  # Clamp to avoid numerical errors
                    angle = np.arccos(cos_angle)
                    total_curvature += angle

            avg_curvature = total_curvature / max(len(planned_path) - 2, 1)
        else:
            avg_curvature = 0.0

        metrics = {
            "pathLengthMeters": path_length,
            "waypointCount": len(planned_path),
            "averageCurvature": avg_curvature,
            "smoothingMethod": session.state.isaac_specific.get("smoothing_method", "none"),
            "obstacleAvoidanceUsed": session.state.isaac_specific.get("obstacle_avoidance_active", False),
            "socialNavigationEnabled": session.state.isaac_specific.get("social_navigation_enabled", False),
            "planningConstraints": session.state.isaac_specific.get("planning_constraints", {}),
            "timestamp": datetime.now().isoformat()
        }

        session.last_updated = datetime.now()

        self.logger.info(
            f"Retrieved path metrics",
            extra={
                "session_id": session_id,
                "path_length": metrics["pathLengthMeters"],
                "waypoint_count": metrics["waypointCount"]
            }
        )

        return metrics

    async def execute_path_following(self, session_id: str, path: List[Dict[str, float]], velocity_profile: Optional[Dict[str, float]] = None) -> Dict[str, Any]:
        """
        Execute path following for the planned path.

        Args:
            session_id: Unique identifier for the path planning session
            path: Path to follow
            velocity_profile: Optional velocity profile to apply

        Returns:
            Dictionary containing path execution results
        """
        if session_id not in self.path_sessions:
            raise SimulationException(f"Path planning session {session_id} not found")

        session = self.path_sessions[session_id]

        if not path:
            raise SimulationException("Cannot execute path following: path is empty")

        # Apply velocity profile if provided, otherwise use humanoid-appropriate defaults
        if velocity_profile is None:
            velocity_profile = {
                "max_linear_velocity": 0.3,  # Humanoid-specific slower speed for stability
                "max_angular_velocity": 0.5,
                "acceleration_limit": 0.8,
                "deceleration_limit": -1.0
            }

        # Simulate path following execution
        # In a real implementation, this would interface with Nav2's controller
        execution_result = {
            "status": "executing",
            "pathCompletedRatio": 0.0,
            "currentPosition": path[0],  # Start at first waypoint
            "distanceRemaining": self._calculate_path_length(path),
            "estimatedTimeToGoal": self._estimate_execution_time(path, velocity_profile),
            "velocityProfileApplied": velocity_profile,
            "obstacleEncountered": False,
            "replanningRequired": False
        }

        # Update session state
        session.state.isaac_specific["path_following_status"] = "executing"
        session.state.isaac_specific["current_path_index"] = 0
        session.state.isaac_specific["execution_velocity_profile"] = velocity_profile
        session.last_updated = datetime.now()

        self.logger.info(
            f"Started path following execution",
            extra={
                "session_id": session_id,
                "path_length": len(path),
                "estimated_time": execution_result["estimatedTimeToGoal"]
            }
        )

        return execution_result

    def _estimate_execution_time(self, path: List[Dict[str, float]], velocity_profile: Dict[str, float]) -> float:
        """
        Estimate execution time based on path length and velocity profile.

        Args:
            path: Path to estimate execution time for
            velocity_profile: Applied velocity profile

        Returns:
            Estimated execution time in seconds
        """
        path_length = self._calculate_path_length(path)
        max_velocity = velocity_profile.get("max_linear_velocity", 0.3)

        # Simple estimation: time = distance / average_velocity
        # In reality, this would account for acceleration/deceleration profiles
        if max_velocity > 0:
            estimated_time = path_length / max_velocity
            # Add buffer time for accelerations and turning
            estimated_time *= 1.3  # 30% buffer for humanoid-specific movement
        else:
            estimated_time = float('inf')

        return estimated_time

    async def update_path_planning_params(self, session_id: str, new_params: Dict[str, Any]) -> Dict[str, Any]:
        """
        Update path planning parameters for the session.

        Args:
            session_id: Unique identifier for the path planning session
            new_params: New parameters to apply

        Returns:
            Dictionary containing updated parameters
        """
        if session_id not in self.path_sessions:
            raise SimulationException(f"Path planning session {session_id} not found")

        session = self.path_sessions[session_id]

        # Update the planning parameters in session state
        current_params = session.state.isaac_specific.get("planning_params", self.default_planning_params.copy())
        current_params.update(new_params)

        session.state.isaac_specific["planning_params"] = current_params
        session.last_updated = datetime.now()

        self.logger.info(
            f"Updated path planning parameters",
            extra={
                "session_id": session_id,
                "updated_params": list(new_params.keys())
            }
        )

        return current_params

    async def reset_path_planning_session(self, session_id: str) -> SimulationState:
        """
        Reset the path planning session to its initial state.

        Args:
            session_id: Unique identifier for the path planning session

        Returns:
            SimulationState: Initial state of the path planning session
        """
        if session_id not in self.path_sessions:
            raise SimulationException(f"Path planning session {session_id} not found")

        # Create initial path planning state
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
                "path_planning_status": "idle",
                "current_goal": None,
                "planned_path": [],
                "path_length": 0.0,
                "planning_success_rate": 0.0,
                "path_smoothing_enabled": True,
                "obstacle_avoidance_active": False,
                "social_navigation_enabled": False,
                "planning_params": self.default_planning_params
            }
        )

        # Update the session
        session = self.path_sessions[session_id]
        session.state = initial_state
        session.last_updated = datetime.now()

        self.logger.info(
            f"Reset path planning session",
            extra={"session_id": session_id}
        )

        return initial_state

    async def end_path_planning_session(self, session_id: str) -> Dict[str, Any]:
        """
        End a path planning session and clean up resources.

        Args:
            session_id: Unique identifier for the path planning session

        Returns:
            Dict: Session summary information
        """
        if session_id not in self.path_sessions:
            raise SimulationException(f"Path planning session {session_id} not found")

        session = self.path_sessions[session_id]

        # Calculate session metrics
        duration = (datetime.now() - session.created_at).total_seconds()

        # Get final statistics
        final_stats = {
            "pathsPlanned": session.state.isaac_specific.get("paths_planned", 0),
            "pathSuccessRate": session.state.isaac_specific.get("planning_success_rate", 0.0),
            "totalDistancePlanned": session.state.isaac_specific.get("total_path_length", 0.0),
            "smoothingApplied": session.state.isaac_specific.get("smoothing_method", "none"),
            "sessionDurationSeconds": duration
        }

        # Remove the session
        del self.path_sessions[session_id]

        self.logger.info(
            f"Ended path planning session",
            extra={
                "session_id": session_id,
                "duration": duration,
                "final_stats": final_stats
            }
        )

        return {
            "message": "Path planning session ended successfully",
            "duration": int(duration),
            "finalStatistics": final_stats
        }


# Global instance of the service
path_planning_service = PathPlanningService()