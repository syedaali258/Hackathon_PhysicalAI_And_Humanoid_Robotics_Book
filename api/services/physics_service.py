"""
Physics Service for the AI/Robotics Book API

This service handles physics simulation operations for Gazebo-based simulations
in the Module 2 content on The Digital Twin (Gazebo & Unity).
"""

from typing import Dict, Any, Optional, List
from datetime import datetime
import uuid
import logging
from api.models.simulation import SimulationState
from api.services.digital_twin_service import digital_twin_service
from api.utils.errors import SimulationException


class PhysicsService:
    """
    Service class for handling physics simulation operations.
    """

    def __init__(self):
        self.logger = logging.getLogger(__name__)
        self.gravity_constant = -9.81  # Earth's gravity in m/s^2
        self.collision_threshold = 0.01  # Threshold for detecting collisions

    async def update_physics_state(self, state: SimulationState, dt: float = 0.01) -> SimulationState:
        """
        Update the physics state based on physical laws and time step.

        Args:
            state: Current simulation state
            dt: Time step for physics update

        Returns:
            SimulationState: Updated state after physics calculations
        """
        # Create a copy of the state
        new_state = state.copy()

        # Apply gravity to the robot
        new_state.physics["gravity"] = self.gravity_constant

        # Update position based on velocity (simplified physics)
        # In a real implementation, this would be much more complex
        velocity = new_state.physics.get("velocity", {"x": 0.0, "y": 0.0, "z": 0.0})

        # Update position: new_pos = old_pos + velocity * dt
        new_state.robot_position["x"] += velocity["x"] * dt
        new_state.robot_position["y"] += velocity["y"] * dt
        new_state.robot_position["z"] += velocity["z"] * dt

        # Apply gravity effect on Z position
        velocity["z"] += self.gravity_constant * dt
        new_state.physics["velocity"] = velocity

        # Check for collisions (simplified)
        collision_count = new_state.physics.get("collision_count", 0)
        # In a real implementation, collision detection would be more sophisticated
        if new_state.robot_position["z"] < 0:  # Simple ground collision
            new_state.robot_position["z"] = 0  # Don't go below ground
            velocity["z"] = -velocity["z"] * 0.8  # Bounce with damping
            collision_count += 1
            new_state.physics["collision_count"] = collision_count

        # Update simulation speed based on real-time factor
        new_state.physics["simulation_speed"] = dt / 0.01  # Normalize to 100Hz

        return new_state

    async def apply_forces(self, state: SimulationState, forces: Dict[str, Any]) -> SimulationState:
        """
        Apply external forces to the simulation state.

        Args:
            state: Current simulation state
            forces: Dictionary containing force information

        Returns:
            SimulationState: Updated state after applying forces
        """
        new_state = state.copy()

        # Apply linear forces
        linear_force = forces.get("linear", {"x": 0.0, "y": 0.0, "z": 0.0})

        # Get current velocity
        current_velocity = new_state.physics.get("velocity", {"x": 0.0, "y": 0.0, "z": 0.0})

        # Calculate acceleration (F = ma, so a = F/m, assuming mass = 1 for simplicity)
        acceleration = {
            "x": linear_force["x"],
            "y": linear_force["y"],
            "z": linear_force["z"]
        }

        # Update velocity: v = v0 + a*t
        dt = 0.01  # Time step
        new_velocity = {
            "x": current_velocity["x"] + acceleration["x"] * dt,
            "y": current_velocity["y"] + acceleration["y"] * dt,
            "z": current_velocity["z"] + acceleration["z"] * dt
        }

        new_state.physics["velocity"] = new_velocity

        # Apply angular forces (torques)
        angular_force = forces.get("angular", {"x": 0.0, "y": 0.0, "z": 0.0})
        current_angular_velocity = new_state.physics.get("angular_velocity", {"x": 0.0, "y": 0.0, "z": 0.0})

        # Update angular velocity
        new_angular_velocity = {
            "x": current_angular_velocity["x"] + angular_force["x"] * dt,
            "y": current_angular_velocity["y"] + angular_force["y"] * dt,
            "z": current_angular_velocity["z"] + angular_force["z"] * dt
        }

        new_state.physics["angular_velocity"] = new_angular_velocity

        # Update rotation based on angular velocity
        current_rotation = new_state.robot_position.get("rotation", {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0})
        # Simplified rotation update (in a real implementation, quaternion integration would be more complex)
        rotation_update = {
            "x": current_rotation["x"] + new_angular_velocity["x"] * dt * 0.1,  # Scale factor for quaternion
            "y": current_rotation["y"] + new_angular_velocity["y"] * dt * 0.1,
            "z": current_rotation["z"] + new_angular_velocity["z"] * dt * 0.1,
            "w": current_rotation["w"]  # Keep w component for now
        }

        # Normalize quaternion
        norm = (rotation_update["x"]**2 + rotation_update["y"]**2 + rotation_update["z"]**2 + rotation_update["w"]**2)**0.5
        if norm > 0:
            rotation_update = {k: v/norm for k, v in rotation_update.items()}

        new_state.robot_position["rotation"] = rotation_update

        return new_state

    async def calculate_collisions(self, state: SimulationState) -> List[Dict[str, Any]]:
        """
        Calculate potential collisions in the simulation environment.

        Args:
            state: Current simulation state

        Returns:
            List of collision information
        """
        collisions = []

        # Simplified collision detection
        # In a real implementation, this would check against environment objects
        robot_pos = state.robot_position

        # Check for ground collision
        if robot_pos["z"] <= 0:
            collisions.append({
                "type": "ground_collision",
                "position": robot_pos,
                "timestamp": datetime.now().isoformat()
            })

        # Check for boundary collisions (simplified)
        boundary_limit = 10.0
        if (abs(robot_pos["x"]) > boundary_limit or
            abs(robot_pos["y"]) > boundary_limit or
            robot_pos["z"] > boundary_limit):
            collisions.append({
                "type": "boundary_collision",
                "position": robot_pos,
                "timestamp": datetime.now().isoformat()
            })

        return collisions

    async def set_gravity(self, session_id: str, gravity: float) -> SimulationState:
        """
        Set the gravity constant for a specific simulation session.

        Args:
            session_id: ID of the simulation session
            gravity: Gravity value in m/s^2 (typically negative)

        Returns:
            SimulationState: Updated state with new gravity
        """
        # Get current state
        current_state = await digital_twin_service.get_simulation_state(session_id)

        # Create new state with updated gravity
        new_state = current_state.copy()
        new_state.physics["gravity"] = gravity

        # Update the session with new state
        # In a real implementation, we'd need to update the session directly
        # For now, we'll just return the new state
        return new_state

    async def get_physics_parameters(self, state: SimulationState) -> Dict[str, Any]:
        """
        Get current physics parameters from the simulation state.

        Args:
            state: Current simulation state

        Returns:
            Dictionary containing physics parameters
        """
        return {
            "gravity": state.physics.get("gravity", self.gravity_constant),
            "collision_count": state.physics.get("collision_count", 0),
            "simulation_speed": state.physics.get("simulation_speed", 1.0),
            "velocity": state.physics.get("velocity", {"x": 0.0, "y": 0.0, "z": 0.0}),
            "angular_velocity": state.physics.get("angular_velocity", {"x": 0.0, "y": 0.0, "z": 0.0}),
            "mass": state.physics.get("mass", 1.0),  # Default mass
            "friction": state.physics.get("friction", 0.5),  # Default friction
            "restitution": state.physics.get("restitution", 0.2)  # Default bounciness
        }

    async def apply_collision_response(self, state: SimulationState, collisions: List[Dict[str, Any]]) -> SimulationState:
        """
        Apply collision responses to the simulation state.

        Args:
            state: Current simulation state
            collisions: List of collision information

        Returns:
            SimulationState: Updated state after collision responses
        """
        new_state = state.copy()

        for collision in collisions:
            collision_type = collision["type"]

            if collision_type == "ground_collision":
                # Stop downward motion
                velocity = new_state.physics.get("velocity", {"x": 0.0, "y": 0.0, "z": 0.0})
                if velocity["z"] < 0:  # Moving downward
                    velocity["z"] = -velocity["z"] * 0.8  # Bounce with damping
                    new_state.physics["velocity"] = velocity

                    # Set position to ground level
                    new_state.robot_position["z"] = 0.0

            elif collision_type == "boundary_collision":
                # Reverse velocity components that caused the boundary collision
                velocity = new_state.physics.get("velocity", {"x": 0.0, "y": 0.0, "z": 0.0})

                # For simplicity, reverse all velocity components
                velocity["x"] = -velocity["x"] * 0.8
                velocity["y"] = -velocity["y"] * 0.8
                velocity["z"] = -velocity["z"] * 0.8

                new_state.physics["velocity"] = velocity

        return new_state


# Global instance of the service
physics_service = PhysicsService()