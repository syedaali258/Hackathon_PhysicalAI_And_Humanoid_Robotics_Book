"""
Digital Twin Service for the AI/Robotics Book API

This service handles digital twin concepts and foundational simulation operations
for the Module 2 content on The Digital Twin (Gazebo & Unity).
"""

from typing import Dict, Any, Optional, List
from datetime import datetime
import uuid
import logging
from api.models.simulation import SimulationSession, SimulationState, SimulationCommand
from api.utils.errors import SimulationException


class DigitalTwinService:
    """
    Service class for handling digital twin concepts and foundational simulation operations.
    """

    def __init__(self):
        self.logger = logging.getLogger(__name__)
        self.sessions: Dict[str, SimulationSession] = {}

    async def create_simulation_session(self, user_id: Optional[str], simulation_type: str, environment: str) -> SimulationSession:
        """
        Create a new digital twin simulation session.

        Args:
            user_id: Optional user identifier
            simulation_type: Type of simulation (gazebo, unity, mixed)
            environment: Specific simulation environment name

        Returns:
            SimulationSession: Created session with initial state
        """
        session_id = str(uuid.uuid4())

        # Validate simulation type
        valid_types = ["gazebo", "unity", "mixed"]
        if simulation_type not in valid_types:
            raise SimulationException(f"Invalid simulation type. Must be one of: {valid_types}")

        # Create initial simulation state
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
            }
        )

        session = SimulationSession(
            id=session_id,
            user_id=user_id,
            simulation_type=simulation_type,
            environment=environment,
            state=initial_state,
            created_at=datetime.now(),
            last_updated=datetime.now()
        )

        self.sessions[session_id] = session

        self.logger.info(
            f"Created new simulation session",
            extra={
                "session_id": session_id,
                "simulation_type": simulation_type,
                "environment": environment
            }
        )

        return session

    async def get_simulation_state(self, session_id: str) -> SimulationState:
        """
        Get the current state of a simulation session.

        Args:
            session_id: Unique identifier for the simulation session

        Returns:
            SimulationState: Current state of the simulation
        """
        if session_id not in self.sessions:
            raise SimulationException(f"Session {session_id} not found")

        session = self.sessions[session_id]
        session.last_updated = datetime.now()

        return session.state

    async def execute_command(self, session_id: str, command: SimulationCommand) -> SimulationState:
        """
        Execute a command in the simulation and return the new state.

        Args:
            session_id: Unique identifier for the simulation session
            command: Command to execute in the simulation

        Returns:
            SimulationState: Updated state after command execution
        """
        if session_id not in self.sessions:
            raise SimulationException(f"Session {session_id} not found")

        session = self.sessions[session_id]

        # Log the command execution
        self.logger.info(
            f"Executing command in simulation",
            extra={
                "session_id": session_id,
                "command": command.command,
                "parameters": command.parameters
            }
        )

        # Apply the command to the simulation state
        new_state = await self._apply_command_to_state(session.state, command)

        # Update the session with the new state
        session.state = new_state
        session.last_updated = datetime.now()

        return new_state

    async def reset_simulation(self, session_id: str) -> SimulationState:
        """
        Reset the simulation to its initial state.

        Args:
            session_id: Unique identifier for the simulation session

        Returns:
            SimulationState: Initial state of the simulation
        """
        if session_id not in self.sessions:
            raise SimulationException(f"Session {session_id} not found")

        session = self.sessions[session_id]

        # Reset to initial state based on environment
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
            }
        )

        session.state = initial_state
        session.last_updated = datetime.now()

        self.logger.info(
            f"Reset simulation session",
            extra={"session_id": session_id}
        )

        return initial_state

    async def end_simulation(self, session_id: str) -> Dict[str, Any]:
        """
        End a simulation session and clean up resources.

        Args:
            session_id: Unique identifier for the simulation session

        Returns:
            Dict: Session summary information
        """
        if session_id not in self.sessions:
            raise SimulationException(f"Session {session_id} not found")

        session = self.sessions[session_id]

        # Calculate session duration
        duration = (datetime.now() - session.created_at).total_seconds()

        # Remove the session
        del self.sessions[session_id]

        self.logger.info(
            f"Ended simulation session",
            extra={
                "session_id": session_id,
                "duration": duration
            }
        )

        return {
            "message": "Simulation session ended successfully",
            "duration": int(duration)
        }

    async def _apply_command_to_state(self, current_state: SimulationState, command: SimulationCommand) -> SimulationState:
        """
        Apply a command to the current simulation state and return the new state.

        Args:
            current_state: Current simulation state
            command: Command to apply

        Returns:
            SimulationState: New state after applying the command
        """
        # Create a copy of the current state
        new_state = current_state.copy()

        command_type = command.command
        parameters = command.parameters or {}

        if command_type == "move_forward":
            # Move robot forward by a small amount
            linear_speed = parameters.get("linear", 0.1)
            new_state.robot_position["x"] += linear_speed * 0.1  # Scale by time step

        elif command_type == "turn_left":
            # Simple rotation
            angular_speed = parameters.get("angular", 0.1)
            # This is a simplified rotation model
            new_state.robot_position["rotation"]["z"] += angular_speed * 0.1

        elif command_type == "turn_right":
            # Simple rotation
            angular_speed = parameters.get("angular", 0.1)
            new_state.robot_position["rotation"]["z"] -= angular_speed * 0.1

        elif command_type == "set_velocity":
            # For a more sophisticated simulation, this would integrate velocity over time
            linear = parameters.get("linear", 0.0)
            angular = parameters.get("angular", 0.0)

            # Update position based on velocity (simplified)
            new_state.robot_position["x"] += linear * 0.01  # Small time step
            new_state.robot_position["rotation"]["z"] += angular * 0.01

        elif command_type == "interact_object":
            # Handle object interaction (for Unity scenarios)
            object_id = parameters.get("object_id", "")
            interaction_type = parameters.get("interaction_type", "")

            # Log the interaction
            self.logger.debug(
                f"Object interaction",
                extra={
                    "object_id": object_id,
                    "interaction_type": interaction_type
                }
            )

        # Update the timestamp
        new_state.timestamp = datetime.now()

        return new_state


# Global instance of the service
digital_twin_service = DigitalTwinService()