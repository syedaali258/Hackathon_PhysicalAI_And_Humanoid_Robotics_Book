"""
Unity Service for the AI/Robotics Book API

This service handles Unity interaction operations for human-robot interaction
in the Module 2 content on The Digital Twin (Gazebo & Unity).
"""

from typing import Dict, Any, Optional, List
from datetime import datetime
import uuid
import math
import logging
from api.models.simulation import SimulationState, SimulationCommand
from api.services.digital_twin_service import digital_twin_service
from api.services.sensor_service import sensor_service
from api.utils.errors import SimulationException


class UnityService:
    """
    Service class for handling Unity interaction operations.
    """

    def __init__(self):
        self.logger = logging.getLogger(__name__)
        self.unity_sessions: Dict[str, Dict[str, Any]] = {}

    async def create_unity_session(self, session_id: str, user_id: Optional[str] = None) -> Dict[str, Any]:
        """
        Create a Unity-specific session with visualization and interaction parameters.

        Args:
            session_id: ID of the existing simulation session
            user_id: Optional user identifier

        Returns:
            Dictionary containing Unity session information
        """
        # Verify the simulation session exists
        try:
            await digital_twin_service.get_simulation_state(session_id)
        except SimulationException:
            raise SimulationException(f"Simulation session {session_id} does not exist")

        # Create Unity-specific session data
        unity_session_data = {
            "session_id": session_id,
            "user_id": user_id,
            "visualization_settings": {
                "render_quality": "high",
                "shadows": True,
                "post_processing": True,
                "lod_bias": 1.0
            },
            "interaction_modes": {
                "teleoperation": True,
                "waypoint_navigation": True,
                "gesture_control": False,
                "voice_control": False
            },
            "camera_settings": {
                "fov": 60,
                "near_clip": 0.1,
                "far_clip": 1000.0,
                "follow_robot": True
            },
            "ui_settings": {
                "show_sensors": True,
                "show_collision_meshes": False,
                "show_trajectories": True,
                "show_debug_info": False
            },
            "created_at": datetime.now().isoformat(),
            "last_updated": datetime.now().isoformat()
        }

        self.unity_sessions[session_id] = unity_session_data

        self.logger.info(
            f"Created Unity session",
            extra={"session_id": session_id, "user_id": user_id}
        )

        return unity_session_data

    async def get_visualization_data(self, session_id: str) -> Dict[str, Any]:
        """
        Get data formatted for Unity visualization.

        Args:
            session_id: ID of the simulation session

        Returns:
            Dictionary containing data formatted for Unity visualization
        """
        if session_id not in self.unity_sessions:
            await self.create_unity_session(session_id)  # Auto-create if needed

        # Get current simulation state
        sim_state = await digital_twin_service.get_simulation_state(session_id)

        # Format data for Unity
        unity_data = {
            "robot_position": {
                "x": sim_state.robot_position["x"],
                "y": sim_state.robot_position["y"],
                "z": sim_state.robot_position["z"]
            },
            "robot_rotation": {
                "x": sim_state.robot_position["rotation"]["x"],
                "y": sim_state.robot_position["rotation"]["y"],
                "z": sim_state.robot_position["rotation"]["z"],
                "w": sim_state.robot_position["rotation"]["w"]
            },
            "sensors": await self._format_sensors_for_unity(sim_state.sensors),
            "physics": sim_state.physics,
            "timestamp": sim_state.timestamp.isoformat() if sim_state.timestamp else datetime.now().isoformat()
        }

        # Update last access time
        if session_id in self.unity_sessions:
            self.unity_sessions[session_id]["last_updated"] = datetime.now().isoformat()

        return unity_data

    async def _format_sensors_for_unity(self, sensors: Dict[str, Any]) -> Dict[str, Any]:
        """
        Format sensor data for Unity consumption.

        Args:
            sensors: Raw sensor data

        Returns:
            Dictionary with Unity-formatted sensor data
        """
        formatted_sensors = {}

        # Format LiDAR data for Unity point cloud
        if "lidar" in sensors:
            lidar_data = sensors["lidar"]
            # Convert to Unity coordinate system if needed
            # For now, just pass through with metadata
            formatted_sensors["lidar"] = {
                "readings": lidar_data,
                "point_cloud": await self._generate_point_cloud(lidar_data),
                "timestamp": sensors.get("timestamp", datetime.now().isoformat())
            }

        # Format IMU data for Unity
        if "imu" in sensors:
            imu_data = sensors["imu"]
            formatted_sensors["imu"] = {
                "linear_acceleration": imu_data.get("linear_acceleration", {"x": 0.0, "y": 0.0, "z": 0.0}),
                "angular_velocity": imu_data.get("angular_velocity", {"x": 0.0, "y": 0.0, "z": 0.0}),
                "orientation": imu_data.get("orientation", {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}),
                "timestamp": imu_data.get("timestamp", datetime.now().isoformat())
            }

        # Format camera data for Unity
        if "depth_camera" in sensors:
            camera_data = sensors["depth_camera"]
            formatted_sensors["camera"] = {
                "rgb": camera_data.get("rgb"),
                "depth": camera_data.get("depth"),
                "width": camera_data.get("width", 640),
                "height": camera_data.get("height", 480),
                "fov": camera_data.get("fov", 60),
                "timestamp": sensors.get("timestamp", datetime.now().isoformat())
            }

        return formatted_sensors

    async def _generate_point_cloud(self, lidar_readings: List[float]) -> List[Dict[str, float]]:
        """
        Generate a simple point cloud from LiDAR readings.

        Args:
            lidar_readings: List of distance measurements

        Returns:
            List of points in 3D space
        """
        points = []
        for i, distance in enumerate(lidar_readings):
            # Convert index to angle
            angle = (2 * 3.14159 * i) / len(lidar_readings)
            # Convert polar to Cartesian coordinates
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            z = 0.0  # For simplicity, assume 2D scan at z=0

            points.append({
                "x": x,
                "y": y,
                "z": z
            })

        return points

    async def process_unity_command(self, session_id: str, command: str, parameters: Dict[str, Any]) -> Dict[str, Any]:
        """
        Process a command from the Unity interface.

        Args:
            session_id: ID of the simulation session
            command: Command string from Unity
            parameters: Command parameters

        Returns:
            Dictionary containing command execution result
        """
        if session_id not in self.unity_sessions:
            raise SimulationException(f"Unity session {session_id} not initialized")

        # Map Unity commands to simulation commands
        command_mapping = {
            "move_robot": "set_velocity",
            "rotate_robot": "set_velocity",  # Combined with move for rotation
            "set_waypoint": "navigate_to",
            "toggle_follow_camera": "camera_follow",
            "request_sensor_data": "get_sensors",
            "emergency_stop": "stop_robot"
        }

        mapped_command = command_mapping.get(command, command)

        # Prepare simulation command
        sim_command = SimulationCommand(
            command=mapped_command,
            parameters=parameters,
            timestamp=datetime.now()
        )

        try:
            # Execute the command in the simulation
            new_state = await digital_twin_service.execute_command(session_id, sim_command)

            result = {
                "status": "success",
                "command": command,
                "mapped_command": mapped_command,
                "new_state": await self.get_visualization_data(session_id),
                "timestamp": datetime.now().isoformat()
            }

            self.logger.info(
                f"Processed Unity command",
                extra={
                    "session_id": session_id,
                    "command": command,
                    "status": "success"
                }
            )

            return result

        except Exception as e:
            self.logger.error(
                f"Error processing Unity command",
                extra={
                    "session_id": session_id,
                    "command": command,
                    "error": str(e)
                },
                exc_info=True
            )

            return {
                "status": "error",
                "command": command,
                "error": str(e),
                "timestamp": datetime.now().isoformat()
            }

    async def update_unity_ui_data(self, session_id: str) -> Dict[str, Any]:
        """
        Get data specifically formatted for Unity UI elements.

        Args:
            session_id: ID of the simulation session

        Returns:
            Dictionary containing UI-specific data
        """
        # Get current simulation state
        sim_state = await digital_twin_service.get_simulation_state(session_id)

        # Get sensor data
        sensor_readings = await sensor_service.update_sensor_readings(sim_state)

        ui_data = {
            "robot_status": {
                "position": sim_state.robot_position,
                "velocity": sim_state.physics.get("velocity", {"x": 0.0, "y": 0.0, "z": 0.0}),
                "battery_level": self._calculate_battery_level(session_id),
                "connection_status": "connected",
                "operational_mode": "autonomous"  # Could be teleop, autonomous, etc.
            },
            "sensor_status": {
                "lidar_operational": True,
                "camera_operational": True,
                "imu_operational": True,
                "last_sensor_update": sensor_readings.get("timestamp", datetime.now().isoformat())
            },
            "environment_data": {
                "detected_obstacles": await self._get_detected_obstacles(sensor_readings),
                "traversability_map": await self._get_traversability_map(sim_state),
                "waypoints": await self._get_active_waypoints(session_id)
            },
            "performance_metrics": {
                "simulation_speed": sim_state.physics.get("simulation_speed", 1.0),
                "frame_rate": 60.0,  # Would come from Unity
                "latency": 0.01  # Network latency estimate
            },
            "timestamp": datetime.now().isoformat()
        }

        return ui_data

    def _calculate_battery_level(self, session_id: str) -> float:
        """
        Calculate simulated battery level for the robot.

        Args:
            session_id: ID of the simulation session

        Returns:
            Battery level as a float between 0 and 1
        """
        # Simulate battery drain over time
        # In a real implementation, this would track actual power consumption
        import time
        base_time = hash(session_id) % 10000  # Deterministic but session-specific
        elapsed_minutes = (time.time() - base_time) / 60
        battery_level = max(0.0, 1.0 - (elapsed_minutes / 120.0))  # 2-hour battery life
        return round(battery_level, 2)

    async def _get_detected_obstacles(self, sensor_readings: Dict[str, Any]) -> List[Dict[str, float]]:
        """
        Extract obstacle information from sensor readings.

        Args:
            sensor_readings: Current sensor readings

        Returns:
            List of detected obstacles
        """
        obstacles = []

        # Extract from LiDAR data
        if "lidar" in sensor_readings:
            lidar_data = sensor_readings["lidar"]
            for i, distance in enumerate(lidar_data.get("readings", [])):
                if distance < 3.0:  # Obstacle within 3m
                    angle = (2 * 3.14159 * i) / len(lidar_data["readings"])
                    obstacle = {
                        "bearing": angle,
                        "distance": distance,
                        "certainty": 0.9  # High certainty for simulated data
                    }
                    obstacles.append(obstacle)

        return obstacles

    async def _get_traversability_map(self, state: SimulationState) -> List[List[float]]:
        """
        Generate a simple traversability map based on simulation state.

        Args:
            state: Current simulation state

        Returns:
            2D grid representing traversability (0 = impassable, 1 = free)
        """
        # Create a 10x10 grid around the robot
        grid_size = 10
        grid = [[1.0 for _ in range(grid_size)] for _ in range(grid_size)]

        robot_x, robot_y = state.robot_position["x"], state.robot_position["y"]

        # Add some simulated obstacles
        for i in range(grid_size):
            for j in range(grid_size):
                grid_x = robot_x - grid_size/2 + i
                grid_y = robot_y - grid_size/2 + j

                # Add circular obstacles
                if math.sqrt((grid_x - robot_x)**2 + (grid_y - robot_y)**2) < 1.0:
                    grid[i][j] = 0.0  # Obstacle

        return grid

    async def _get_active_waypoints(self, session_id: str) -> List[Dict[str, float]]:
        """
        Get active waypoints for the robot.

        Args:
            session_id: ID of the simulation session

        Returns:
            List of waypoints
        """
        # In a real implementation, this would track actual waypoints
        # For now, return empty list
        return []

    async def update_visualization_settings(self, session_id: str, settings: Dict[str, Any]) -> Dict[str, Any]:
        """
        Update Unity visualization settings.

        Args:
            session_id: ID of the Unity session
            settings: New visualization settings

        Returns:
            Updated settings
        """
        if session_id not in self.unity_sessions:
            raise SimulationException(f"Unity session {session_id} not found")

        # Update the settings
        current_settings = self.unity_sessions[session_id]["visualization_settings"]
        current_settings.update(settings)

        # Update other settings too
        if "interaction_modes" in settings:
            self.unity_sessions[session_id]["interaction_modes"].update(settings["interaction_modes"])

        if "camera_settings" in settings:
            self.unity_sessions[session_id]["camera_settings"].update(settings["camera_settings"])

        if "ui_settings" in settings:
            self.unity_sessions[session_id]["ui_settings"].update(settings["ui_settings"])

        self.unity_sessions[session_id]["last_updated"] = datetime.now().isoformat()

        self.logger.info(
            f"Updated Unity visualization settings",
            extra={"session_id": session_id}
        )

        return {
            "visualization_settings": self.unity_sessions[session_id]["visualization_settings"],
            "interaction_modes": self.unity_sessions[session_id]["interaction_modes"],
            "camera_settings": self.unity_sessions[session_id]["camera_settings"],
            "ui_settings": self.unity_sessions[session_id]["ui_settings"]
        }

    async def get_interaction_capabilities(self, session_id: str) -> Dict[str, Any]:
        """
        Get available interaction capabilities for the Unity interface.

        Args:
            session_id: ID of the Unity session

        Returns:
            Dictionary of available interaction capabilities
        """
        if session_id not in self.unity_sessions:
            raise SimulationException(f"Unity session {session_id} not found")

        capabilities = {
            "teleoperation": {
                "enabled": True,
                "controls": ["move_forward", "move_backward", "turn_left", "turn_right", "stop"],
                "max_linear_speed": 1.0,
                "max_angular_speed": 1.0
            },
            "navigation": {
                "enabled": True,
                "methods": ["waypoint", "path_planning", "direct_position"],
                "max_waypoints": 10
            },
            "sensor_control": {
                "enabled": True,
                "controllable_sensors": ["lidar", "camera", "imu"],
                "calibration_available": True
            },
            "monitoring": {
                "enabled": True,
                "data_streams": ["robot_state", "sensor_data", "environment_map"],
                "update_frequency": 30  # Hz
            },
            "safety": {
                "enabled": True,
                "emergency_stop": True,
                "collision_avoidance": True,
                "geofence": True
            }
        }

        return capabilities


# Global instance of the service
unity_service = UnityService()