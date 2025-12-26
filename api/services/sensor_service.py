"""
Sensor Service for the AI/Robotics Book API

This service handles sensor simulation operations for LiDAR, depth cameras, and IMUs
in the Module 2 content on The Digital Twin (Gazebo & Unity).
"""

from typing import Dict, Any, Optional, List
from datetime import datetime
import uuid
import random
import math
import logging
from api.models.simulation import SimulationState
from api.services.digital_twin_service import digital_twin_service
from api.utils.errors import SimulationException


class SensorService:
    """
    Service class for handling sensor simulation operations.
    """

    def __init__(self):
        self.logger = logging.getLogger(__name__)

    async def generate_lidar_data(self, state: SimulationState, params: Optional[Dict[str, Any]] = None) -> List[float]:
        """
        Generate simulated LiDAR data based on the current simulation state.

        Args:
            state: Current simulation state
            params: Optional parameters for LiDAR configuration

        Returns:
            List of distance measurements from LiDAR scan
        """
        if params is None:
            params = {}

        # Default LiDAR parameters
        samples = params.get("samples", 360)
        max_range = params.get("max_range", 30.0)
        min_range = params.get("min_range", 0.1)
        noise_stddev = params.get("noise_stddev", 0.01)

        # Get robot position to determine what the LiDAR would "see"
        robot_pos = state.robot_position

        # Generate simulated LiDAR data
        # In a real implementation, this would ray-cast into the environment
        lidar_data = []
        for i in range(samples):
            # Calculate angle for this sample
            angle = (2 * math.pi * i) / samples

            # Simulate distance measurement with some basic environment
            # This is a simplified model - in reality, this would involve raycasting
            # against environment objects
            distance = max_range  # Default to max range (no obstacle detected)

            # Add some basic environmental features (walls, objects)
            # For demonstration, let's add some virtual obstacles
            if 1.0 < (i % 90) < 10:  # Create some "walls" at regular intervals
                distance = 2.0 + random.uniform(-0.1, 0.1)

            # Add noise to make it more realistic
            noisy_distance = distance + random.gauss(0, noise_stddev)
            # Ensure distance is within valid range
            noisy_distance = max(min_range, min(max_range, noisy_distance))

            lidar_data.append(noisy_distance)

        return lidar_data

    async def generate_depth_camera_data(self, state: SimulationState, params: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """
        Generate simulated depth camera data based on the current simulation state.

        Args:
            state: Current simulation state
            params: Optional parameters for depth camera configuration

        Returns:
            Dictionary containing RGB image and depth map data
        """
        if params is None:
            params = {}

        # Default depth camera parameters
        width = params.get("width", 640)
        height = params.get("height", 480)
        fov = params.get("fov", 60)  # Field of view in degrees

        # Generate simulated depth map
        # In a real implementation, this would involve raycasting or rendering
        depth_map = []
        for y in range(height):
            row = []
            for x in range(width):
                # Calculate normalized coordinates
                nx = (x - width / 2) / (width / 2)
                ny = (y - height / 2) / (height / 2)

                # Calculate angle from center
                angle_x = math.radians(fov / 2) * nx
                angle_y = math.radians(fov / 2) * ny

                # Simulate depth based on angle and environment
                # This is a simplified model
                depth = 10.0  # Default depth

                # Add some variation based on position
                depth += math.sin(angle_x * 5) * 2 + math.cos(angle_y * 3) * 1.5

                # Add some noise
                noise = random.gauss(0, 0.05)
                depth = max(0.1, depth + noise)  # Ensure positive depth

                row.append(depth)
            depth_map.append(row)

        # Generate a simple RGB image (grayscale for simplicity)
        rgb_image = []
        for y in range(height):
            row = []
            for x in range(width):
                # Create a pattern based on depth
                intensity = min(255, int(depth_map[y][x] * 25))  # Scale depth to grayscale
                pixel = [intensity, intensity, intensity]  # RGB values
                row.append(pixel)
            rgb_image.append(row)

        return {
            "rgb": rgb_image,
            "depth": depth_map,
            "width": width,
            "height": height,
            "fov": fov
        }

    async def generate_imu_data(self, state: SimulationState, params: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """
        Generate simulated IMU data based on the current simulation state.

        Args:
            state: Current simulation state
            params: Optional parameters for IMU configuration

        Returns:
            Dictionary containing IMU measurements
        """
        if params is None:
            params = {}

        # Default IMU parameters
        noise_params = params.get("noise", {
            "accelerometer_stddev": 0.017,  # ~0.1 deg/s for gyroscope
            "gyroscope_stddev": 0.0017,
            "magnetometer_stddev": 0.1
        })

        # Get physics information from the state
        physics = state.physics

        # Simulate linear acceleration
        # In a real implementation, this would consider actual forces on the robot
        linear_acceleration = physics.get("velocity", {"x": 0.0, "y": 0.0, "z": 0.0}).copy()

        # Add gravity to the Z component
        linear_acceleration["z"] += abs(physics.get("gravity", -9.81))

        # Add noise to make it realistic
        linear_acceleration["x"] += random.gauss(0, noise_params["accelerometer_stddev"])
        linear_acceleration["y"] += random.gauss(0, noise_params["accelerometer_stddev"])
        linear_acceleration["z"] += random.gauss(0, noise_params["accelerometer_stddev"])

        # Simulate angular velocity
        angular_velocity = physics.get("angular_velocity", {"x": 0.0, "y": 0.0, "z": 0.0}).copy()

        # Add noise
        angular_velocity["x"] += random.gauss(0, noise_params["gyroscope_stddev"])
        angular_velocity["y"] += random.gauss(0, noise_params["gyroscope_stddev"])
        angular_velocity["z"] += random.gauss(0, noise_params["gyroscope_stddev"])

        # Simulate magnetometer data (simplified)
        # In reality, this would depend on location and surrounding magnetic fields
        magnetometer = {
            "x": 25.0 + random.gauss(0, noise_params["magnetometer_stddev"]),
            "y": 0.0 + random.gauss(0, noise_params["magnetometer_stddev"]),
            "z": 42.0 + random.gauss(0, noise_params["magnetometer_stddev"])
        }

        # Simulate orientation (simplified - in reality, this would be integrated from angular velocity)
        rotation = state.robot_position.get("rotation", {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0})

        return {
            "linear_acceleration": linear_acceleration,
            "angular_velocity": angular_velocity,
            "magnetometer": magnetometer,
            "orientation": rotation,
            "timestamp": datetime.now().isoformat()
        }

    async def update_sensor_readings(self, state: SimulationState) -> Dict[str, Any]:
        """
        Update all sensor readings based on the current simulation state.

        Args:
            state: Current simulation state

        Returns:
            Dictionary containing all sensor readings
        """
        # Generate data for all sensor types
        lidar_data = await self.generate_lidar_data(state)
        depth_camera_data = await self.generate_depth_camera_data(state)
        imu_data = await self.generate_imu_data(state)

        # Combine all sensor data
        sensor_readings = {
            "lidar": lidar_data,
            "depth_camera": depth_camera_data,
            "imu": imu_data,
            "timestamp": datetime.now().isoformat()
        }

        # Update the state with sensor readings
        new_state = state.copy()
        new_state.sensors = sensor_readings

        return sensor_readings

    async def add_sensor_noise(self, sensor_data: Any, sensor_type: str, params: Optional[Dict[str, Any]] = None) -> Any:
        """
        Add realistic noise to sensor data based on sensor type.

        Args:
            sensor_data: Raw sensor data
            sensor_type: Type of sensor ('lidar', 'camera', 'imu', etc.)
            params: Optional noise parameters

        Returns:
            Sensor data with added noise
        """
        if params is None:
            params = {}

        if sensor_type == "lidar":
            noise_stddev = params.get("noise_stddev", 0.01)
            if isinstance(sensor_data, list):
                return [reading + random.gauss(0, noise_stddev) for reading in sensor_data]
        elif sensor_type == "imu":
            noise_params = params.get("noise", {
                "accelerometer_stddev": 0.017,
                "gyroscope_stddev": 0.0017
            })

            if isinstance(sensor_data, dict):
                noisy_data = sensor_data.copy()
                # Add noise to linear acceleration
                if "linear_acceleration" in noisy_data:
                    la = noisy_data["linear_acceleration"]
                    la["x"] += random.gauss(0, noise_params["accelerometer_stddev"])
                    la["y"] += random.gauss(0, noise_params["accelerometer_stddev"])
                    la["z"] += random.gauss(0, noise_params["accelerometer_stddev"])

                # Add noise to angular velocity
                if "angular_velocity" in noisy_data:
                    av = noisy_data["angular_velocity"]
                    av["x"] += random.gauss(0, noise_params["gyroscope_stddev"])
                    av["y"] += random.gauss(0, noise_params["gyroscope_stddev"])
                    av["z"] += random.gauss(0, noise_params["gyroscope_stddev"])

                return noisy_data
        elif sensor_type == "camera":
            # For camera data, noise could include pixel noise, quantization, etc.
            noise_level = params.get("noise_level", 0.01)
            # This is a simplified example - real camera noise is more complex
            return sensor_data  # Return as is for this example

        return sensor_data

    async def simulate_sensor_fusion(self, state: SimulationState) -> Dict[str, Any]:
        """
        Simulate basic sensor fusion by combining data from multiple sensors.

        Args:
            state: Current simulation state with sensor data

        Returns:
            Dictionary containing fused sensor information
        """
        sensors = state.sensors

        fused_data = {
            "position_estimate": None,
            "velocity_estimate": None,
            "orientation_estimate": None,
            "environment_map": None,
            "timestamp": datetime.now().isoformat()
        }

        # If we have IMU data, we can estimate orientation
        if "imu" in sensors:
            imu_data = sensors["imu"]
            fused_data["orientation_estimate"] = imu_data.get("orientation")

        # If we have LiDAR data, we can create a basic environment map
        if "lidar" in sensors:
            lidar_data = sensors["lidar"]
            # Simplified: create a basic representation of nearby obstacles
            nearby_obstacles = []
            for i, distance in enumerate(lidar_data):
                if distance < 5.0:  # Obstacle within 5m
                    angle = (2 * math.pi * i) / len(lidar_data)
                    obstacle = {
                        "angle": angle,
                        "distance": distance,
                        "x": distance * math.cos(angle),
                        "y": distance * math.sin(angle)
                    }
                    nearby_obstacles.append(obstacle)

            fused_data["environment_map"] = nearby_obstacles

        # If we have both IMU and position data, we can estimate velocity
        if "imu" in sensors and "linear_acceleration" in sensors["imu"]:
            # Simplified velocity estimation by integrating acceleration
            # In reality, this would require more sophisticated filtering
            accel = sensors["imu"]["linear_acceleration"]
            # For now, just return the acceleration as a simple estimate
            fused_data["velocity_estimate"] = {
                "x": accel["x"] * 0.1,  # Scale factor
                "y": accel["y"] * 0.1,
                "z": accel["z"] * 0.1
            }

        return fused_data

    async def get_sensor_configuration(self, sensor_type: str) -> Dict[str, Any]:
        """
        Get default configuration for a specific sensor type.

        Args:
            sensor_type: Type of sensor

        Returns:
            Dictionary containing default configuration
        """
        configurations = {
            "lidar": {
                "samples": 360,
                "min_range": 0.1,
                "max_range": 30.0,
                "resolution": 1,
                "noise_stddev": 0.01
            },
            "depth_camera": {
                "width": 640,
                "height": 480,
                "fov": 60,
                "min_range": 0.1,
                "max_range": 10.0,
                "noise_level": 0.05
            },
            "imu": {
                "accelerometer_stddev": 0.017,
                "gyroscope_stddev": 0.0017,
                "magnetometer_stddev": 0.1,
                "update_rate": 100
            }
        }

        return configurations.get(sensor_type, {})


# Global instance of the service
sensor_service = SensorService()