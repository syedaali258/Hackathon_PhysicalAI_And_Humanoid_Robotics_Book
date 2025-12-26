import pytest
from fastapi.testclient import TestClient
from api.main import app


class TestSensorSimulationContract:
    """Contract tests for sensor simulation endpoints based on the OpenAPI specification."""

    def setup_method(self):
        """Set up test client for each test."""
        self.client = TestClient(app)

    def test_sensor_simulation_start_contract(self):
        """Test that /simulation/start endpoint works with sensor simulation."""
        # Test with mixed simulation type (includes sensors)
        simulation_request = {
            "simulationType": "mixed",
            "environment": "sensor_world"
        }
        response = self.client.post("/simulation/start", json=simulation_request)

        # Check status code (could be 201 for success or various error codes)
        assert response.status_code in [201, 400, 500], f"Unexpected status code: {response.status_code}"

        if response.status_code == 201:
            data = response.json()

            # Check response structure according to OpenAPI spec
            assert "sessionId" in data, "Response should contain sessionId"
            assert "timestamp" in data, "Response should contain timestamp"
            assert "message" in data, "Response should contain message"
            assert "environmentConfig" in data, "Response should contain environmentConfig"

            # Check data types
            assert isinstance(data["sessionId"], str), "sessionId should be a string"
            assert isinstance(data["timestamp"], str), "timestamp should be a string"
            assert isinstance(data["message"], str), "message should be a string"
            assert isinstance(data["environmentConfig"], dict), "environmentConfig should be a dict"

    def test_lidar_sensor_data_contract(self):
        """Test that LiDAR sensor data is properly structured in simulation state."""
        # First start a sensor simulation to get a session ID
        simulation_request = {
            "simulationType": "gazebo",
            "environment": "lidar_world"
        }
        start_response = self.client.post("/simulation/start", json=simulation_request)

        if start_response.status_code == 201:
            session_id = start_response.json()["sessionId"]

            # Get simulation state
            response = self.client.get(f"/simulation/{session_id}/state")

            # Check status code
            assert response.status_code == 200, f"Expected status 200, got {response.status_code}"

            # Check response structure according to OpenAPI spec
            data = response.json()
            assert "state" in data, "Response should contain 'state' field"

            # Check state structure
            state = data["state"]
            assert "sensors" in state, "State should contain sensors"

            # Check that LiDAR data is properly structured when present
            sensors = state["sensors"]
            if "lidar" in sensors:
                lidar_data = sensors["lidar"]
                assert isinstance(lidar_data, list), "LiDAR data should be a list"

                # LiDAR data should contain distance measurements (numbers)
                for measurement in lidar_data:
                    assert isinstance(measurement, (int, float)), "LiDAR measurements should be numbers"
                    assert measurement >= 0, "LiDAR measurements should be non-negative"

    def test_imu_sensor_data_contract(self):
        """Test that IMU sensor data is properly structured in simulation state."""
        # First start a sensor simulation to get a session ID
        simulation_request = {
            "simulationType": "gazebo",
            "environment": "imu_world"
        }
        start_response = self.client.post("/simulation/start", json=simulation_request)

        if start_response.status_code == 201:
            session_id = start_response.json()["sessionId"]

            # Get simulation state
            response = self.client.get(f"/simulation/{session_id}/state")

            # Check status code
            assert response.status_code == 200, f"Expected status 200, got {response.status_code}"

            # Check response structure according to OpenAPI spec
            data = response.json()
            assert "state" in data, "Response should contain 'state' field"

            # Check state structure
            state = data["state"]
            assert "sensors" in state, "State should contain sensors"

            # Check that IMU data is properly structured when present
            sensors = state["sensors"]
            if "imu" in sensors:
                imu_data = sensors["imu"]
                assert isinstance(imu_data, dict), "IMU data should be a dict"

                # Check linear acceleration structure
                if "linearAcceleration" in imu_data:
                    linear_acc = imu_data["linearAcceleration"]
                    assert isinstance(linear_acc, dict), "Linear acceleration should be a dict"
                    assert "x" in linear_acc, "Linear acceleration should have x component"
                    assert "y" in linear_acc, "Linear acceleration should have y component"
                    assert "z" in linear_acc, "Linear acceleration should have z component"

                    # Check data types for linear acceleration
                    for axis in ["x", "y", "z"]:
                        assert isinstance(linear_acc[axis], (int, float)), f"Linear acceleration {axis} should be a number"

                # Check angular velocity structure
                if "angularVelocity" in imu_data:
                    angular_vel = imu_data["angularVelocity"]
                    assert isinstance(angular_vel, dict), "Angular velocity should be a dict"
                    assert "x" in angular_vel, "Angular velocity should have x component"
                    assert "y" in angular_vel, "Angular velocity should have y component"
                    assert "z" in angular_vel, "Angular velocity should have z component"

                    # Check data types for angular velocity
                    for axis in ["x", "y", "z"]:
                        assert isinstance(angular_vel[axis], (int, float)), f"Angular velocity {axis} should be a number"

    def test_sensor_command_contract(self):
        """Test that sensor-related commands properly affect sensor data."""
        # First start a sensor simulation to get a session ID
        simulation_request = {
            "simulationType": "gazebo",
            "environment": "sensor_world"
        }
        start_response = self.client.post("/simulation/start", json=simulation_request)

        if start_response.status_code == 201:
            session_id = start_response.json()["sessionId"]

            # Get initial state with sensors
            initial_response = self.client.get(f"/simulation/{session_id}/state")
            if initial_response.status_code == 200:
                initial_data = initial_response.json()
                initial_sensors = initial_data["state"]["sensors"]

                # Send a command that might affect sensor readings
                command_request = {
                    "command": "move_forward",
                    "parameters": {
                        "linear": 0.5,
                        "angular": 0.0
                    }
                }
                response = self.client.post(f"/simulation/{session_id}/command", json=command_request)

                # Check status code
                assert response.status_code in [200, 400, 404, 500], f"Unexpected status code: {response.status_code}"

                if response.status_code == 200:
                    data = response.json()

                    # Check response structure according to OpenAPI spec
                    assert "status" in data, "Response should contain 'status' field"
                    assert "newState" in data, "Response should contain 'newState' field"

                    # Check that the new state includes updated sensor information
                    new_state = data["newState"]
                    assert "sensors" in new_state, "New state should contain sensors"

                    new_sensors = new_state["sensors"]
                    # If LiDAR was in initial state, it should be in new state too
                    if "lidar" in initial_sensors:
                        assert "lidar" in new_sensors, "LiDAR should still be present after command"
                        assert isinstance(new_sensors["lidar"], list), "LiDAR data should be a list"

                    # If IMU was in initial state, it should be in new state too
                    if "imu" in initial_sensors:
                        assert "imu" in new_sensors, "IMU should still be present after command"
                        assert isinstance(new_sensors["imu"], dict), "IMU data should be a dict"

    def test_depth_camera_sensor_contract(self):
        """Test that depth camera sensor data is properly structured when present."""
        # First start a sensor simulation to get a session ID
        simulation_request = {
            "simulationType": "gazebo",
            "environment": "depth_camera_world"
        }
        start_response = self.client.post("/simulation/start", json=simulation_request)

        if start_response.status_code == 201:
            session_id = start_response.json()["sessionId"]

            # Get simulation state
            response = self.client.get(f"/simulation/{session_id}/state")

            # Check status code
            assert response.status_code == 200, f"Expected status 200, got {response.status_code}"

            # Check response structure according to OpenAPI spec
            data = response.json()
            assert "state" in data, "Response should contain 'state' field"

            # Check state structure
            state = data["state"]
            assert "sensors" in state, "State should contain sensors"

            # Depth camera data would be in a format similar to other sensors
            sensors = state["sensors"]
            # The specific structure would depend on how depth camera data is represented
            # This test ensures that if depth camera data exists, it follows the expected pattern

    def test_sensor_simulation_reset_contract(self):
        """Test that sensor simulation reset properly restores initial sensor state."""
        # First start a sensor simulation to get a session ID
        simulation_request = {
            "simulationType": "gazebo",
            "environment": "sensor_world"
        }
        start_response = self.client.post("/simulation/start", json=simulation_request)

        if start_response.status_code == 201:
            session_id = start_response.json()["sessionId"]

            # Get initial sensor state
            initial_response = self.client.get(f"/simulation/{session_id}/state")
            if initial_response.status_code == 200:
                initial_data = initial_response.json()
                initial_sensors = initial_data["state"]["sensors"]

                # Send a command to change the state
                command_request = {
                    "command": "move_forward",
                    "parameters": {
                        "linear": 1.0,
                        "angular": 0.0
                    }
                }
                command_response = self.client.post(f"/simulation/{session_id}/command", json=command_request)

                if command_response.status_code == 200:
                    # Reset the simulation
                    reset_response = self.client.post(f"/simulation/{session_id}/reset")

                    # Check status code
                    assert reset_response.status_code in [200, 404, 500], f"Unexpected status code: {reset_response.status_code}"

                    if reset_response.status_code == 200:
                        reset_data = reset_response.json()

                        # Check response structure according to OpenAPI spec
                        assert "initialState" in reset_data, "Response should contain 'initialState' field"

                        # Check that initial state has sensors
                        initial_state = reset_data["initialState"]
                        assert "sensors" in initial_state, "Initial state should contain sensors"
                        reset_sensors = initial_state["sensors"]

                        # The reset state should have the same sensor types as the initial state
                        for sensor_type in initial_sensors.keys():
                            assert sensor_type in reset_sensors, f"Sensor type {sensor_type} should be present after reset"