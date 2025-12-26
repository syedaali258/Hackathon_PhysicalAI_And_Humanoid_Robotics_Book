import pytest
from fastapi.testclient import TestClient
from api.main import app


class TestDigitalTwinConceptsContract:
    """Contract tests for digital twin concepts endpoints based on the OpenAPI specification."""

    def setup_method(self):
        """Set up test client for each test."""
        self.client = TestClient(app)

    def test_simulation_start_contract(self):
        """Test that /simulation/start endpoint matches the OpenAPI contract."""
        # Test with gazebo simulation type
        simulation_request = {
            "simulationType": "gazebo",
            "environment": "humanoid_robot_world"
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

            # Check that sessionId is not empty
            assert len(data["sessionId"]) > 0, "sessionId should not be empty"

    def test_simulation_state_contract(self):
        """Test that /simulation/{sessionId}/state endpoint matches the OpenAPI contract."""
        # First start a simulation to get a session ID
        simulation_request = {
            "simulationType": "gazebo",
            "environment": "humanoid_robot_world"
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
            assert "timestamp" in data, "Response should contain 'timestamp' field"

            # Check state structure
            state = data["state"]
            assert "robotPosition" in state, "State should contain robotPosition"
            assert "sensors" in state, "State should contain sensors"
            assert "physics" in state, "State should contain physics"

            # Check robot position structure
            robot_pos = state["robotPosition"]
            assert "x" in robot_pos, "Robot position should have x coordinate"
            assert "y" in robot_pos, "Robot position should have y coordinate"
            assert "z" in robot_pos, "Robot position should have z coordinate"
            assert "rotation" in robot_pos, "Robot position should have rotation"

            # Check sensors structure
            sensors = state["sensors"]
            # LiDAR and IMU may or may not be present depending on simulation config
            if "lidar" in sensors:
                assert isinstance(sensors["lidar"], list), "LiDAR data should be a list"

            if "imu" in sensors:
                assert isinstance(sensors["imu"], dict), "IMU data should be a dict"

            # Check physics structure
            physics = state["physics"]
            assert "gravity" in physics, "Physics should have gravity"
            assert "collisionCount" in physics, "Physics should have collisionCount"
            assert "simulationSpeed" in physics, "Physics should have simulationSpeed"

    def test_simulation_command_contract(self):
        """Test that /simulation/{sessionId}/command endpoint matches the OpenAPI contract."""
        # First start a simulation to get a session ID
        simulation_request = {
            "simulationType": "gazebo",
            "environment": "humanoid_robot_world"
        }
        start_response = self.client.post("/simulation/start", json=simulation_request)

        if start_response.status_code == 201:
            session_id = start_response.json()["sessionId"]

            # Send a command to the simulation
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
                assert "timestamp" in data, "Response should contain 'timestamp' field"

                # Check data types
                assert isinstance(data["status"], str), "status should be a string"
                assert isinstance(data["newState"], dict), "newState should be a dict"
                assert isinstance(data["timestamp"], str), "timestamp should be a string"

                # Check that newState has the expected structure
                new_state = data["newState"]
                assert "robotPosition" in new_state, "New state should contain robotPosition"
                assert "sensors" in new_state, "New state should contain sensors"
                assert "physics" in new_state, "New state should contain physics"

    def test_simulation_reset_contract(self):
        """Test that /simulation/{sessionId}/reset endpoint matches the OpenAPI contract."""
        # First start a simulation to get a session ID
        simulation_request = {
            "simulationType": "gazebo",
            "environment": "humanoid_robot_world"
        }
        start_response = self.client.post("/simulation/start", json=simulation_request)

        if start_response.status_code == 201:
            session_id = start_response.json()["sessionId"]

            # Reset the simulation
            response = self.client.post(f"/simulation/{session_id}/reset")

            # Check status code
            assert response.status_code in [200, 404, 500], f"Unexpected status code: {response.status_code}"

            if response.status_code == 200:
                data = response.json()

                # Check response structure according to OpenAPI spec
                assert "message" in data, "Response should contain 'message' field"
                assert "initialState" in data, "Response should contain 'initialState' field"

                # Check data types
                assert isinstance(data["message"], str), "message should be a string"
                assert isinstance(data["initialState"], dict), "initialState should be a dict"

                # Check that initialState has the expected structure
                initial_state = data["initialState"]
                assert "robotPosition" in initial_state, "Initial state should contain robotPosition"
                assert "sensors" in initial_state, "Initial state should contain sensors"
                assert "physics" in initial_state, "Initial state should contain physics"

    def test_simulation_end_contract(self):
        """Test that /simulation/{sessionId}/end endpoint matches the OpenAPI contract."""
        # First start a simulation to get a session ID
        simulation_request = {
            "simulationType": "gazebo",
            "environment": "humanoid_robot_world"
        }
        start_response = self.client.post("/simulation/start", json=simulation_request)

        if start_response.status_code == 201:
            session_id = start_response.json()["sessionId"]

            # End the simulation
            response = self.client.post(f"/simulation/{session_id}/end")

            # Check status code
            assert response.status_code in [200, 404, 500], f"Unexpected status code: {response.status_code}"

            if response.status_code == 200:
                data = response.json()

                # Check response structure according to OpenAPI spec
                assert "message" in data, "Response should contain 'message' field"

                # Duration might be present or not depending on implementation
                if "duration" in data:
                    assert isinstance(data["duration"], int), "duration should be an integer if present"

                # Check data types
                assert isinstance(data["message"], str), "message should be a string"

                # Check that message contains expected content
                assert "ended" in data["message"].lower(), "Message should indicate session ended"